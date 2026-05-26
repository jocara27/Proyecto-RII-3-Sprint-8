#!/usr/bin/env python3

import json
import math
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
MAP_FILE = SCRIPT_DIR / "map_config.json"


def distance(a, b):
    return math.hypot(float(a["x"]) - float(b["x"]), float(a["y"]) - float(b["y"]))


def inflate_rect(rect, margin):
    return {
        "x": float(rect["x"]) - margin,
        "y": float(rect["y"]) - margin,
        "w": float(rect["w"]) + 2.0 * margin,
        "h": float(rect["h"]) + 2.0 * margin
    }


def point_inside_rect(px, py, rect):
    return (
        float(rect["x"]) <= px <= float(rect["x"]) + float(rect["w"])
        and float(rect["y"]) <= py <= float(rect["y"]) + float(rect["h"])
    )


def segment_intersects_rect(p1, p2, rect, step_mm=10.0):
    """
    Comprobación conservadora por muestreo.
    Se usa para detectar si una línea directa cruza una zona prohibida.
    """
    length = distance(p1, p2)

    if length < 1e-6:
        return point_inside_rect(float(p1["x"]), float(p1["y"]), rect)

    steps = max(2, int(length / step_mm))

    for i in range(steps + 1):
        t = i / steps

        x = float(p1["x"]) + (float(p2["x"]) - float(p1["x"])) * t
        y = float(p1["y"]) + (float(p2["y"]) - float(p1["y"])) * t

        if point_inside_rect(x, y, rect):
            return True

    return False


class PathPlanner:
    def __init__(self, map_file=MAP_FILE):
        self.map_file = Path(map_file)
        self.config = self.load_map()

    def load_map(self):
        with open(self.map_file, "r", encoding="utf-8") as f:
            return json.load(f)

    # ============================================================
    # MÁRGENES ROBOT / DEBUG VISUAL
    # ============================================================

    def get_robot_safety_margin(self):
        robot = self.config.get("robot", {})
        return float(robot.get("safety_margin_mm", 50.0))

    def get_robot_radius_margin(self):
        robot = self.config.get("robot", {})

        length_x = float(robot.get("length_x_mm", 230.0))
        width_y = float(robot.get("width_y_mm", 250.0))
        safety = float(robot.get("safety_margin_mm", 50.0))

        half_diag = math.hypot(length_x / 2.0, width_y / 2.0)

        return half_diag + safety

    def make_obstacle(self, name, rect, margin, kind):
        real = {
            "x": float(rect["x"]),
            "y": float(rect["y"]),
            "w": float(rect["w"]),
            "h": float(rect["h"]),
            "name": name,
            "kind": kind,
            "inflated": False,
            "margin": 0.0
        }

        inflated = inflate_rect(rect, margin)
        inflated["name"] = name
        inflated["kind"] = kind
        inflated["inflated"] = True
        inflated["margin"] = float(margin)

        return real, inflated

    def get_debug_obstacles(self):
        """
        Se conserva para que el frontend pueda seguir pintando/debugueando
        obstáculos si lo necesita.
        """
        debug = []
        safety_margin = self.get_robot_safety_margin()

        for section in ["almacenes", "despensas"]:
            for name, rect in self.config.get(section, {}).items():
                real, inflated = self.make_obstacle(
                    name=f"{section}.{name}",
                    rect=rect,
                    margin=safety_margin,
                    kind=section
                )
                debug.append(real)
                debug.append(inflated)

        for name, rect in self.config.get("forbidden_zones", {}).items():
            real, _ = self.make_obstacle(
                name=f"forbidden_zones.{name}",
                rect=rect,
                margin=0.0,
                kind="forbidden_zone"
            )
            debug.append(real)

        return debug

    # ============================================================
    # NODOS
    # ============================================================

    def transit_node_name(self, name):
        """
        Nombre de nodo para transit.

        Si en map_config.json guardas:
          "transit": { "Palm2_Desp2_1": {...} }

        El nodo será:
          T_Palm2_Desp2_1

        Si por error/compatibilidad guardas la clave como T_xxx,
        no duplica el prefijo.
        """
        name = str(name).strip()

        if name.startswith("T_"):
            return name

        return f"T_{name}"

    def get_nodes(self):
        nodes = {}

        start = self.config["start"]
        nodes["START"] = {
            "x": float(start["x"]),
            "y": float(start["y"]),
            "theta_deg": float(start.get("theta_deg", 0.0)),
            "type": "start"
        }

        for name, point in self.config.get("approach_almacenes", {}).items():
            nodes[f"AA_{name}"] = {
                "x": float(point["x"]),
                "y": float(point["y"]),
                "theta_deg": float(point.get("theta_deg", 0.0)),
                "type": "approach_almacen"
            }

        for name, point in self.config.get("approach_despensas", {}).items():
            nodes[f"AD_{name}"] = {
                "x": float(point["x"]),
                "y": float(point["y"]),
                "theta_deg": float(point.get("theta_deg", 0.0)),
                "type": "approach_despensa"
            }

        for name, point in self.config.get("transit", {}).items():
            nodes[self.transit_node_name(name)] = {
                "x": float(point["x"]),
                "y": float(point["y"]),
                "type": "transit"
            }

        return nodes

    def resolve_transit_node(self, transit_name, nodes):
        """
        Acepta nombres flexibles en mission_sequence:
          "Palm2_Desp2_1"
          "T_Palm2_Desp2_1"

        Y devuelve el nombre real del nodo:
          "T_Palm2_Desp2_1"
        """
        raw = str(transit_name).strip()

        candidates = [
            raw,
            self.transit_node_name(raw),
        ]

        for candidate in candidates:
            if candidate in nodes:
                return candidate

        raise ValueError(f"Transit no existe en map_config.json: {transit_name}")

    def route_from_names(self, names):
        nodes = self.get_nodes()
        route = []

        for name in names:
            if name not in nodes:
                raise ValueError(f"Nodo no existe: {name}")

            node = nodes[name].copy()
            node["name"] = name
            route.append(node)

        return route

    # ============================================================
    # VALIDACIÓN GEOMÉTRICA
    # ============================================================

    def point_inside_board(self, point):
        board = self.config["board"]

        return (
            0.0 <= float(point["x"]) <= float(board["height_x_mm"])
            and 0.0 <= float(point["y"]) <= float(board["width_y_mm"])
        )

    def get_forbidden_zones(self):
        zones = []

        for name, rect in self.config.get("forbidden_zones", {}).items():
            zone = {
                "x": float(rect["x"]),
                "y": float(rect["y"]),
                "w": float(rect["w"]),
                "h": float(rect["h"]),
                "name": name
            }
            zones.append(zone)

        return zones

    def validate_segment_or_raise(self, p1, p2):
        if not self.point_inside_board(p1):
            raise ValueError(f"El punto {p1['name']} está fuera del tablero")

        if not self.point_inside_board(p2):
            raise ValueError(f"El punto {p2['name']} está fuera del tablero")

        for zone in self.get_forbidden_zones():
            if segment_intersects_rect(p1, p2, zone):
                raise RuntimeError(
                    f"La trayectoria entre {p1['name']} y {p2['name']} "
                    f"cruza zona prohibida {zone['name']}"
                )

    def validate_route_or_raise(self, route):
        if len(route) < 2:
            return

        for i in range(len(route) - 1):
            self.validate_segment_or_raise(route[i], route[i + 1])

    def route_distance(self, route):
        total = 0.0

        for i in range(len(route) - 1):
            total += distance(route[i], route[i + 1])

        return total

    # ============================================================
    # MISSION_SEQUENCE
    # ============================================================

    def get_mission_sequence(self):
        return self.config.get("mission_sequence", [])

    def storage_node(self, storage_name):
        return f"AA_{storage_name}"

    def pantry_node(self, pantry_name):
        return f"AD_{pantry_name}"

    def get_sequence_item_by_storage(self, storage_name):
        for item in self.get_mission_sequence():
            if str(item.get("storage", "")).strip() == str(storage_name).strip():
                return item

        raise ValueError(f"storage no existe en mission_sequence: {storage_name}")

    def get_sequence_index_by_storage(self, storage_name):
        for index, item in enumerate(self.get_mission_sequence()):
            if str(item.get("storage", "")).strip() == str(storage_name).strip():
                return index

        raise ValueError(f"storage no existe en mission_sequence: {storage_name}")

    def get_previous_pantry_node_for_index(self, index):
        if index <= 0:
            return "START"

        previous = self.get_mission_sequence()[index - 1]
        return self.pantry_node(previous["pantry"])

    def normalize_transit_list(self, transit_list, nodes):
        result = []

        for transit_name in transit_list or []:
            result.append(self.resolve_transit_node(transit_name, nodes))

        return result

    def explicit_route_names(self, origin, destination):
        """
        Compatibilidad con base_robot.py:
        base_robot sigue llamando planner.plan(origin, destination).

        Aquí intentamos encontrar si ese tramo corresponde a una regla
        de mission_sequence. Si no hay regla, se valida como tramo directo.
        """
        origin = str(origin).strip()
        destination = str(destination).strip()

        nodes = self.get_nodes()

        if origin not in nodes:
            raise ValueError(f"Origen no existe: {origin}")

        if destination not in nodes:
            raise ValueError(f"Destino no existe: {destination}")

        sequence = self.get_mission_sequence()

        for index, item in enumerate(sequence):
            storage = item["storage"]
            pantry = item["pantry"]

            storage_node = self.storage_node(storage)
            pantry_node = self.pantry_node(pantry)
            previous_origin = self.get_previous_pantry_node_for_index(index)

            # Tramo desde START o despensa anterior hasta almacén actual.
            # Solo usa from_previous_transit si el origen coincide con el
            # punto anterior natural de la secuencia.
            if origin == previous_origin and destination == storage_node:
                transits = self.normalize_transit_list(
                    item.get("from_previous_transit", []),
                    nodes
                )
                return [origin] + transits + [destination]

            # Tramo almacén actual -> despensa actual.
            if origin == storage_node and destination == pantry_node:
                transits = self.normalize_transit_list(
                    item.get("storage_to_pantry_transit", []),
                    nodes
                )
                return [origin] + transits + [destination]

            # Tramo despensa actual -> HOME/START.
            if origin == pantry_node and destination == "START":
                transits = self.normalize_transit_list(
                    item.get("pantry_to_home_transit", []),
                    nodes
                )
                return [origin] + transits + [destination]

        # Si no hay regla explícita, el tramo es directo.
        return [origin, destination]

    def plan(self, origin, destination):
        names = self.explicit_route_names(origin, destination)
        route = self.route_from_names(names)

        self.validate_route_or_raise(route)

        return {
            "origin": origin,
            "destination": destination,
            "route": route,
            "distance_mm": self.route_distance(route),
            "obstacles": self.get_debug_obstacles()
        }

    # ============================================================
    # MISIÓN COMPLETA
    # ============================================================

    def build_leg(self, origin, destination, transit_names=None, label=""):
        nodes = self.get_nodes()
        transit_nodes = self.normalize_transit_list(transit_names or [], nodes)

        names = [origin] + transit_nodes + [destination]
        route = self.route_from_names(names)

        self.validate_route_or_raise(route)

        return {
            "label": label,
            "origin": origin,
            "destination": destination,
            "node_names": names,
            "route": route,
            "distance_mm": self.route_distance(route)
        }

    def build_sequence_plan(self, selected_storages):
        """
        Genera una misión completa a partir de los almacenes seleccionados
        desde el HTML.

        selected_storages:
          ["Palm1", "Palm2", "Palm3"]

        Devuelve legs explícitas. Cada leg trae node_names para que
        mission_fsm.py pueda ejecutar segmento a segmento sin tocar
        base_robot.py.
        """
        selected_storages = [
            str(s).strip()
            for s in selected_storages
            if str(s).strip()
        ]

        if not selected_storages:
            raise ValueError("No hay almacenes seleccionados para la misión")

        sequence = self.get_mission_sequence()

        if not sequence:
            raise ValueError("map_config.json no tiene mission_sequence")

        selected_items = []

        for storage in selected_storages:
            item = self.get_sequence_item_by_storage(storage)
            selected_items.append(item)

        legs = []
        current_node = "START"

        for idx, item in enumerate(selected_items):
            storage = item["storage"]
            pantry = item["pantry"]

            storage_node = self.storage_node(storage)
            pantry_node = self.pantry_node(pantry)

            # Llegada al almacén.
            # Si es el primer almacén seleccionado y venimos de START,
            # no usamos from_previous_transit salvo que ese almacén sea
            # realmente el primer bloque de mission_sequence.
            if current_node == "START":
                sequence_index = self.get_sequence_index_by_storage(storage)
                previous_origin = self.get_previous_pantry_node_for_index(sequence_index)

                if previous_origin == "START":
                    from_previous = item.get("from_previous_transit", [])
                else:
                    from_previous = []
            else:
                from_previous = item.get("from_previous_transit", [])

            legs.append(
                self.build_leg(
                    origin=current_node,
                    destination=storage_node,
                    transit_names=from_previous,
                    label=f"GO_TO_STORAGE {storage}"
                )
            )

            # Almacén -> despensa.
            legs.append(
                self.build_leg(
                    origin=storage_node,
                    destination=pantry_node,
                    transit_names=item.get("storage_to_pantry_transit", []),
                    label=f"GO_TO_PANTRY {storage}->{pantry}"
                )
            )

            current_node = pantry_node

        # Última despensa -> START.
        last_item = selected_items[-1]

        legs.append(
            self.build_leg(
                origin=current_node,
                destination="START",
                transit_names=last_item.get("pantry_to_home_transit", []),
                label="RETURN_HOME"
            )
        )

        total_distance = sum(float(leg["distance_mm"]) for leg in legs)

        return {
            "selected_storages": selected_storages,
            "legs": legs,
            "distance_mm": total_distance,
            "obstacles": self.get_debug_obstacles()
        }


def main():
    planner = PathPlanner()

    print(json.dumps(
        planner.build_sequence_plan(["Palm1"]),
        indent=2,
        ensure_ascii=False
    ))


if __name__ == "__main__":
    main()