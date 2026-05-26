#!/usr/bin/env python3

import json
import mimetypes
import subprocess
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import urlparse

from path_planner import PathPlanner


SCRIPT_DIR = Path(__file__).resolve().parent
WEB_DIR = SCRIPT_DIR / "web"

MAP_FILE = SCRIPT_DIR / "map_config.json"
STATE_FILE = SCRIPT_DIR / "robot_runtime_state.json"
ROI_FILE = SCRIPT_DIR / "roi_config.json"
IMU_CONFIG_FILE = SCRIPT_DIR / "imu_config.json"

HOST = "0.0.0.0"
PORT = 5000


class ControlPanelHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        return

    # ============================================================
    # UTILIDADES
    # ============================================================

    def send_json(self, data, status=200):
        payload = json.dumps(data, ensure_ascii=False).encode("utf-8")

        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def send_file(self, path):
        if not path.exists() or not path.is_file():
            self.send_json({"ok": False, "error": "Archivo no encontrado"}, status=404)
            return

        content = path.read_bytes()
        mime_type, _ = mimetypes.guess_type(str(path))

        if mime_type is None:
            mime_type = "application/octet-stream"

        self.send_response(200)
        self.send_header("Content-Type", mime_type)
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def read_json_body(self):
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length)

        if not body:
            return {}

        return json.loads(body.decode("utf-8"))

    def load_json_file(self, path, default=None):
        if not path.exists():
            return default

        try:
            if path.stat().st_size == 0:
                return default

            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)

        except json.JSONDecodeError:
            return default

        except Exception:
            return default

    def save_json_file(self, path, data):
        tmp_path = path.with_suffix(path.suffix + ".tmp")

        with open(tmp_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

        tmp_path.replace(path)

    # ============================================================
    # GET
    # ============================================================

    def do_GET(self):
        parsed = urlparse(self.path)

        if parsed.path == "/":
            self.send_file(WEB_DIR / "index.html")
            return

        if parsed.path.startswith("/web/"):
            relative = parsed.path.replace("/web/", "", 1)
            self.send_file(WEB_DIR / relative)
            return

        if parsed.path == "/api/map":
            config = self.load_json_file(MAP_FILE, default={})
            self.send_json(config)
            return

        if parsed.path == "/api/state":
            default_state = {
                "pose": None,
                "running": False,
                "last_action": "Sin estado todavía",
                "roi": {
                    "piece_in_roi": False,
                    "topic": "/arm_camera/piece_in_roi"
                },
                "arm": {
                    "available": False,
                    "last_action": "Pendiente"
                }
            }

            state = self.load_json_file(STATE_FILE, default=default_state)

            if state is None:
                state = default_state

            self.send_json({
                "ok": True,
                "state": state
            })
            return

        if parsed.path == "/api/roi":
            default_roi = {
                "image_width": 640,
                "image_height": 480,
                "roi": {
                    "x": 250,
                    "y": 170,
                    "w": 140,
                    "h": 120
                }
            }

            roi_config = self.load_json_file(ROI_FILE, default=default_roi)

            if roi_config is None:
                roi_config = default_roi

            self.send_json({
                "ok": True,
                "roi_config": roi_config
            })
            return

        if parsed.path == "/api/imu_config":
            default_imu_config = {
                "use_imu_turning": True,
                "imu_yaw_offset_deg": -11.81,
                "imu_yaw_tolerance_deg": 2.0,
                "min_angular_speed": 0.06,
                "max_angular_speed": 0.15,
                "angular_kp": 0.012,
                "angular_command_inverted": True
            }

            imu_config = self.load_json_file(IMU_CONFIG_FILE, default=default_imu_config)

            if imu_config is None:
                imu_config = default_imu_config

            self.send_json({
                "ok": True,
                "imu_config": imu_config
            })
            return

        self.send_json({"ok": False, "error": "Ruta GET no encontrada"}, status=404)

    # ============================================================
    # POST
    # ============================================================

    def do_POST(self):
        parsed = urlparse(self.path)

        if parsed.path == "/api/item/save":
            self.handle_item_save()
            return

        if parsed.path == "/api/item/delete":
            self.handle_item_delete()
            return

        if parsed.path == "/api/plan":
            self.handle_plan()
            return

        if parsed.path == "/api/execute_route":
            self.handle_execute_route()
            return

        if parsed.path == "/api/reset_start":
            self.handle_reset_start()
            return

        if parsed.path == "/api/arm_action":
            self.handle_arm_action()
            return

        if parsed.path == "/api/vacuum_action":
            self.handle_vacuum_action()
            return

        if parsed.path == "/api/start_pick_place":
            self.handle_start_pick_place()
            return

        if parsed.path == "/api/sequence_plan":
            self.handle_sequence_plan()
            return

        if parsed.path == "/api/start_sequence_mission":
            self.handle_start_sequence_mission()
            return

        if parsed.path == "/api/roi/save":
            self.handle_roi_save()
            return

        if parsed.path == "/api/imu_config/save":
            self.handle_imu_config_save()
            return

        if parsed.path == "/api/forbidden/save":
            self.handle_forbidden_save()
            return

        if parsed.path == "/api/forbidden/delete":
            self.handle_forbidden_delete()
            return

        if parsed.path == "/api/background/save":
            self.handle_background_save()
            return

        self.send_json({"ok": False, "error": "Ruta POST no encontrada"}, status=404)

    # ============================================================
    # Función para manejar pick & place
    # ============================================================
    def handle_start_pick_place(self):
        try:
            body = self.read_json_body()
            storage = str(body.get("storage", "")).strip()
            pantry = str(body.get("pantry", "")).strip()
            origin = str(body.get("origin", "START")).strip()
            # ... resto del código de lanzamiento
        except Exception as e:
            self.send_json({"ok": False, "error": str(e)}, status=400)

    def handle_sequence_plan(self):
        try:
            body = self.read_json_body()
            selected_storages = body.get("selected_storages", [])

            if not isinstance(selected_storages, list):
                raise ValueError("selected_storages debe ser una lista")

            planner = PathPlanner(MAP_FILE)
            plan = planner.build_sequence_plan(selected_storages)

            self.send_json({
                "ok": True,
                "plan": plan
            })

        except Exception as e:
            self.send_json({"ok": False, "error": str(e)}, status=400)

    def handle_start_sequence_mission(self):
        try:
            body = self.read_json_body()

            selected_storages = body.get("selected_storages", [])

            if not isinstance(selected_storages, list):
                raise ValueError("selected_storages debe ser una lista")

            if not selected_storages:
                raise ValueError("No hay almacenes seleccionados")

            linear_speed = float(body.get("linear_speed", 0.03))
            angular_speed = float(body.get("angular_speed", 0.15))
            slow_pick_speed = float(body.get("slow_pick_speed", 0.015))

            planner = PathPlanner(MAP_FILE)

            # Esto valida toda la misión antes de lanzar nada.
            plan = planner.build_sequence_plan(selected_storages)

            mission_payload = {
                "selected_storages": selected_storages,
                "plan": plan,
                "linear_speed": linear_speed,
                "angular_speed": angular_speed,
                "slow_pick_speed": slow_pick_speed
            }

            mission_dir = SCRIPT_DIR / "runtime_missions"
            mission_dir.mkdir(parents=True, exist_ok=True)

            mission_file = mission_dir / f"mission_{int(time.time())}.json"
            self.save_json_file(mission_file, mission_payload)

            log_path = SCRIPT_DIR / "logs" / "mission_fsm.log"
            log_path.parent.mkdir(parents=True, exist_ok=True)

            cmd = [
                "bash",
                "-lc",
                (
                    "cd /home/jetson/robot_ws/src/diff_car/scripts && "
                    "source /opt/ros/humble/setup.bash && "
                    "source /home/jetson/robot_ws/install/setup.bash && "
                    "source /home/jetson/brazo_ws/install/setup.bash && "
                    "source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash && "
                    f"python3 mission_fsm.py "
                    f"--mission-json {mission_file}"
                )
            ]

            with open(log_path, "a", encoding="utf-8") as log_file:
                subprocess.Popen(
                    cmd,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                )

            summary_lines = []

            for leg in plan["legs"]:
                summary_lines.append(
                    f"{leg['label']}: " + " → ".join(leg["node_names"])
                )

            self.send_json({
                "ok": True,
                "message": (
                    "Misión secuencial lanzada\n\n"
                    "Almacenes seleccionados: "
                    + ", ".join(selected_storages)
                    + "\n\nRutas:\n"
                    + "\n".join(summary_lines)
                ),
                "mission_file": str(mission_file),
                "plan": plan
            })

        except Exception as e:
            self.send_json({"ok": False, "error": str(e)}, status=400)


    def ensure_mission_sequence(self, config):
        if "mission_sequence" not in config:
            config["mission_sequence"] = []

        if not isinstance(config["mission_sequence"], list):
            raise ValueError("mission_sequence debe ser una lista")

        return config["mission_sequence"]


    def get_mission_item_by_storage(self, config, storage):
        sequence = self.ensure_mission_sequence(config)

        for item in sequence:
            if str(item.get("storage", "")).strip() == str(storage).strip():
                return item

        raise ValueError(f"No existe storage en mission_sequence: {storage}")


    def ensure_mission_item_lists(self, item):
        for key in [
            "from_previous_transit",
            "storage_to_pantry_transit",
            "pantry_to_home_transit"
        ]:
            if key not in item:
                item[key] = []

            if not isinstance(item[key], list):
                raise ValueError(f"{key} debe ser una lista")


    def normalize_transit_key(self, transit_name):
        transit_name = str(transit_name).strip()

        if transit_name.startswith("T_"):
            return transit_name.replace("T_", "", 1)

        return transit_name


    def remove_transit_from_mission_sequence(self, config, transit_name):
        sequence = self.ensure_mission_sequence(config)

        raw_name = self.normalize_transit_key(transit_name)
        node_name = f"T_{raw_name}"

        for item in sequence:
            self.ensure_mission_item_lists(item)

            for key in [
                "from_previous_transit",
                "storage_to_pantry_transit",
                "pantry_to_home_transit"
            ]:
                item[key] = [
                    value for value in item[key]
                    if value not in [raw_name, node_name]
                ]


    def assign_transit_to_mission_sequence(
        self,
        config,
        transit_name,
        mission_storage,
        transit_role,
        transit_order
    ):
        allowed_roles = [
            "from_previous_transit",
            "storage_to_pantry_transit",
            "pantry_to_home_transit"
        ]

        if transit_role not in allowed_roles:
            raise ValueError(f"Rol transit no válido: {transit_role}")

        item = self.get_mission_item_by_storage(config, mission_storage)
        self.ensure_mission_item_lists(item)

        raw_name = self.normalize_transit_key(transit_name)

        # Primero eliminamos el transit de cualquier lista para que no quede duplicado.
        self.remove_transit_from_mission_sequence(config, raw_name)

        target_list = item[transit_role]

        order = max(0, int(transit_order) - 1)

        if order >= len(target_list):
            target_list.append(raw_name)
        else:
            target_list.insert(order, raw_name)

    # ============================================================
    # HANDLERS MAPA
    # ============================================================

    def handle_item_save(self):
        try:
            body = self.read_json_body()

            section = body["type"]
            name = str(body["name"]).strip()
            data = body["data"]

            if not name:
                raise ValueError("Nombre vacío")

            config = self.load_json_file(MAP_FILE, default={})

            if section not in config:
                raise ValueError(f"Sección no válida: {section}")

            if section == "almacenes":
                data["w"] = config["fixed_sizes"]["almacen"]["x_size_mm"]
                data["h"] = config["fixed_sizes"]["almacen"]["y_size_mm"]

            elif section == "despensas":
                data["w"] = config["fixed_sizes"]["despensa"]["x_size_mm"]
                data["h"] = config["fixed_sizes"]["despensa"]["y_size_mm"]

            elif section == "transit":
                data.pop("theta_deg", None)

                # Campos opcionales que vienen del HTML para asignar el transit
                # a mission_sequence.
                mission_storage = str(body.get("mission_storage", "")).strip()
                transit_role = str(body.get("transit_role", "")).strip()
                transit_order = int(body.get("transit_order", 999))

                # Solo X/Y deben quedarse dentro de config["transit"].
                data = {
                    "x": float(data["x"]),
                    "y": float(data["y"])
                }

                config[section][name] = data

                if mission_storage and transit_role:
                    self.assign_transit_to_mission_sequence(
                        config=config,
                        transit_name=name,
                        mission_storage=mission_storage,
                        transit_role=transit_role,
                        transit_order=transit_order
                    )

                self.save_json_file(MAP_FILE, config)
                self.send_json({"ok": True})
                return

            elif section == "forbidden_zones":
                data.pop("theta_deg", None)

                if "w" not in data or "h" not in data:
                    raise ValueError("La zona prohibida necesita tamaño W/H")

                data["w"] = float(data["w"])
                data["h"] = float(data["h"])

                if data["w"] <= 0 or data["h"] <= 0:
                    raise ValueError("La zona prohibida debe tener tamaño positivo")

            config[section][name] = data
            self.save_json_file(MAP_FILE, config)

            self.send_json({"ok": True})

        except Exception as e:
            self.send_json({"ok": False, "error": str(e)}, status=400)

    def handle_item_delete(self):
        try:
            body = self.read_json_body()

            section = body["type"]
            name = str(body["name"]).strip()

            if not name:
                raise ValueError("Nombre vacío")

            config = self.load_json_file(MAP_FILE, default={})

            if section not in config:
                raise ValueError(f"Sección no válida: {section}")

            if name in config[section]:
                del config[section][name]

            if section == "transit":
                self.remove_transit_from_mission_sequence(config, name)

            self.save_json_file(MAP_FILE, config)

            self.send_json({"ok": True})

        except Exception as e:
            self.send_json({"ok": False, "error": str(e)}, status=400)

    def handle_plan(self):
        try:
            body = self.read_json_body()

            origin = body["origin"]
            destination = body["destination"]

            planner = PathPlanner(MAP_FILE)
            plan = planner.plan(origin, destination)

            self.send_json({
                "ok": True,
                "plan": plan
            })

        except Exception as e:
            self.send_json({"ok": False, "error": str(e)}, status=400)

    # ============================================================
    # HANDLERS ROBOT / BASE
    # ============================================================

    def handle_execute_route(self):
        try:
            body = self.read_json_body()

            origin = body.get("origin")
            destination = body.get("destination")
            linear_speed = float(body.get("linear_speed", 0.03))
            angular_speed = float(body.get("angular_speed", 0.15))

            if not origin or not destination:
                raise ValueError("Falta origin o destination")

            cmd = [
                "bash",
                "-lc",
                (
                    "cd /home/jetson/robot_ws/src/diff_car/scripts && "
                    "source /opt/ros/humble/setup.bash && "
                    "source /home/jetson/robot_ws/install/setup.bash && "
                    f"python3 base_robot.py --route {origin} {destination} "
                    f"--linear-speed {linear_speed} "
                    f"--angular-speed {angular_speed} "
                    "--yes"
                )
            ]

            subprocess.Popen(cmd)

            self.send_json({
                "ok": True,
                "message": f"Ejecutando ruta real: {origin} -> {destination}"
            })

        except Exception as e:
            self.send_json({"ok": False, "error": str(e)}, status=400)

    def handle_reset_start(self):
        try:
            cmd = [
                "bash",
                "-lc",
                (
                    "cd /home/jetson/robot_ws/src/diff_car/scripts && "
                    "source /opt/ros/humble/setup.bash && "
                    "source /home/jetson/robot_ws/install/setup.bash && "
                    "python3 base_robot.py --reset-start"
                )
            ]

            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=15
            )

            if result.returncode != 0:
                raise RuntimeError(result.stderr)

            self.send_json({
                "ok": True,
                "message": "START reseteado: pose lógica, IMU y reset base enviados"
            })

        except Exception as e:
            self.send_json({"ok": False, "error": str(e)}, status=400)

    # ============================================================
    # HANDLERS BRAZO / VACUUM
    # ============================================================

    def handle_arm_action(self):
        try:
            body = self.read_json_body()
            action = str(body.get("action", "")).strip().upper()

            if not action:
                raise ValueError("Falta acción de brazo")

            cmd = [
                "bash",
                "-lc",
                (
                    "cd ~/robot_ws/src/diff_car/scripts && "
                    "source /opt/ros/humble/setup.bash && "
                    "source ~/brazo_ws/install/setup.bash && "
                    f"python3 arm_actions.py {action} --vacuum-mode none"
                )
            ]

            subprocess.Popen(cmd)
            self.send_json({"ok": True, "message": f"Acción de brazo '{action}' lanzada"})

        except Exception as e:
            self.send_json({"ok": False, "error": str(e)}, status=400)


    def handle_vacuum_action(self):
        try:
            body = self.read_json_body()
            command = str(body.get("command", "")).strip().upper()

            if command not in ["ON", "OFF"]:
                raise ValueError("Comando de vacuum inválido, debe ser 'ON' o 'OFF'")

            self.send_ros_string("/robot/vacuum_command", command)
            self.send_json({"ok": True, "message": f"Vacuum '{command}' enviado"})

        except Exception as e:
            self.send_json({"ok": False, "error": str(e)}, status=400)

    def handle_vacuum_action(self):
        try:
            body = self.read_json_body()
            action = str(body.get("action", "")).lower()

            allowed_actions = ["on", "off", "status"]

            if action not in allowed_actions:
                raise ValueError(f"Acción vacuum no válida: {action}")

            command = action.upper()

            cmd = [
                "bash",
                "-lc",
                (
                    "cd /home/jetson/robot_ws/src/diff_car/scripts && "
                    "source /opt/ros/humble/setup.bash && "
                    "source /home/jetson/robot_ws/install/setup.bash && "
                    f"python3 send_vacuum_command.py {command}"
                )
            ]

            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=8
            )

            if result.returncode != 0:
                raise RuntimeError(result.stderr)

            self.send_json({
                "ok": True,
                "message": f"Comando enviado al vacuum: {command}"
            })

        except Exception as e:
            self.send_json({"ok": False, "error": str(e)}, status=400)

    # ============================================================
    # HANDLERS ROI / FSM
    # ============================================================

    def handle_roi_save(self):
        try:
            body = self.read_json_body()

            roi_config = {
                "image_width": int(body.get("image_width", 640)),
                "image_height": int(body.get("image_height", 480)),
                "roi": {
                    "x": int(body["roi"]["x"]),
                    "y": int(body["roi"]["y"]),
                    "w": int(body["roi"]["w"]),
                    "h": int(body["roi"]["h"])
                }
            }

            roi = roi_config["roi"]

            roi["x"] = max(0, min(roi["x"], roi_config["image_width"] - 1))
            roi["y"] = max(0, min(roi["y"], roi_config["image_height"] - 1))
            roi["w"] = max(1, min(roi["w"], roi_config["image_width"] - roi["x"]))
            roi["h"] = max(1, min(roi["h"], roi_config["image_height"] - roi["y"]))

            self.save_json_file(ROI_FILE, roi_config)

            self.send_json({
                "ok": True,
                "message": "ROI guardado",
                "roi_config": roi_config
            })

        except Exception as e:
            self.send_json({"ok": False, "error": str(e)}, status=400)

    def handle_start_pick_place(self):
        try:
            body = self.read_json_body()

            storage = str(body.get("storage", "")).strip()
            pantry = str(body.get("pantry", "")).strip()
            origin = str(body.get("origin", "START")).strip()

            linear_speed = float(body.get("linear_speed", 0.03))
            angular_speed = float(body.get("angular_speed", 0.15))
            slow_pick_speed = float(body.get("slow_pick_speed", 0.015))

            if not storage:
                raise ValueError("Falta storage")

            if not pantry:
                raise ValueError("Falta pantry")

            log_path = SCRIPT_DIR / "logs" / "mission_fsm.log"
            log_path.parent.mkdir(parents=True, exist_ok=True)

            cmd = [
                "bash",
                "-lc",
                (
                    "cd /home/jetson/robot_ws/src/diff_car/scripts && "
                    "source /opt/ros/humble/setup.bash && "
                    "source /home/jetson/robot_ws/install/setup.bash && "
                    "source /home/jetson/brazo_ws/install/setup.bash && "
                    "source /home/jetson/yahboomcar_ros2_ws/software/library_ws/install/setup.bash && "
                    f"python3 mission_fsm.py "
                    f"--storage {storage} "
                    f"--pantry {pantry} "
                    f"--origin {origin} "
                    f"--linear-speed {linear_speed} "
                    f"--angular-speed {angular_speed} "
                    f"--slow-pick-speed {slow_pick_speed}"
                )
            ]

            with open(log_path, "a", encoding="utf-8") as log_file:
                subprocess.Popen(
                    cmd,
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                )

            self.send_json({
                "ok": True,
                "message": (
                    "Misión automática lanzada\n"
                    f"Origen: {origin}\n"
                    f"Almacén: AA_{storage}\n"
                    f"Despensa: AD_{pantry}\n\n"
                    "Secuencia:\n"
                    f"{origin} → AA_{storage}\n"
                    "SLOW_PICK_ROI\n"
                    "VACUUM ON\n"
                    "PICK\n"
                    "HOME\n"
                    f"AA_{storage} → AD_{pantry}\n"
                    "PLACE\n"
                    "VACUUM OFF\n"
                    "HOME"
                )
            })

        except Exception as e:
            self.send_json({"ok": False, "error": str(e)}, status=400)

    def handle_imu_config_save(self):
        try:
            body = self.read_json_body()

            imu_config = {
                "use_imu_turning": bool(body.get("use_imu_turning", True)),
                "imu_yaw_offset_deg": float(body.get("imu_yaw_offset_deg", -11.81)),
                "imu_yaw_tolerance_deg": float(body.get("imu_yaw_tolerance_deg", 2.0)),
                "min_angular_speed": float(body.get("min_angular_speed", 0.06)),
                "max_angular_speed": float(body.get("max_angular_speed", 0.15)),
                "angular_kp": float(body.get("angular_kp", 0.012)),
                "angular_command_inverted": bool(body.get("angular_command_inverted", True))
            }

            if imu_config["imu_yaw_tolerance_deg"] < 0.5:
                imu_config["imu_yaw_tolerance_deg"] = 0.5

            if imu_config["min_angular_speed"] < 0.01:
                imu_config["min_angular_speed"] = 0.01

            if imu_config["max_angular_speed"] < imu_config["min_angular_speed"]:
                imu_config["max_angular_speed"] = imu_config["min_angular_speed"]

            self.save_json_file(IMU_CONFIG_FILE, imu_config)

            self.send_json({
                "ok": True,
                "message": "Configuración IMU guardada",
                "imu_config": imu_config
            })

        except Exception as e:
            self.send_json({"ok": False, "error": str(e)}, status=400)

    def handle_forbidden_save(self):
        try:
            body = self.read_json_body()

            name = str(body.get("name", "")).strip()

            if not name:
                raise ValueError("Nombre de zona prohibida vacío")

            x = float(body["x"])
            y = float(body["y"])
            w = float(body["w"])
            h = float(body["h"])

            if w <= 0 or h <= 0:
                raise ValueError("La zona prohibida debe tener anchura y altura positivas")

            config = self.load_json_file(MAP_FILE, default={})

            if "forbidden_zones" not in config:
                config["forbidden_zones"] = {}

            config["forbidden_zones"][name] = {
                "x": x,
                "y": y,
                "w": w,
                "h": h
            }

            self.save_json_file(MAP_FILE, config)

            self.send_json({
                "ok": True,
                "message": f"Zona prohibida guardada: {name}",
                "map": config
            })

        except Exception as e:
            self.send_json({"ok": False, "error": str(e)}, status=400)


    def handle_forbidden_delete(self):
        try:
            body = self.read_json_body()

            name = str(body.get("name", "")).strip()

            if not name:
                raise ValueError("Nombre de zona prohibida vacío")

            config = self.load_json_file(MAP_FILE, default={})

            if "forbidden_zones" in config and name in config["forbidden_zones"]:
                del config["forbidden_zones"][name]

            self.save_json_file(MAP_FILE, config)

            self.send_json({
                "ok": True,
                "message": f"Zona prohibida eliminada: {name}",
                "map": config
            })

        except Exception as e:
            self.send_json({"ok": False, "error": str(e)}, status=400)

    def handle_background_save(self):
        try:
            body = self.read_json_body()

            config = self.load_json_file(MAP_FILE, default={})

            config["background_image"] = {
                "enabled": bool(body.get("enabled", False)),
                "url": str(body.get("url", "/web/map_background.png")),
                "offset_x": float(body.get("offset_x", 0)),
                "offset_y": float(body.get("offset_y", 0)),
                "scale": float(body.get("scale", 1.0)),
                "opacity": float(body.get("opacity", 0.35))
            }

            self.save_json_file(MAP_FILE, config)

            self.send_json({
                "ok": True,
                "message": "Fondo de mapa guardado",
                "map": config
            })

        except Exception as e:
            self.send_json({"ok": False, "error": str(e)}, status=400)


def main():
    print("CONTROL_PANEL_VERSION_OK_GET_2026")

    server = HTTPServer((HOST, PORT), ControlPanelHandler)

    print("\nPanel iniciado:")
    print(f"  Local : http://127.0.0.1:{PORT}")
    print(f"  Red   : http://IP_DE_LA_JETSON:{PORT}")
    print("\nCtrl+C para salir.\n")

    try:
        server.serve_forever()

    except KeyboardInterrupt:
        print("\nCerrando panel...")

    finally:
        server.server_close()


if __name__ == "__main__":
    main()