#!/usr/bin/env python3

import argparse
import json
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from transitions import Machine, State


SCRIPT_DIR = "/home/jetson/robot_ws/src/diff_car/scripts"

BASE_SOURCE = (
    "source /opt/ros/humble/setup.bash && "
    "source /home/jetson/robot_ws/install/setup.bash && "
)

ARM_COMMAND_TOPIC = "/robot/arm_command"
ARM_STATUS_TOPIC = "/robot/arm_status"
VACUUM_COMMAND_TOPIC = "/robot/vacuum_command"

DEFAULT_LINEAR_SPEED = 0.03
DEFAULT_ANGULAR_SPEED = 0.15
DEFAULT_SLOW_PICK_SPEED = 0.015


class MissionFSM(Node):
    states = [
        State(name="IDLE"),
        State(name="RESET_START"),
        State(name="ARM_HOME_INIT"),
        State(name="GO_TO_STORAGE"),
        State(name="SLOW_PICK_ROI"),
        State(name="VACUUM_ON"),
        State(name="ARM_PICK"),
        State(name="ARM_HOME_AFTER_PICK"),
        State(name="GO_TO_PANTRY"),
        State(name="ARM_PLACE"),
        State(name="VACUUM_OFF"),
        State(name="ARM_HOME_AFTER_PLACE"),
        State(name="DONE"),
        State(name="ERROR"),
    ]

    def __init__(
        self,
        storage_name=None,
        pantry_name=None,
        origin_name="START",
        linear_speed=DEFAULT_LINEAR_SPEED,
        angular_speed=DEFAULT_ANGULAR_SPEED,
        slow_pick_speed=DEFAULT_SLOW_PICK_SPEED,
        mission_plan=None,
    ):
        super().__init__("mission_fsm_node")

        self.mission_plan = mission_plan
        self.sequence_mode = mission_plan is not None

        self.current_cycle_index = 0
        self.current_cycle = None

        self.storage_name = storage_name
        self.pantry_name = pantry_name
        self.origin_name = origin_name

        if self.sequence_mode:
            self.storage_node = None
            self.pantry_node = None
        else:
            self.storage_node = f"AA_{storage_name}"
            self.pantry_node = f"AD_{pantry_name}"

        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.slow_pick_speed = slow_pick_speed

        self.last_arm_status = ""
        self.arm_status_time = 0.0

        self.arm_command_pub = self.create_publisher(
            String,
            ARM_COMMAND_TOPIC,
            10
        )

        self.vacuum_command_pub = self.create_publisher(
            String,
            VACUUM_COMMAND_TOPIC,
            10
        )

        self.arm_status_sub = self.create_subscription(
            String,
            ARM_STATUS_TOPIC,
            self.arm_status_callback,
            10
        )

        self.machine = Machine(
            model=self,
            states=MissionFSM.states,
            initial="IDLE",
            ignore_invalid_triggers=True,
            after_state_change="print_state",
        )

        self.machine.add_transition(
            "start",
            "IDLE",
            "RESET_START",
            after="reset_start_if_needed"
        )

        self.machine.add_transition(
            "reset_start_done",
            "RESET_START",
            "ARM_HOME_INIT",
            after="arm_home_init"
        )

        self.machine.add_transition(
            "home_init_done",
            "ARM_HOME_INIT",
            "GO_TO_STORAGE",
            after="go_to_storage"
        )

        self.machine.add_transition(
            "storage_reached",
            "GO_TO_STORAGE",
            "SLOW_PICK_ROI",
            after="slow_pick_roi"
        )

        self.machine.add_transition(
            "slow_pick_done",
            "SLOW_PICK_ROI",
            "VACUUM_ON",
            after="vacuum_on"
        )

        self.machine.add_transition(
            "vacuum_on_done",
            "VACUUM_ON",
            "ARM_PICK",
            after="arm_pick"
        )

        self.machine.add_transition(
            "pick_done",
            "ARM_PICK",
            "ARM_HOME_AFTER_PICK",
            after="arm_home_after_pick"
        )

        self.machine.add_transition(
            "home_after_pick_done",
            "ARM_HOME_AFTER_PICK",
            "GO_TO_PANTRY",
            after="go_to_pantry"
        )

        self.machine.add_transition(
            "pantry_reached",
            "GO_TO_PANTRY",
            "ARM_PLACE",
            after="arm_place"
        )

        self.machine.add_transition(
            "place_done",
            "ARM_PLACE",
            "VACUUM_OFF",
            after="vacuum_off"
        )

        self.machine.add_transition(
            "vacuum_off_done",
            "VACUUM_OFF",
            "ARM_HOME_AFTER_PLACE",
            after="arm_home_after_place"
        )

        self.machine.add_transition(
            "home_after_place_done",
            "ARM_HOME_AFTER_PLACE",
            "DONE",
            after="done"
        )

        self.machine.add_transition(
            "fail",
            "*",
            "ERROR",
            after="error"
        )

        self.get_logger().info("MissionFSM iniciado.")
        self.get_logger().info(f"Arm command topic: {ARM_COMMAND_TOPIC}")
        self.get_logger().info(f"Arm status topic : {ARM_STATUS_TOPIC}")
        self.get_logger().info(f"Vacuum topic     : {VACUUM_COMMAND_TOPIC}")

    # ============================================================
    # CALLBACKS
    # ============================================================

    def arm_status_callback(self, msg):
        self.last_arm_status = msg.data.strip()
        self.arm_status_time = time.time()

    # ============================================================
    # UTILIDADES SECUENCIA
    # ============================================================

    def get_cycles_from_plan(self):
        """
        Convierte las legs del planner en ciclos ejecutables.

        Legs esperadas:
          GO_TO_STORAGE Palm1
          GO_TO_PANTRY Palm1->Desp1
          GO_TO_STORAGE Palm2
          GO_TO_PANTRY Palm2->Desp2
          RETURN_HOME
        """
        if not self.mission_plan:
            return [], None

        legs = self.mission_plan.get("legs", [])

        cycles = []
        current = None
        return_home_leg = None

        for leg in legs:
            label = str(leg.get("label", ""))

            if label.startswith("GO_TO_STORAGE"):
                storage_name = label.replace("GO_TO_STORAGE", "", 1).strip()

                current = {
                    "storage": storage_name,
                    "storage_node": leg["destination"],
                    "go_to_storage_leg": leg,
                    "go_to_pantry_leg": None,
                    "pantry": None,
                    "pantry_node": None
                }

                cycles.append(current)

            elif label.startswith("GO_TO_PANTRY"):
                if current is None:
                    raise RuntimeError("GO_TO_PANTRY sin GO_TO_STORAGE previo")

                relation = label.replace("GO_TO_PANTRY", "", 1).strip()

                pantry_name = ""
                if "->" in relation:
                    pantry_name = relation.split("->", 1)[1].strip()

                current["pantry"] = pantry_name
                current["pantry_node"] = leg["destination"]
                current["go_to_pantry_leg"] = leg

            elif label == "RETURN_HOME":
                return_home_leg = leg

        for cycle in cycles:
            if cycle["go_to_storage_leg"] is None:
                raise RuntimeError(f"Ciclo sin ruta a almacén: {cycle}")

            if cycle["go_to_pantry_leg"] is None:
                raise RuntimeError(f"Ciclo sin ruta a despensa: {cycle}")

        if return_home_leg is None:
            raise RuntimeError("La misión no tiene leg RETURN_HOME")

        return cycles, return_home_leg

    def base_route_leg(self, leg):
        origin = leg["origin"]
        destination = leg["destination"]
        return self.base_route(origin, destination)

    def set_current_cycle(self, cycle):
        self.current_cycle = cycle

        self.storage_name = cycle["storage"]
        self.pantry_name = cycle["pantry"]

        self.storage_node = cycle["storage_node"]
        self.pantry_node = cycle["pantry_node"]

    # ============================================================
    # UTILIDADES
    # ============================================================

    def print_state(self):
        print(f"\n[FSM] Estado actual: {self.state}", flush=True)

    def run_command(self, title, command, timeout=None):
        print("\n" + "=" * 60, flush=True)
        print(f"[FSM] {title}", flush=True)
        print("=" * 60, flush=True)
        print(command, flush=True)

        try:
            result = subprocess.run(
                ["bash", "-lc", command],
                text=True,
                timeout=timeout,
            )

            if result.returncode != 0:
                print(f"[FSM] ERROR en comando: {title}", flush=True)
                self.fail()
                return False

            return True

        except subprocess.TimeoutExpired:
            print(f"[FSM] TIMEOUT en comando: {title}", flush=True)
            self.fail()
            return False

        except Exception as e:
            print(f"[FSM] EXCEPCIÓN en comando {title}: {e}", flush=True)
            self.fail()
            return False

    def publish_string_repeated(self, publisher, text, duration=0.5):
        msg = String()
        msg.data = text

        start = time.time()

        while rclpy.ok() and time.time() - start < duration:
            publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.05)

    # ============================================================
    # BASE
    # ============================================================

    def base_reset_start(self):
        command = (
            f"cd {SCRIPT_DIR} && "
            f"{BASE_SOURCE} "
            f"python3 base_robot.py --reset-start"
        )

        return self.run_command(
            title="BASE reset START + zero IMU",
            command=command,
            timeout=10
        )

    def base_route(self, origin, destination):
        command = (
            f"cd {SCRIPT_DIR} && "
            f"{BASE_SOURCE} "
            f"python3 base_robot.py "
            f"--route {origin} {destination} "
            f"--linear-speed {self.linear_speed} "
            f"--angular-speed {self.angular_speed} "
            f"--yes"
        )

        return self.run_command(
            title=f"BASE ruta {origin} -> {destination}",
            command=command,
        )

    def base_slow_pick_roi(self):
        command = (
            f"cd {SCRIPT_DIR} && "
            f"{BASE_SOURCE} "
            f"python3 base_robot.py "
            f"--slow-pick-roi "
            f"--slow-pick-speed {self.slow_pick_speed}"
        )

        return self.run_command(
            title="BASE slow pick por ROI",
            command=command,
        )

    # ============================================================
    # BRAZO DIRECTO POR ROS
    # ============================================================

    def wait_for_arm_server(self, timeout=4.0):
        start = time.time()

        while rclpy.ok() and time.time() - start < timeout:
            if self.arm_command_pub.get_subscription_count() > 0:
                return True

            rclpy.spin_once(self, timeout_sec=0.05)

        print("[FSM] ERROR: arm_server no está suscrito a /robot/arm_command", flush=True)
        return False

    def arm_command(self, command_name, timeout=12.0):
        command_name = command_name.upper()

        print("\n" + "=" * 60, flush=True)
        print(f"[FSM] BRAZO {command_name}", flush=True)
        print("=" * 60, flush=True)

        if not self.wait_for_arm_server(timeout=4.0):
            self.fail()
            return False

        self.last_arm_status = ""
        self.arm_status_time = 0.0

        self.publish_string_repeated(
            self.arm_command_pub,
            command_name,
            duration=0.6
        )

        start = time.time()
        saw_busy = False

        while rclpy.ok():
            elapsed = time.time() - start
            rclpy.spin_once(self, timeout_sec=0.05)

            status = self.last_arm_status.upper()

            if status.startswith("ERROR"):
                print(f"[FSM] ERROR brazo: {self.last_arm_status}", flush=True)
                self.fail()
                return False

            if status.startswith("BUSY"):
                saw_busy = True

            if saw_busy and status.startswith("IDLE"):
                print(f"[FSM] Brazo terminado: {self.last_arm_status}", flush=True)
                return True

            if elapsed > 2.5 and status.startswith("IDLE"):
                print(f"[FSM] Brazo terminado: {self.last_arm_status}", flush=True)
                return True

            if elapsed >= timeout:
                print(f"[FSM] TIMEOUT esperando brazo {command_name}", flush=True)
                print(f"[FSM] Último estado brazo: {self.last_arm_status}", flush=True)
                self.fail()
                return False

        self.fail()
        return False

    # ============================================================
    # VACUUM DIRECTO POR ROS
    # ============================================================

    def vacuum_command(self, command_name):
        command_name = command_name.upper()

        print("\n" + "=" * 60, flush=True)
        print(f"[FSM] VACUUM {command_name}", flush=True)
        print("=" * 60, flush=True)

        self.publish_string_repeated(
            self.vacuum_command_pub,
            command_name,
            duration=0.6
        )

        time.sleep(0.2)
        return True

    # ============================================================
    # MISIÓN SECUENCIAL
    # ============================================================

    def run_sequence_mission(self):
        """
        Ejecuta misión completa de varios almacenes/despensas.

        Por ciclo:
          ruta a almacén
          slow pick ROI
          vacuum ON
          PICK
          HOME brazo
          ruta a despensa
          PLACE
          vacuum OFF
          HOME brazo

        Al final:
          ruta última despensa -> START
        """
        print("\n========== FSM MISIÓN SECUENCIAL ==========", flush=True)

        cycles, return_home_leg = self.get_cycles_from_plan()

        print(f"Ciclos: {len(cycles)}", flush=True)

        for i, cycle in enumerate(cycles, start=1):
            print(
                f"  {i}. {cycle['storage_node']} → {cycle['pantry_node']}",
                flush=True
            )

        print(
            "  HOME: " + " → ".join(return_home_leg.get("node_names", [])),
            flush=True
        )

        print("==========================================", flush=True)

        if self.origin_name == "START":
            print("\n[FSM] Origen START: calibrando pose e IMU antes de mover.", flush=True)
            if not self.base_reset_start():
                return False
        else:
            print("\n[FSM] Origen no START: no recalibro IMU.", flush=True)

        if not self.arm_command("HOME"):
            return False

        for index, cycle in enumerate(cycles, start=1):
            self.current_cycle_index = index
            self.set_current_cycle(cycle)

            print("\n" + "#" * 60, flush=True)
            print(
                f"[FSM] CICLO {index}/{len(cycles)} | "
                f"{self.storage_node} → {self.pantry_node}",
                flush=True
            )
            print("#" * 60, flush=True)

            print("\n[FSM] Ir a almacén", flush=True)
            if not self.base_route_leg(cycle["go_to_storage_leg"]):
                return False

            print("\n[FSM] Slow pick ROI", flush=True)
            if not self.base_slow_pick_roi():
                return False

            print("\n[FSM] Vacuum ON", flush=True)
            if not self.vacuum_command("ON"):
                return False

            print("\n[FSM] Brazo PICK", flush=True)
            if not self.arm_command("PICK"):
                return False

            print("\n[FSM] Brazo HOME después de PICK", flush=True)
            if not self.arm_command("HOME"):
                return False

            print("\n[FSM] Ir a despensa", flush=True)
            if not self.base_route_leg(cycle["go_to_pantry_leg"]):
                return False

            print("\n[FSM] Brazo PLACE", flush=True)
            if not self.arm_command("PLACE"):
                return False

            print("\n[FSM] Vacuum OFF", flush=True)
            if not self.vacuum_command("OFF"):
                return False

            print("\n[FSM] Brazo HOME después de PLACE", flush=True)
            if not self.arm_command("HOME"):
                return False

        print("\n[FSM] Volver a START / HOME", flush=True)
        if not self.base_route_leg(return_home_leg):
            return False

        print("\n" + "=" * 60, flush=True)
        print("[FSM] MISIÓN SECUENCIAL TERMINADA", flush=True)
        print("=" * 60, flush=True)

        return True

    # ============================================================
    # ESTADOS FSM ANTIGUA DE 1 PICK & PLACE
    # ============================================================

    def reset_start_if_needed(self):
        if self.origin_name == "START":
            print("\n[FSM] Origen START: calibrando pose e IMU antes de mover.", flush=True)
            ok = self.base_reset_start()

            if ok:
                self.reset_start_done()
            return

        print("\n[FSM] Origen no es START: no recalibro IMU.", flush=True)
        self.reset_start_done()

    def arm_home_init(self):
        ok = self.arm_command("HOME")

        if ok:
            self.home_init_done()

    def go_to_storage(self):
        ok = self.base_route(self.origin_name, self.storage_node)

        if ok:
            self.storage_reached()

    def slow_pick_roi(self):
        ok = self.base_slow_pick_roi()

        if ok:
            self.slow_pick_done()

    def vacuum_on(self):
        ok = self.vacuum_command("ON")

        if ok:
            self.vacuum_on_done()

    def arm_pick(self):
        ok = self.arm_command("PICK")

        if ok:
            self.pick_done()

    def arm_home_after_pick(self):
        ok = self.arm_command("HOME")

        if ok:
            self.home_after_pick_done()

    def go_to_pantry(self):
        ok = self.base_route(self.storage_node, self.pantry_node)

        if ok:
            self.pantry_reached()

    def arm_place(self):
        ok = self.arm_command("PLACE")

        if ok:
            self.place_done()

    def vacuum_off(self):
        ok = self.vacuum_command("OFF")

        if ok:
            self.vacuum_off_done()

    def arm_home_after_place(self):
        ok = self.arm_command("HOME")

        if ok:
            self.home_after_place_done()

    def done(self):
        print("\n" + "=" * 60, flush=True)
        print("[FSM] MISIÓN PICK & PLACE TERMINADA", flush=True)
        print("=" * 60, flush=True)
        print(f"Almacén : {self.storage_node}", flush=True)
        print(f"Despensa: {self.pantry_node}", flush=True)

    def error(self):
        print("\n" + "=" * 60, flush=True)
        print("[FSM] MISIÓN DETENIDA POR ERROR", flush=True)
        print("=" * 60, flush=True)


def main(args=None):
    parser = argparse.ArgumentParser()

    parser.add_argument("--storage", help="Nombre almacén sin AA_, ejemplo: Palm1")
    parser.add_argument("--pantry", help="Nombre despensa sin AD_, ejemplo: Desp1")
    parser.add_argument("--origin", default="START", help="Nodo origen, ejemplo START o AD_Desp1")
    parser.add_argument("--mission-json", help="Archivo JSON con misión secuencial")

    parser.add_argument("--linear-speed", type=float, default=DEFAULT_LINEAR_SPEED)
    parser.add_argument("--angular-speed", type=float, default=DEFAULT_ANGULAR_SPEED)
    parser.add_argument("--slow-pick-speed", type=float, default=DEFAULT_SLOW_PICK_SPEED)

    parsed_args = parser.parse_args()

    mission_plan = None

    if parsed_args.mission_json:
        with open(parsed_args.mission_json, "r", encoding="utf-8") as f:
            mission_payload = json.load(f)

        mission_plan = mission_payload["plan"]

        parsed_args.linear_speed = float(
            mission_payload.get("linear_speed", parsed_args.linear_speed)
        )
        parsed_args.angular_speed = float(
            mission_payload.get("angular_speed", parsed_args.angular_speed)
        )
        parsed_args.slow_pick_speed = float(
            mission_payload.get("slow_pick_speed", parsed_args.slow_pick_speed)
        )

    else:
        if not parsed_args.storage:
            parser.error("--storage es obligatorio si no usas --mission-json")

        if not parsed_args.pantry:
            parser.error("--pantry es obligatorio si no usas --mission-json")

    rclpy.init(args=args)

    node = MissionFSM(
        storage_name=parsed_args.storage,
        pantry_name=parsed_args.pantry,
        origin_name=parsed_args.origin,
        linear_speed=parsed_args.linear_speed,
        angular_speed=parsed_args.angular_speed,
        slow_pick_speed=parsed_args.slow_pick_speed,
        mission_plan=mission_plan,
    )

    try:
        if node.sequence_mode:
            ok = node.run_sequence_mission()

            if not ok:
                sys.exit(1)

        else:
            print("\n========== FSM PICK & PLACE ==========", flush=True)
            print(f"Origen   : {node.origin_name}", flush=True)
            print(f"Almacén  : {node.storage_node}", flush=True)
            print(f"Despensa : {node.pantry_node}", flush=True)
            print("=====================================", flush=True)

            node.start()

            if node.state == "ERROR":
                sys.exit(1)

    except KeyboardInterrupt:
        print("\n[FSM] Interrumpido por usuario.", flush=True)

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()