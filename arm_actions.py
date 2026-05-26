#!/usr/bin/env python3

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


# ============================================================
# CONFIGURACIÓN
# ============================================================

SCRIPT_DIR = Path(__file__).resolve().parent
CONFIG_FILE = SCRIPT_DIR / "robot_config.json"

TRAJECTORY_TOPIC = "/arm/arm_controller/joint_trajectory"
JOINT_STATES_TOPIC = "/arm/joint_states"

VACUUM_TOPIC = "/vacuum/set"

DEFAULT_MOVE_TIME = 2.0


# ============================================================
# CLASE ARM ACTIONS
# ============================================================

class ArmActions(Node):
    def __init__(self):
        super().__init__("arm_actions_node")

        self.publisher = self.create_publisher(
            JointTrajectory,
            TRAJECTORY_TOPIC,
            10
        )

        self.vacuum_pub = self.create_publisher(
            Bool,
            VACUUM_TOPIC,
            10
        )

        self.config = self.load_config()
        self.joint_names = self.config["joint_names"]

        self.current_position_deg = [0.0 for _ in self.joint_names]
        self.have_real_position = False

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            JOINT_STATES_TOPIC,
            self.joint_state_callback,
            10
        )

        time.sleep(0.5)
        self.update_joint_state(seconds=1.0)

        self.get_logger().info("ArmActions iniciado.")
        self.get_logger().info(f"Trajectory topic: {TRAJECTORY_TOPIC}")
        self.get_logger().info(f"Joint states topic: {JOINT_STATES_TOPIC}")

    # ============================================================
    # CONFIG
    # ============================================================

    def load_config(self):
        if not CONFIG_FILE.exists():
            raise FileNotFoundError(
                f"No encuentro {CONFIG_FILE}. "
                "Crea robot_config.json en la carpeta scripts."
            )

        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            config = json.load(f)

        required_keys = ["joint_names", "puntos"]

        for key in required_keys:
            if key not in config:
                raise KeyError(f"Falta la clave '{key}' en {CONFIG_FILE}")

        if "HOME" not in config["puntos"]:
            raise KeyError("Falta el punto HOME en robot_config.json")

        if "PICKDROP" not in config["puntos"]:
            raise KeyError("Falta el punto PICKDROP en robot_config.json")

        return config

    # ============================================================
    # JOINT STATES
    # ============================================================

    def joint_state_callback(self, msg):
        new_position = self.current_position_deg.copy()

        for i, joint_name in enumerate(self.joint_names):
            if joint_name in msg.name:
                index = msg.name.index(joint_name)

                if index < len(msg.position):
                    new_position[i] = math.degrees(msg.position[index])

        self.current_position_deg = new_position
        self.have_real_position = True

    def update_joint_state(self, seconds=0.5):
        start = time.time()

        while time.time() - start < seconds and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def format_position(self, position):
        return "[" + ", ".join(f"{x:.2f}" for x in position) + "]"

    def print_current_position(self):
        self.update_joint_state(seconds=0.3)

        print("\n========== POSICIÓN BRAZO ==========")

        if self.have_real_position:
            print("Fuente: /arm/joint_states")
        else:
            print("AVISO: no he recibido /arm/joint_states todavía.")

        for name, value in zip(self.joint_names, self.current_position_deg):
            print(f"{name}: {value:.2f} grados")

    # ============================================================
    # MOVIMIENTO
    # ============================================================

    def deg2rad(self, degrees_list):
        return [math.radians(float(d)) for d in degrees_list]

    def seconds_to_duration(self, seconds):
        seconds = float(seconds)
        sec = int(seconds)
        nanosec = int((seconds - sec) * 1_000_000_000)

        return Duration(sec=sec, nanosec=nanosec)

    def send_position(self, position_deg, move_time=DEFAULT_MOVE_TIME):
        move_time = float(move_time)

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.deg2rad(position_deg)
        point.time_from_start = self.seconds_to_duration(move_time)

        msg.points.append(point)

        # Publicar varias veces al inicio para asegurar recepción.
        for _ in range(3):
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info(
            f"Moviendo brazo a {self.format_position(position_deg)} "
            f"en {move_time:.2f} s"
        )

        start = time.time()

        while time.time() - start < move_time and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

        self.update_joint_state(seconds=0.3)

        self.get_logger().info("Movimiento de brazo terminado.")
        self.print_current_position()

    def move_to_point(self, point_name, move_time=DEFAULT_MOVE_TIME):
        puntos = self.config["puntos"]

        if point_name not in puntos:
            raise KeyError(f"El punto {point_name} no existe.")

        position_deg = puntos[point_name]

        self.get_logger().info(f"Ejecutando punto {point_name}")
        self.send_position(position_deg, move_time)

    # ============================================================
    # VACUUM
    # ============================================================

    def vacuum_terminal(self, state):
        if state:
            input("\n[VACUUM] Activa la vacuum ON y pulsa ENTER...")
        else:
            input("\n[VACUUM] Apaga la vacuum OFF y pulsa ENTER...")

    def vacuum_topic(self, state):
        msg = Bool()
        msg.data = bool(state)

        for _ in range(5):
            self.vacuum_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        if state:
            self.get_logger().info(f"Vacuum ON publicado en {VACUUM_TOPIC}")
        else:
            self.get_logger().info(f"Vacuum OFF publicado en {VACUUM_TOPIC}")

    def vacuum_none(self, state):
        if state:
            self.get_logger().info("Vacuum ON omitido.")
        else:
            self.get_logger().info("Vacuum OFF omitido.")

    def set_vacuum(self, state, mode):
        mode = mode.lower()

        if mode == "terminal":
            self.vacuum_terminal(state)

        elif mode == "topic":
            self.vacuum_topic(state)

        elif mode == "none":
            self.vacuum_none(state)

        else:
            raise ValueError("vacuum-mode debe ser: terminal, topic o none")

    # ============================================================
    # ACCIONES
    # ============================================================

    def action_home(self, move_time):
        print("\n========== ARM HOME ==========")
        self.move_to_point("HOME", move_time)

    def action_pick(self, move_time, vacuum_mode):
        print("\n========== ARM PICK ==========")
        print("1. Brazo baja a PICKDROP")
        self.move_to_point("PICKDROP", move_time)

        print("2. Vacuum ON")
        self.set_vacuum(True, vacuum_mode)

        print("PICK terminado. El brazo queda en PICKDROP.")
        print("Después mission_manager llamará a HOME para subir con la pieza.")

    def action_place(self, move_time, vacuum_mode):
        print("\n========== ARM PLACE ==========")
        print("1. Brazo baja a PICKDROP")
        self.move_to_point("PICKDROP", move_time)

        print("2. Vacuum OFF")
        self.set_vacuum(False, vacuum_mode)

        print("PLACE terminado. El brazo queda en PICKDROP.")
        print("Después mission_manager llamará a HOME.")

    def close(self):
        pass


# ============================================================
# MAIN
# ============================================================

def main(args=None):
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "action",
        choices=["home", "pick", "place", "position"],
        help="Acción del brazo"
    )

    parser.add_argument(
        "--time",
        type=float,
        default=DEFAULT_MOVE_TIME,
        help="Tiempo de movimiento en segundos"
    )

    parser.add_argument(
        "--vacuum-mode",
        choices=["terminal", "topic", "none"],
        default="terminal",
        help="Cómo controlar la vacuum"
    )

    parsed_args = parser.parse_args()

    rclpy.init(args=args)

    node = ArmActions()

    try:
        if parsed_args.action == "home":
            node.action_home(parsed_args.time)

        elif parsed_args.action == "pick":
            node.action_pick(parsed_args.time, parsed_args.vacuum_mode)

        elif parsed_args.action == "place":
            node.action_place(parsed_args.time, parsed_args.vacuum_mode)

        elif parsed_args.action == "position":
            node.print_current_position()

    except KeyboardInterrupt:
        print("\nInterrumpido por usuario.")

    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()