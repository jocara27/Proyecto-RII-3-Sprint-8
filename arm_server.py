#!/usr/bin/env python3

import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState


SCRIPT_DIR = Path(__file__).resolve().parent
CONFIG_FILE = SCRIPT_DIR / "robot_config.json"

TRAJECTORY_TOPIC = "/arm/arm_controller/joint_trajectory"
JOINT_STATES_TOPIC = "/arm/joint_states"

ARM_COMMAND_TOPIC = "/robot/arm_command"
ARM_STATUS_TOPIC = "/robot/arm_status"

DEFAULT_MOVE_TIME = 2.0


class ArmServer(Node):
    def __init__(self):
        super().__init__("arm_server_node")

        self.config = self.load_config()
        self.joint_names = self.config["joint_names"]

        self.current_position_deg = [0.0 for _ in self.joint_names]
        self.have_real_position = False
        self.busy = False
        self.last_status = "IDLE"

        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            TRAJECTORY_TOPIC,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            ARM_STATUS_TOPIC,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            JOINT_STATES_TOPIC,
            self.joint_state_callback,
            10
        )

        self.command_sub = self.create_subscription(
            String,
            ARM_COMMAND_TOPIC,
            self.command_callback,
            10
        )

        self.status_timer = self.create_timer(0.5, self.publish_status)

        self.publish_status_text("IDLE: arm_server iniciado")
        self.get_logger().info("arm_server iniciado.")
        self.get_logger().info(f"Escuchando comandos en: {ARM_COMMAND_TOPIC}")
        self.get_logger().info(f"Publicando estado en: {ARM_STATUS_TOPIC}")

    # ============================================================
    # CONFIG
    # ============================================================

    def load_config(self):
        if not CONFIG_FILE.exists():
            raise FileNotFoundError(f"No existe {CONFIG_FILE}")

        with open(CONFIG_FILE, "r", encoding="utf-8") as f:
            config = json.load(f)

        required = ["joint_names", "puntos"]

        for key in required:
            if key not in config:
                raise KeyError(f"Falta '{key}' en {CONFIG_FILE}")

        for point in ["HOME", "PICKDROP"]:
            if point not in config["puntos"]:
                raise KeyError(f"Falta punto '{point}' en robot_config.json")

        return config

    # ============================================================
    # STATUS
    # ============================================================

    def publish_status_text(self, text):
        self.last_status = text

        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

        self.get_logger().info(text)

    def publish_status(self):
        msg = String()
        msg.data = self.last_status
        self.status_pub.publish(msg)

    def format_position(self, position):
        return "[" + ", ".join(f"{x:.2f}" for x in position) + "]"

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

    # ============================================================
    # MOVEMENT
    # ============================================================

    def deg2rad(self, degrees_list):
        return [math.radians(float(d)) for d in degrees_list]

    def seconds_to_duration(self, seconds):
        seconds = float(seconds)
        sec = int(seconds)
        nanosec = int((seconds - sec) * 1_000_000_000)
        return Duration(sec=sec, nanosec=nanosec)

    def send_position(self, position_deg, move_time=DEFAULT_MOVE_TIME):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.deg2rad(position_deg)
        point.time_from_start = self.seconds_to_duration(move_time)

        msg.points.append(point)

        for _ in range(3):
            self.trajectory_pub.publish(msg)
            time.sleep(0.05)

        start = time.time()

        while time.time() - start < move_time and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def move_to_point(self, point_name, move_time=DEFAULT_MOVE_TIME):
        puntos = self.config["puntos"]

        if point_name not in puntos:
            raise KeyError(f"No existe punto {point_name}")

        position = puntos[point_name]

        self.publish_status_text(
            f"BUSY: moviendo a {point_name} {self.format_position(position)}"
        )

        self.send_position(position, move_time)

        if self.have_real_position:
            self.publish_status_text(
                f"IDLE: terminado {point_name}. Posición real {self.format_position(self.current_position_deg)}"
            )
        else:
            self.publish_status_text(f"IDLE: terminado {point_name}")

    # ============================================================
    # ACTIONS
    # ============================================================

    def action_home(self):
        self.move_to_point("HOME", DEFAULT_MOVE_TIME)

    def action_pick(self):
        self.move_to_point("PICKDROP", DEFAULT_MOVE_TIME)
        self.publish_status_text("IDLE: PICK terminado en PICKDROP")

    def action_place(self):
        self.move_to_point("PICKDROP", DEFAULT_MOVE_TIME)
        self.publish_status_text("IDLE: PLACE terminado en PICKDROP")

    def action_status(self):
        if self.have_real_position:
            self.publish_status_text(
                f"IDLE: posición actual {self.format_position(self.current_position_deg)}"
            )
        else:
            self.publish_status_text("IDLE: sin /arm/joint_states todavía")

    # ============================================================
    # COMMAND CALLBACK
    # ============================================================

    def command_callback(self, msg):
        command = msg.data.strip().upper()

        if self.busy:
            self.publish_status_text(f"BUSY: ignorado comando {command}")
            return

        self.busy = True

        try:
            self.publish_status_text(f"BUSY: comando recibido {command}")

            if command == "HOME":
                self.action_home()

            elif command == "PICK":
                self.action_pick()

            elif command == "PLACE":
                self.action_place()

            elif command == "STATUS":
                self.action_status()

            else:
                self.publish_status_text(f"ERROR: comando no válido {command}")

        except Exception as e:
            self.publish_status_text(f"ERROR: {e}")

        finally:
            self.busy = False


def main(args=None):
    rclpy.init(args=args)

    node = ArmServer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\narm_server interrumpido.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()