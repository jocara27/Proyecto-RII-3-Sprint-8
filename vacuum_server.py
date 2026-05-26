#!/usr/bin/env python3

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


VACUUM_COMMAND_TOPIC = "/robot/vacuum_command"
VACUUM_STATUS_TOPIC = "/robot/vacuum_status"


class VacuumServer(Node):
    def __init__(self, mode, port, baudrate):
        super().__init__("vacuum_server_node")

        self.mode = mode
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.vacuum_on = False
        self.last_status = "IDLE: vacuum_server iniciado"

        self.command_sub = self.create_subscription(
            String,
            VACUUM_COMMAND_TOPIC,
            self.command_callback,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            VACUUM_STATUS_TOPIC,
            10
        )

        self.timer = self.create_timer(0.5, self.publish_status)

        if self.mode == "serial":
            self.connect_serial()
        else:
            self.publish_status_text("IDLE: vacuum_server en modo FAKE")

        self.get_logger().info(f"Escuchando comandos en {VACUUM_COMMAND_TOPIC}")
        self.get_logger().info(f"Publicando estado en {VACUUM_STATUS_TOPIC}")

    def connect_serial(self):
        try:
            import serial

            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )

            time.sleep(2.0)

            self.publish_status_text(
                f"IDLE: vacuum conectado en {self.port} a {self.baudrate}"
            )

            self.send_serial("v")
            self.vacuum_on = False

        except Exception as e:
            self.serial = None
            self.publish_status_text(f"ERROR: no se pudo abrir vacuum serial: {e}")

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

    def send_serial(self, command):
        if self.mode != "serial":
            return True

        if self.serial is None or not self.serial.is_open:
            self.publish_status_text("ERROR: serial vacuum no conectado")
            return False

        try:
            self.serial.write(command.encode("utf-8"))
            self.serial.flush()
            time.sleep(0.1)

            while self.serial.in_waiting > 0:
                line = self.serial.readline().decode(errors="ignore").strip()
                if line:
                    self.get_logger().info(f"Arduino vacuum: {line}")

            return True

        except Exception as e:
            self.publish_status_text(f"ERROR serial vacuum: {e}")
            return False

    def vacuum_on_action(self):
        ok = self.send_serial("V")

        if ok:
            self.vacuum_on = True
            self.publish_status_text("IDLE: VACUUM ON")

    def vacuum_off_action(self):
        ok = self.send_serial("v")

        if ok:
            self.vacuum_on = False
            self.publish_status_text("IDLE: VACUUM OFF")

    def command_callback(self, msg):
        command = msg.data.strip().upper()

        self.publish_status_text(f"BUSY: comando recibido {command}")

        if command == "ON":
            self.vacuum_on_action()

        elif command == "OFF":
            self.vacuum_off_action()

        elif command == "STATUS":
            state = "ON" if self.vacuum_on else "OFF"
            self.publish_status_text(f"IDLE: VACUUM {state}")

        else:
            self.publish_status_text(f"ERROR: comando vacuum no válido {command}")

    def close(self):
        try:
            self.vacuum_off_action()
        except Exception:
            pass

        if self.serial is not None and self.serial.is_open:
            self.serial.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["fake", "serial"], default="fake")
    parser.add_argument("--port", default="/dev/ttyACM1")
    parser.add_argument("--baudrate", type=int, default=115200)

    args = parser.parse_args()

    rclpy.init()

    node = VacuumServer(
        mode=args.mode,
        port=args.port,
        baudrate=args.baudrate
    )

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\nvacuum_server interrumpido.")

    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()