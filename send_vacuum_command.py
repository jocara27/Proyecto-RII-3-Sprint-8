#!/usr/bin/env python3

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


VACUUM_COMMAND_TOPIC = "/robot/vacuum_command"


class VacuumCommandSender(Node):
    def __init__(self):
        super().__init__("vacuum_command_sender_node")
        self.publisher = self.create_publisher(String, VACUUM_COMMAND_TOPIC, 10)

    def wait_for_subscriber(self, timeout=3.0):
        start = time.time()

        while rclpy.ok() and time.time() - start < timeout:
            if self.publisher.get_subscription_count() > 0:
                return True

            rclpy.spin_once(self, timeout_sec=0.05)

        return False

    def send(self, command):
        msg = String()
        msg.data = command

        print(f"Esperando subscriber en {VACUUM_COMMAND_TOPIC}...")

        if not self.wait_for_subscriber(timeout=3.0):
            print("ERROR: no hay subscriber. ¿Está abierto vacuum_server.py?")
            return False

        print(f"Enviando comando vacuum: {command}")

        start = time.time()

        while rclpy.ok() and time.time() - start < 0.8:
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        print("Comando vacuum enviado.")
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=["ON", "OFF", "STATUS"])
    args = parser.parse_args()

    rclpy.init()

    node = VacuumCommandSender()

    try:
        ok = node.send(args.command)

        if not ok:
            raise SystemExit(1)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()