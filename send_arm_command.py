#!/usr/bin/env python3

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


ARM_COMMAND_TOPIC = "/robot/arm_command"


class ArmCommandSender(Node):
    def __init__(self):
        super().__init__("arm_command_sender_node")
        self.publisher = self.create_publisher(String, ARM_COMMAND_TOPIC, 10)

    def wait_for_subscriber(self, timeout=3.0):
        start = time.time()

        while rclpy.ok() and time.time() - start < timeout:
            count = self.publisher.get_subscription_count()

            if count > 0:
                return True

            rclpy.spin_once(self, timeout_sec=0.05)

        return False

    def send(self, command):
        msg = String()
        msg.data = command

        print(f"Esperando subscriber en {ARM_COMMAND_TOPIC}...")

        if not self.wait_for_subscriber(timeout=3.0):
            print("ERROR: no hay subscriber. ¿Está abierto arm_server.py?")
            return False

        print(f"Enviando comando: {command}")

        # Publicamos durante 1 segundo para asegurar recepción DDS.
        start = time.time()

        while rclpy.ok() and time.time() - start < 1.0:
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        print("Comando enviado.")
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=["HOME", "PICK", "PLACE", "STATUS"])
    args = parser.parse_args()

    rclpy.init()

    node = ArmCommandSender()

    try:
        ok = node.send(args.command)

        if not ok:
            raise SystemExit(1)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()