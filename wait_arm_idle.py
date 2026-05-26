#!/usr/bin/env python3

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


ARM_STATUS_TOPIC = "/robot/arm_status"


class ArmIdleWaiter(Node):
    def __init__(self):
        super().__init__("arm_idle_waiter_node")

        self.last_status = None
        self.is_idle = False

        self.sub = self.create_subscription(
            String,
            ARM_STATUS_TOPIC,
            self.status_callback,
            10
        )

    def status_callback(self, msg):
        self.last_status = msg.data.strip()

        if self.last_status.upper().startswith("IDLE"):
            self.is_idle = True

        elif self.last_status.upper().startswith("ERROR"):
            raise RuntimeError(self.last_status)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()

    rclpy.init()

    node = ArmIdleWaiter()

    print(f"Esperando IDLE en {ARM_STATUS_TOPIC}...")

    start = time.time()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            if node.is_idle:
                print(f"ARM IDLE recibido: {node.last_status}")
                return

            if time.time() - start > args.timeout:
                print("ERROR: timeout esperando IDLE del brazo.")
                print(f"Último estado: {node.last_status}")
                raise SystemExit(1)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()