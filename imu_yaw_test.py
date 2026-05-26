#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def quat_to_yaw_deg(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw)


class ImuYawTest(Node):
    def __init__(self):
        super().__init__("imu_yaw_test")
        self.sub = self.create_subscription(
            Imu,
            "/imu_broadcaster/imu",
            self.cb,
            10
        )

    def cb(self, msg):
        yaw = quat_to_yaw_deg(msg.orientation)
        print(f"Yaw IMU: {yaw:8.2f} deg")


def main():
    rclpy.init()
    node = ImuYawTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()