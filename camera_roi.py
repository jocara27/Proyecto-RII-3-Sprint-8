#!/usr/bin/env python3

import json
import time
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


SCRIPT_DIR = Path(__file__).resolve().parent
ROI_FILE = SCRIPT_DIR / "roi_config.json"

IMAGE_TOPIC = "/camera/color/image_raw"
PIECE_IN_ROI_TOPIC = "/arm_camera/piece_in_roi"
ROI_STATUS_TOPIC = "/arm_camera/roi_status"

ARUCO_DICTIONARY = cv2.aruco.DICT_4X4_50


class CameraROI(Node):
    def __init__(self):
        super().__init__("camera_roi_node")

        self.bridge = CvBridge()

        self.piece_in_roi_pub = self.create_publisher(
            Bool,
            PIECE_IN_ROI_TOPIC,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            ROI_STATUS_TOPIC,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback,
            10
        )

        self.last_roi_load_time = 0.0
        self.roi_config = self.load_roi_config()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARY)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.detector = cv2.aruco.ArucoDetector(
            self.aruco_dict,
            self.aruco_params
        )

        self.last_publish_time = 0.0
        self.last_piece_in_roi = False

        self.get_logger().info("camera_roi iniciado.")
        self.get_logger().info(f"Imagen: {IMAGE_TOPIC}")
        self.get_logger().info(f"Publica Bool: {PIECE_IN_ROI_TOPIC}")
        self.get_logger().info(f"ROI config: {ROI_FILE}")

    # ============================================================
    # ROI CONFIG
    # ============================================================

    def load_roi_config(self):
        default_config = {
            "image_width": 640,
            "image_height": 480,
            "roi": {
                "x": 250,
                "y": 170,
                "w": 140,
                "h": 120
            }
        }

        if not ROI_FILE.exists():
            return default_config

        try:
            with open(ROI_FILE, "r", encoding="utf-8") as f:
                config = json.load(f)

            if "roi" not in config:
                return default_config

            return config

        except Exception as e:
            self.get_logger().error(f"No puedo leer roi_config.json: {e}")
            return default_config

    def reload_roi_periodically(self):
        now = time.time()

        if now - self.last_roi_load_time > 0.5:
            self.roi_config = self.load_roi_config()
            self.last_roi_load_time = now

    def get_roi(self, image):
        self.reload_roi_periodically()

        height, width = image.shape[:2]

        roi = self.roi_config.get("roi", {})

        x = int(roi.get("x", 0))
        y = int(roi.get("y", 0))
        w = int(roi.get("w", 100))
        h = int(roi.get("h", 100))

        x = max(0, min(x, width - 1))
        y = max(0, min(y, height - 1))
        w = max(1, min(w, width - x))
        h = max(1, min(h, height - y))

        return x, y, w, h

    # ============================================================
    # DETECCIÓN
    # ============================================================

    def aruco_center_inside_roi(self, corners, roi):
        x, y, w, h = roi

        for marker_corners in corners:
            pts = marker_corners[0]

            cx = float(pts[:, 0].mean())
            cy = float(pts[:, 1].mean())

            inside = (
                x <= cx <= x + w and
                y <= cy <= y + h
            )

            if inside:
                return True, cx, cy

        return False, None, None

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        except Exception as e:
            self.publish_status(False, f"ERROR cv_bridge: {e}")
            return

        roi = self.get_roi(image)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)

        marker_count = 0 if ids is None else len(ids)

        piece_in_roi = False
        cx = None
        cy = None

        if marker_count > 0:
            piece_in_roi, cx, cy = self.aruco_center_inside_roi(corners, roi)

        if piece_in_roi != self.last_piece_in_roi:
            self.last_piece_in_roi = piece_in_roi

            if piece_in_roi:
                self.get_logger().info("PIEZA/ARUCO DENTRO DEL ROI")
            else:
                self.get_logger().info("PIEZA/ARUCO FUERA DEL ROI")

        status = (
            f"markers={marker_count} "
            f"piece_in_roi={piece_in_roi} "
            f"roi={roi}"
        )

        if cx is not None:
            status += f" center=({cx:.1f},{cy:.1f})"

        self.publish_status(piece_in_roi, status)

    # ============================================================
    # PUBLICACIÓN
    # ============================================================

    def publish_status(self, piece_in_roi, text):
        now = time.time()

        # Publicamos Bool a alta frecuencia razonable.
        bool_msg = Bool()
        bool_msg.data = bool(piece_in_roi)
        self.piece_in_roi_pub.publish(bool_msg)

        # Status menos ruidoso.
        if now - self.last_publish_time > 0.5:
            status_msg = String()
            status_msg.data = text
            self.status_pub.publish(status_msg)
            self.last_publish_time = now


def main(args=None):
    rclpy.init(args=args)

    node = CameraROI()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\ncamera_roi interrumpido.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()