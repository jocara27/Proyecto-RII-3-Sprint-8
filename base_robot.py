#!/usr/bin/env python3

import argparse
import json
import math
import select
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from path_planner import PathPlanner


SCRIPT_DIR = Path(__file__).resolve().parent
MAP_FILE = SCRIPT_DIR / "map_config.json"
STATE_FILE = SCRIPT_DIR / "robot_runtime_state.json"
IMU_CONFIG_FILE = SCRIPT_DIR / "imu_config.json"

CMD_VEL_TOPIC = "/diff_drive_controller/cmd_vel_unstamped"
ROI_TOPIC = "/arm_camera/piece_in_roi"
IMU_TOPIC = "/imu_broadcaster/imu"
ODOM_TOPIC = "/diff_drive_controller/odom"

CONTROL_DT = 0.05
PRINT_PERIOD = 0.25
STATE_UPDATE_PERIOD = 0.20

# Perfil de velocidad lineal.
# --linear-speed será velocidad máxima, no velocidad constante.
CONTROL_DT = 0.05
PRINT_PERIOD = 0.25
STATE_UPDATE_PERIOD = 0.20

# ============================================================
# PERFIL LINEAL PLANIFICADO
# ============================================================
# --linear-speed es velocidad máxima permitida, no velocidad obligatoria.
LINEAR_ACCEL = 0.22              # m/s²
LINEAR_DECEL = 0.28              # m/s²
MIN_LINEAR_SPEED = 0.018         # m/s
MAX_LINEAR_CMD_ACCEL = 0.18     # m/s² subida real máxima de comando
MAX_LINEAR_CMD_DECEL = 0.22     # m/s² bajada real máxima de comando

# Control de seguimiento de perfil s(t)
LINEAR_PROFILE_KP = 0.85         # corrección por error de posición
MAX_PROFILE_CORRECTION = 0.025   # m/s máximo extra/resta por corrección

POSITION_TOLERANCE = 0.006       # m
FINAL_CRAWL_DISTANCE = 0.025     # m
FINAL_CRAWL_SPEED = 0.012        # m/s

FINAL_APPROACH_SPEED = 0.012          # m/s
FINAL_APPROACH_MAX_DISTANCE = 0.003    # m, máximo que corrige fino
FINAL_APPROACH_TOLERANCE = 0.004      # m, 4 mm
FINAL_APPROACH_TIMEOUT = 5.0          # s

# ============================================================
# PERFIL ANGULAR PLANIFICADO
# ============================================================
# --angular-speed es velocidad angular máxima permitida, no obligatoria.
ANGULAR_ACCEL = 1.20              # rad/s²
ANGULAR_DECEL = 1.50              # rad/s²
MIN_ANGULAR_SPEED = 0.055         # rad/s

ANGULAR_PROFILE_KP = 1.60         # corrección por error angular
MAX_ANGULAR_CORRECTION = 0.18     # rad/s máximo extra/resta por corrección

YAW_TOLERANCE_PROFILE_DEG = 2.5
FINAL_CRAWL_ANGLE_DEG = 5.0
FINAL_CRAWL_ANGULAR_SPEED = 0.040


def normalize_angle_deg(angle):
    angle = float(angle)

    while angle > 180.0:
        angle -= 360.0

    while angle < -180.0:
        angle += 360.0

    return angle


def deg2rad(value):
    return math.radians(float(value))


def rad2deg(value):
    return math.degrees(float(value))


def quat_to_yaw_deg(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw)


DEFAULT_LINEAR_SPEED = 0.06
DEFAULT_ANGULAR_SPEED = 0.30
DEFAULT_SLOW_PICK_SPEED = 0.015


class BaseRobot(Node):
    def __init__(self):
        super().__init__("base_robot_node")

        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self.base_reset_pub = self.create_publisher(
            String,
            "/robot/base_reset",
            10
        )

        self.piece_in_roi = False
        self.roi_sub = self.create_subscription(
            Bool,
            ROI_TOPIC,
            self.roi_callback,
            10
        )

        self.imu_raw_yaw_deg = None
        self.imu_map_yaw_deg = None
        self.imu_sub = self.create_subscription(
            Imu,
            IMU_TOPIC,
            self.imu_callback,
            10
        )

        self.odom_x = None
        self.odom_y = None
        self.odom_sub = self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self.odom_callback,
            10
        )

        self.map_config = self.load_map()
        self.imu_config = self.load_imu_config()
        self.state = self.load_or_create_state()

        self.x = float(self.state["pose"]["x"])
        self.y = float(self.state["pose"]["y"])
        self.theta_deg = float(self.state["pose"]["theta_deg"])

        self.linear_speed = float(self.state.get("linear_speed", DEFAULT_LINEAR_SPEED))
        self.angular_speed = float(self.state.get("angular_speed", DEFAULT_ANGULAR_SPEED))
        self.slow_pick_speed = float(self.state.get("slow_pick_speed", DEFAULT_SLOW_PICK_SPEED))

        self.auto_confirm = False

        self.get_logger().info("BaseRobot iniciado.")
        self.get_logger().info(f"cmd_vel: {CMD_VEL_TOPIC}")
        self.get_logger().info(f"ROI topic: {ROI_TOPIC}")
        self.get_logger().info(f"IMU topic: {IMU_TOPIC}")
        self.get_logger().info(f"Odom topic: {ODOM_TOPIC}")
        self.get_logger().info(f"IMU offset: {self.imu_config['imu_yaw_offset_deg']} deg")

    # ============================================================
    # CALLBACKS
    # ============================================================

    def roi_callback(self, msg):
        self.piece_in_roi = bool(msg.data)

    def imu_callback(self, msg):
        raw_yaw = quat_to_yaw_deg(msg.orientation)
        self.imu_raw_yaw_deg = normalize_angle_deg(raw_yaw)

        offset = float(self.imu_config.get("imu_yaw_offset_deg", 0.0))
        self.imu_map_yaw_deg = normalize_angle_deg(self.imu_raw_yaw_deg - offset)

    def odom_callback(self, msg):
        self.odom_x = float(msg.pose.pose.position.x)
        self.odom_y = float(msg.pose.pose.position.y)

    # ============================================================
    # ARCHIVOS
    # ============================================================

    def atomic_write_json(self, path, data):
        tmp_path = path.with_suffix(path.suffix + ".tmp")

        with open(tmp_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

        tmp_path.replace(path)

    def load_map(self):
        with open(MAP_FILE, "r", encoding="utf-8") as f:
            return json.load(f)

    def load_imu_config(self):
        default_config = {
            "use_imu_turning": True,
            "imu_yaw_offset_deg": 0.0,
            "imu_yaw_tolerance_deg": 2.0,
            "min_angular_speed": 0.05,
            "max_angular_speed": 0.18,
            "angular_kp": 0.010,
            "angular_command_inverted": True,
            "heading_hold_kp": 0.012,
            "heading_hold_max_correction": 0.10,
            "last_zero_raw_yaw_deg": None,
            "last_zero_time": None
        }

        if not IMU_CONFIG_FILE.exists():
            self.atomic_write_json(IMU_CONFIG_FILE, default_config)
            return default_config

        try:
            with open(IMU_CONFIG_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)

            for key, value in default_config.items():
                if key not in data:
                    data[key] = value

            return data

        except Exception:
            return default_config

    def make_start_state(self):
        start = self.map_config["start"]

        return {
            "pose": {
                "x": float(start["x"]),
                "y": float(start["y"]),
                "theta_deg": float(start.get("theta_deg", 0))
            },
            "linear_speed": DEFAULT_LINEAR_SPEED,
            "angular_speed": DEFAULT_ANGULAR_SPEED,
            "slow_pick_speed": DEFAULT_SLOW_PICK_SPEED,
            "last_action": "Estado creado desde START"
        }

    def state_pose_is_valid(self, state):
        try:
            pose = state["pose"]
            x = float(pose["x"])
            y = float(pose["y"])

            board = self.map_config["board"]
            max_x = float(board["height_x_mm"])
            max_y = float(board["width_y_mm"])

            if x < 0.0 or x > max_x:
                return False

            if y < 0.0 or y > max_y:
                return False

            return True

        except Exception:
            return False

    def load_or_create_state(self):
        if STATE_FILE.exists():
            try:
                if STATE_FILE.stat().st_size > 0:
                    with open(STATE_FILE, "r", encoding="utf-8") as f:
                        state = json.load(f)

                    if self.state_pose_is_valid(state):
                        return state

                    print("AVISO: robot_runtime_state.json fuera del tablero. Reseteando a START.")

            except Exception:
                pass

        state = self.make_start_state()
        self.atomic_write_json(STATE_FILE, state)
        return state

    def save_state(self):
        self.state["pose"] = {
            "x": round(float(self.x), 3),
            "y": round(float(self.y), 3),
            "theta_deg": round(float(self.theta_deg), 3)
        }

        self.state["linear_speed"] = float(self.linear_speed)
        self.state["angular_speed"] = float(self.angular_speed)
        self.state["slow_pick_speed"] = float(self.slow_pick_speed)
        self.state["last_action"] = "Base actualizada"

        self.atomic_write_json(STATE_FILE, self.state)

    def update_realtime_state(self, last_state_update):
        now = time.time()

        if now - last_state_update >= STATE_UPDATE_PERIOD:
            self.save_state()
            return now

        return last_state_update

    # ============================================================
    # ESPERAS SENSORIALES
    # ============================================================

    def wait_for_imu(self, timeout=2.0):
        if not self.imu_config.get("use_imu_turning", True):
            return False

        start = time.time()

        while rclpy.ok() and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.imu_raw_yaw_deg is not None and self.imu_map_yaw_deg is not None:
                return True

            time.sleep(0.02)

        return False

    def wait_for_odom(self, timeout=2.0):
        start = time.time()

        while rclpy.ok() and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.odom_x is not None and self.odom_y is not None:
                return True

            time.sleep(0.02)

        return False

    # ============================================================
    # IMU
    # ============================================================

    def sync_theta_from_imu(self):
        if self.imu_map_yaw_deg is None:
            return False

        self.theta_deg = normalize_angle_deg(self.imu_map_yaw_deg)
        self.save_state()
        return True

    def zero_imu_here_as_theta_zero(self):
        print("\n========== ZERO IMU EN START ==========")
        print("Leyendo yaw actual de /imu_broadcaster/imu...")

        if not self.wait_for_imu(timeout=5.0):
            print("ERROR: No se pudo leer IMU. Se mantiene offset anterior.")
            return False

        if self.imu_raw_yaw_deg is None:
            print("ERROR: IMU raw yaw es None. Se mantiene offset anterior.")
            return False

        new_offset = normalize_angle_deg(self.imu_raw_yaw_deg)

        self.imu_config["imu_yaw_offset_deg"] = float(new_offset)
        self.imu_config["last_zero_raw_yaw_deg"] = float(new_offset)
        self.imu_config["last_zero_time"] = time.strftime("%Y-%m-%d %H:%M:%S")

        self.atomic_write_json(IMU_CONFIG_FILE, self.imu_config)

        self.imu_map_yaw_deg = normalize_angle_deg(self.imu_raw_yaw_deg - new_offset)

        print(f"Raw yaw actual        : {self.imu_raw_yaw_deg:.2f} deg")
        print(f"Nuevo offset guardado: {new_offset:.2f} deg")
        print(f"Yaw mapa calibrado   : {self.imu_map_yaw_deg:.2f} deg")

        return True

    def print_imu_status(self):
        print("\n========== IMU ==========")
        print(f"Raw yaw  : {self.imu_raw_yaw_deg}")
        print(f"Offset   : {self.imu_config.get('imu_yaw_offset_deg')}")
        print(f"Map yaw  : {self.imu_map_yaw_deg}")
        print(f"Use IMU  : {self.imu_config.get('use_imu_turning')}")
        print(f"Inverted : {self.imu_config.get('angular_command_inverted')}")
        print("")
        print("Regla:")
        print("  theta_mapa = raw_yaw - offset")
        print("  En START físico, tras --reset-start, Map yaw debe ser ≈ 0")

    # ============================================================
    # POSE
    # ============================================================

    def format_pose(self):
        return (
            f"X={self.x:.1f} mm | "
            f"Y={self.y:.1f} mm | "
            f"Theta={self.theta_deg:.1f} deg"
        )

    def print_pose(self):
        print("\n========== POSE BASE ==========")
        print(self.format_pose())
        print(f"v={self.linear_speed:.3f} m/s | w={self.angular_speed:.3f} rad/s")

    def send_base_reset(self):
        print("\n========== RESET BASE ARDUINO ==========")
        print("Enviando /robot/base_reset = RESET")

        msg = String()
        msg.data = "RESET"

        # Publicamos varias veces para asegurar entrega al bridge interno.
        for _ in range(5):
            self.base_reset_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.10)

        # Stop extra por seguridad después del reset.
        self.stop_robot()

        print("Reset base enviado.")

    def reset_to_start(self):
        start = self.map_config["start"]

        print("\n========== RESET START ==========")
        print("IMPORTANTE:")
        print("  El robot debe estar físicamente en START.")
        print("  La orientación física debe ser theta=0 del mapa.")
        print("  Se calibrará el offset IMU automáticamente.")

        self.send_base_reset()
        time.sleep(0.5)
        

        imu_zero_ok = self.zero_imu_here_as_theta_zero()

        self.x = float(start["x"])
        self.y = float(start["y"])
        self.theta_deg = float(start.get("theta_deg", 0))

        self.save_state()

        print("\nPose lógica reseteada a START.")
        self.print_pose()

        if not imu_zero_ok:
            print("\nAVISO: START reseteado, pero IMU NO calibrada.")

    def set_pose(self, x, y, theta_deg):
        self.x = float(x)
        self.y = float(y)
        self.theta_deg = normalize_angle_deg(theta_deg)

        self.save_state()
        self.print_pose()

    # ============================================================
    # CMD VEL
    # ============================================================

    def publish_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def send_cmd_vel(self, linear, angular):
        self.publish_velocity(linear, angular)

    def stop_robot(self):
        for _ in range(8):
            self.publish_velocity(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.001)
            time.sleep(0.04)

    # ============================================================
    # MODELO INTERNO
    # ============================================================

    def update_pose_forward(self, distance_m):
        distance_mm_value = distance_m * 1000.0
        theta_rad = deg2rad(self.theta_deg)

        self.x += distance_mm_value * math.cos(theta_rad)
        self.y += distance_mm_value * math.sin(theta_rad)

    def update_pose_rotation(self, delta_deg):
        self.theta_deg = normalize_angle_deg(self.theta_deg + delta_deg)

    # ============================================================
    # MOVIMIENTOS
    # ============================================================

    def rotate_degrees_open_loop(self, delta_deg):
        delta_deg = normalize_angle_deg(delta_deg)

        if abs(delta_deg) < 1.0:
            return

        sign = 1.0 if delta_deg >= 0.0 else -1.0
        duration = abs(deg2rad(delta_deg)) / max(self.angular_speed, 0.001)

        print(
            f"\nGIRO OPEN LOOP {delta_deg:+.1f} deg | "
            f"w={-sign * self.angular_speed:.3f} rad/s | "
            f"t={duration:.2f}s"
        )

        start = time.time()
        last_print = start
        last_state_update = start

        while rclpy.ok():
            now = time.time()
            elapsed = now - start

            if elapsed >= duration:
                break

            self.publish_velocity(0.0, -sign * self.angular_speed)
            rclpy.spin_once(self, timeout_sec=0.001)

            progress = min(1.0, elapsed / max(duration, 0.001))
            self.theta_deg = normalize_angle_deg(
                self.state["pose"]["theta_deg"] + delta_deg * progress
            )

            last_state_update = self.update_realtime_state(last_state_update)

            if now - last_print >= PRINT_PERIOD:
                print(self.format_pose())
                last_print = now

            time.sleep(CONTROL_DT)

        self.stop_robot()
        self.update_pose_rotation(delta_deg)
        self.save_state()
        time.sleep(0.15)

    def rotate_degrees_imu(self, delta_deg):
        delta_deg = normalize_angle_deg(delta_deg)

        if abs(delta_deg) < 0.8:
            return

        if not self.wait_for_imu(timeout=2.0):
            print("\nIMU no disponible. Usando giro por tiempo.")
            self.rotate_degrees_open_loop(delta_deg)
            return

        self.sync_theta_from_imu()

        start_yaw = normalize_angle_deg(self.theta_deg)
        target_yaw = normalize_angle_deg(start_yaw + delta_deg)

        direction = 1.0 if delta_deg > 0.0 else -1.0
        angle_rad_total = abs(math.radians(delta_deg))

        w_max_user = abs(float(self.angular_speed))
        w_max_user = max(0.03, w_max_user)

        profile = self.build_angular_motion_profile(
            angle_rad=angle_rad_total,
            w_max=w_max_user,
            accel=ANGULAR_ACCEL,
            decel=ANGULAR_DECEL
        )

        tolerance = float(self.imu_config.get("imu_yaw_tolerance_deg", YAW_TOLERANCE_PROFILE_DEG))
        tolerance = max(tolerance, YAW_TOLERANCE_PROFILE_DEG)

        inverted = bool(self.imu_config.get("angular_command_inverted", True))

        timeout = max(8.0, profile["total_time"] * 2.5 + 4.0)

        print("\n========== GIRO IMU PERFIL PLANIFICADO ==========")
        print(f"Yaw inicial mapa : {start_yaw:.2f} deg")
        print(f"Delta pedido     : {delta_deg:+.2f} deg")
        print(f"Yaw objetivo mapa: {target_yaw:.2f} deg")
        print(f"W usuario        : {w_max_user:.3f} rad/s")
        print(f"W pico real      : {profile['w_peak']:.3f} rad/s")
        print(f"Accel angular    : {ANGULAR_ACCEL:.3f} rad/s²")
        print(f"Decel angular    : {ANGULAR_DECEL:.3f} rad/s²")
        print(f"Perfil           : {'TRIANGULAR' if profile['triangular'] else 'TRAPEZOIDAL'}")
        print(
            f"Ángulos perfil   : "
            f"acc={math.degrees(profile['a_accel']):.1f}° | "
            f"cruise={math.degrees(profile['a_cruise']):.1f}° | "
            f"dec={math.degrees(profile['a_decel']):.1f}°"
        )
        print(
            f"Tiempos          : "
            f"acc={profile['t_accel']:.2f}s | "
            f"cruise={profile['t_cruise']:.2f}s | "
            f"dec={profile['t_decel']:.2f}s | "
            f"total={profile['total_time']:.2f}s"
        )
        print(f"Tolerancia       : ±{tolerance:.2f} deg")
        print(f"Offset IMU       : {self.imu_config.get('imu_yaw_offset_deg')} deg")
        print(f"Invertido cmd_z  : {inverted}")
        print(f"Timeout          : {timeout:.2f} s")

        start = time.time()
        last_print = start
        last_state_update = start
        stable_since = None

        while rclpy.ok():
            now = time.time()
            elapsed = now - start

            rclpy.spin_once(self, timeout_sec=0.02)

            if self.imu_map_yaw_deg is None:
                continue

            current_yaw = normalize_angle_deg(self.imu_map_yaw_deg)
            self.theta_deg = current_yaw

            # Progreso real desde el yaw inicial, en el sentido del giro pedido.
            real_progress_deg_signed = normalize_angle_deg(current_yaw - start_yaw)
            real_progress_rad = math.radians(real_progress_deg_signed) * direction

            # Evitar valores negativos por ruido/signo.
            real_progress_rad = max(0.0, real_progress_rad)

            # Referencia planificada.
            a_ref, w_ref = self.sample_angular_motion_profile(profile, elapsed)

            # Error de seguimiento de perfil.
            profile_error_rad = a_ref - real_progress_rad

            correction = ANGULAR_PROFILE_KP * profile_error_rad
            correction = max(-MAX_ANGULAR_CORRECTION, min(MAX_ANGULAR_CORRECTION, correction))

            w_cmd_abs = w_ref + correction

            # Nunca superar el pico calculado ni el máximo de usuario.
            w_cmd_abs = min(w_cmd_abs, profile["w_peak"], w_max_user)
            w_cmd_abs = max(0.0, w_cmd_abs)

            # Error final al objetivo.
            yaw_error_deg = normalize_angle_deg(target_yaw - current_yaw)
            remaining_deg = abs(yaw_error_deg)

            # Últimos grados: velocidad fina.
            if remaining_deg <= FINAL_CRAWL_ANGLE_DEG:
                w_cmd_abs = min(w_cmd_abs, FINAL_CRAWL_ANGULAR_SPEED)

            # Si todavía queda bastante, evitar que se muera por rozamiento.
            if remaining_deg > FINAL_CRAWL_ANGLE_DEG:
                w_cmd_abs = max(MIN_ANGULAR_SPEED, w_cmd_abs)

            if remaining_deg <= tolerance:
                if stable_since is None:
                    stable_since = now

                if now - stable_since >= 0.20:
                    print(f"GIRO IMU OK. Error final: {yaw_error_deg:+.2f} deg")
                    break
            else:
                stable_since = None

            if elapsed > profile["total_time"] + 1.0 and remaining_deg <= tolerance * 2.0:
                print(f"Perfil terminado y yaw cercano. Error final: {yaw_error_deg:+.2f} deg")
                break

            if elapsed >= timeout:
                if remaining_deg <= tolerance * 1.5:
                    print(f"GIRO IMU OK POR CERCANÍA. Error final: {yaw_error_deg:+.2f} deg")
                else:
                    print(f"TIMEOUT GIRO IMU. Error final: {yaw_error_deg:+.2f} deg")
                break



            # Signo de giro según error real restante, no solo según delta inicial.
            # Esto permite corregir si se pasa un poco.
            cmd_direction = 1.0 if yaw_error_deg > 0.0 else -1.0
            cmd_z = cmd_direction * w_cmd_abs

            if inverted:
                cmd_z *= -1.0

            self.publish_velocity(0.0, cmd_z)

            last_state_update = self.update_realtime_state(last_state_update)

            if now - last_print >= PRINT_PERIOD:
                print(
                    f"Yaw={current_yaw:+7.2f} | "
                    f"Target={target_yaw:+7.2f} | "
                    f"Err={yaw_error_deg:+7.2f} | "
                    f"prog={math.degrees(real_progress_rad):6.1f}/{abs(delta_deg):.1f}° | "
                    f"ref={math.degrees(a_ref):6.1f}° | "
                    f"perr={math.degrees(profile_error_rad):+6.1f}° | "
                    f"w_ref={w_ref:.3f} | "
                    f"w_cmd={cmd_z:+.3f} | "
                    f"t={elapsed:.1f}/{profile['total_time']:.1f}s"
                )
                last_print = now

            time.sleep(CONTROL_DT)

        self.stop_robot()

        if self.imu_map_yaw_deg is not None:
            self.theta_deg = normalize_angle_deg(self.imu_map_yaw_deg)
        else:
            self.theta_deg = target_yaw

        self.save_state()
        time.sleep(0.15)

    def rotate_degrees(self, delta_deg):
        if self.imu_config.get("use_imu_turning", True):
            self.rotate_degrees_imu(delta_deg)
        else:
            self.rotate_degrees_open_loop(delta_deg)

    def build_linear_motion_profile(self, distance_m, v_max, accel, decel):
        """
        Construye un perfil triangular/trapezoidal para un avance recto.

        distance_m: distancia total positiva en metros
        v_max: velocidad máxima permitida por usuario
        accel: aceleración máxima positiva
        decel: deceleración máxima positiva

        Devuelve un dict con:
        - v_peak: velocidad pico real
        - t_accel, t_cruise, t_decel
        - d_accel, d_cruise, d_decel
        - total_time
        - triangular: True/False
        """
        distance_m = max(0.0, float(distance_m))
        v_max = max(0.001, abs(float(v_max)))
        accel = max(0.001, abs(float(accel)))
        decel = max(0.001, abs(float(decel)))

        # Velocidad pico posible si NO hay tramo de velocidad constante.
        # Sale de:
        # D = v²/(2a) + v²/(2d)
        v_peak_possible = math.sqrt(
            max(0.0, 2.0 * distance_m * accel * decel / (accel + decel))
        )

        v_peak = min(v_max, v_peak_possible)

        d_accel = (v_peak * v_peak) / (2.0 * accel)
        d_decel = (v_peak * v_peak) / (2.0 * decel)

        d_cruise = distance_m - d_accel - d_decel

        if d_cruise > 0.0 and v_peak >= v_max * 0.999:
            triangular = False
            t_accel = v_peak / accel
            t_cruise = d_cruise / max(v_peak, 0.001)
            t_decel = v_peak / decel
        else:
            triangular = True
            d_cruise = 0.0

            # Recalcular limpio para triangular.
            v_peak = v_peak_possible
            v_peak = min(v_peak, v_max)

            d_accel = (v_peak * v_peak) / (2.0 * accel)
            d_decel = distance_m - d_accel

            if d_decel < 0.0:
                d_decel = (v_peak * v_peak) / (2.0 * decel)

            t_accel = v_peak / accel
            t_cruise = 0.0
            t_decel = v_peak / decel

        total_time = t_accel + t_cruise + t_decel

        return {
            "distance_m": distance_m,
            "v_max_user": v_max,
            "v_peak": v_peak,
            "accel": accel,
            "decel": decel,
            "d_accel": d_accel,
            "d_cruise": d_cruise,
            "d_decel": d_decel,
            "t_accel": t_accel,
            "t_cruise": t_cruise,
            "t_decel": t_decel,
            "total_time": total_time,
            "triangular": triangular
        }


    def sample_linear_motion_profile(self, profile, t):
        """
        Devuelve posición de referencia s_ref y velocidad de referencia v_ref
        para el tiempo t del perfil.
        """
        t = max(0.0, float(t))

        D = profile["distance_m"]
        v_peak = profile["v_peak"]
        accel = profile["accel"]
        decel = profile["decel"]

        t_accel = profile["t_accel"]
        t_cruise = profile["t_cruise"]
        t_decel = profile["t_decel"]

        d_accel = profile["d_accel"]
        d_cruise = profile["d_cruise"]

        total_time = profile["total_time"]

        if t <= 0.0:
            return 0.0, 0.0

        # Fase aceleración.
        if t < t_accel:
            v_ref = accel * t
            s_ref = 0.5 * accel * t * t
            return min(D, s_ref), max(0.0, v_ref)

        # Fase crucero.
        t2 = t - t_accel

        if t2 < t_cruise:
            v_ref = v_peak
            s_ref = d_accel + v_peak * t2
            return min(D, s_ref), max(0.0, v_ref)

        # Fase deceleración.
        t3 = t2 - t_cruise

        if t3 < t_decel:
            v_ref = max(0.0, v_peak - decel * t3)
            s_ref = d_accel + d_cruise + v_peak * t3 - 0.5 * decel * t3 * t3
            return min(D, s_ref), max(0.0, v_ref)

        return D, 0.0

    def final_crawl_to_distance(
        self,
        target_distance_m,
        traveled_m,
        odom_start_x,
        odom_start_y,
        sign,
        heading_target=None,
        use_odom=True
    ):
        """
        Aproximación final lenta.
        Se usa después del perfil rápido si la odometría dice que aún falta distancia.
        """
        remaining_m = target_distance_m - traveled_m

        if remaining_m <= FINAL_APPROACH_TOLERANCE:
            return traveled_m

        if remaining_m > FINAL_APPROACH_MAX_DISTANCE:
            print(
                f"AVISO: corrección final demasiado grande "
                f"({remaining_m:.3f} m). No se corrige por seguridad."
            )
            return traveled_m

        print("\n========== APROXIMACIÓN FINAL ==========")
        print(f"Restante inicial: {remaining_m:.3f} m")
        print(f"Velocidad fina  : {FINAL_APPROACH_SPEED:.3f} m/s")
        print(f"Tolerancia      : {FINAL_APPROACH_TOLERANCE:.3f} m")

        heading_kp = float(self.imu_config.get("heading_hold_kp", 0.012))
        max_correction_w = float(self.imu_config.get("heading_hold_max_correction", 0.10))
        inverted = bool(self.imu_config.get("angular_command_inverted", True))

        use_imu_heading_hold = (
            self.imu_config.get("use_imu_turning", True)
            and heading_target is not None
            and self.imu_map_yaw_deg is not None
        )

        start = time.time()
        last_print = start
        last_state_update = start
        last_pose_distance_m = traveled_m

        while rclpy.ok():
            now = time.time()
            elapsed = now - start

            rclpy.spin_once(self, timeout_sec=0.01)

            if use_odom and self.odom_x is not None and self.odom_y is not None:
                dx_odom = self.odom_x - odom_start_x
                dy_odom = self.odom_y - odom_start_y
                traveled_m = math.hypot(dx_odom, dy_odom)
            else:
                traveled_m += FINAL_APPROACH_SPEED * 0.05

            remaining_m = target_distance_m - traveled_m

            if remaining_m <= FINAL_APPROACH_TOLERANCE:
                print(
                    f"Aproximación final OK. "
                    f"Recorrido={traveled_m:.3f} m | "
                    f"Restante={remaining_m:.3f} m"
                )
                break

            if elapsed >= FINAL_APPROACH_TIMEOUT:
                print(
                    f"TIMEOUT aproximación final. "
                    f"Recorrido={traveled_m:.3f} m | "
                    f"Restante={remaining_m:.3f} m"
                )
                break

            angular_correction = 0.0

            if use_imu_heading_hold:
                current_yaw = normalize_angle_deg(self.imu_map_yaw_deg)
                error = normalize_angle_deg(heading_target - current_yaw)

                self.theta_deg = current_yaw

                correction_abs = abs(error) * heading_kp
                correction_abs = min(max_correction_w, correction_abs)

                direction = 1.0 if error > 0.0 else -1.0
                angular_correction = direction * correction_abs

                if inverted:
                    angular_correction *= -1.0

            # Actualizar gemelo digital.
            delta_pose_m = traveled_m - last_pose_distance_m

            if delta_pose_m > 0.0005:
                self.update_pose_forward(sign * delta_pose_m)
                last_pose_distance_m = traveled_m

            last_state_update = self.update_realtime_state(last_state_update)

            self.publish_velocity(sign * FINAL_APPROACH_SPEED, angular_correction)

            if now - last_print >= PRINT_PERIOD:
                print(
                    f"{self.format_pose()} | "
                    f"final_real={traveled_m:.3f}/{target_distance_m:.3f} | "
                    f"rest={remaining_m:.3f} | "
                    f"v={sign * FINAL_APPROACH_SPEED:+.3f} | "
                    f"corr_z={angular_correction:+.3f}"
                )
                last_print = now

            time.sleep(CONTROL_DT)

        return traveled_m

    def drive_distance_mm(self, distance, heading_target=None):
        distance = float(distance)

        if abs(distance) < 5.0:
            return

        sign = 1.0 if distance >= 0.0 else -1.0
        target_distance_m = abs(distance) / 1000.0

        use_odom = self.wait_for_odom(timeout=2.0)

        if use_odom:
            odom_start_x = self.odom_x
            odom_start_y = self.odom_y
        else:
            odom_start_x = None
            odom_start_y = None

        use_imu_heading_hold = (
            self.imu_config.get("use_imu_turning", True)
            and heading_target is not None
            and self.wait_for_imu(timeout=1.0)
        )

        heading_target = normalize_angle_deg(heading_target) if heading_target is not None else None

        heading_kp = float(self.imu_config.get("heading_hold_kp", 0.012))
        max_correction_w = float(self.imu_config.get("heading_hold_max_correction", 0.10))
        inverted = bool(self.imu_config.get("angular_command_inverted", True))

        # --linear-speed ahora es velocidad máxima permitida.
        v_max_user = abs(float(self.linear_speed))
        v_max_user = max(0.01, v_max_user)

        profile = self.build_linear_motion_profile(
            distance_m=target_distance_m,
            v_max=v_max_user,
            accel=LINEAR_ACCEL,
            decel=LINEAR_DECEL
        )

        timeout = max(10.0, profile["total_time"] * 2.5 + 4.0)

        print("\n========== AVANCE PERFIL PLANIFICADO ==========")
        print(f"Distancia objetivo : {target_distance_m:.3f} m")
        print(f"Velocidad usuario  : {v_max_user:.3f} m/s")
        print(f"Velocidad pico real: {profile['v_peak']:.3f} m/s")
        print(f"Aceleración        : {LINEAR_ACCEL:.3f} m/s²")
        print(f"Deceleración       : {LINEAR_DECEL:.3f} m/s²")
        print(f"Perfil             : {'TRIANGULAR' if profile['triangular'] else 'TRAPEZOIDAL'}")
        print(
            f"Distancias         : "
            f"acc={profile['d_accel']:.3f} | "
            f"cruise={profile['d_cruise']:.3f} | "
            f"dec={profile['d_decel']:.3f}"
        )
        print(
            f"Tiempos            : "
            f"acc={profile['t_accel']:.2f}s | "
            f"cruise={profile['t_cruise']:.2f}s | "
            f"dec={profile['t_decel']:.2f}s | "
            f"total={profile['total_time']:.2f}s"
        )
        print(f"Timeout            : {timeout:.1f}s")

        if use_odom:
            print(f"Odom start         : x={odom_start_x:.3f}, y={odom_start_y:.3f}")
        else:
            print("AVISO: No hay odometría. Se usará avance por tiempo aproximado.")

        if use_imu_heading_hold:
            print(f"Heading hold IMU   : objetivo {heading_target:.2f} deg")

        start = time.time()
        last_time = start
        last_print = start
        last_state_update = start

        traveled_m = 0.0
        last_pose_distance_m = 0.0
        current_v_cmd = 0.0

        while rclpy.ok():
            now = time.time()
            elapsed = now - start
            dt_loop = max(0.001, now - last_time)
            last_time = now

            rclpy.spin_once(self, timeout_sec=0.01)

            # ====================================================
            # DISTANCIA REAL RECORRIDA
            # ====================================================
            if use_odom and self.odom_x is not None and self.odom_y is not None:
                dx_odom = self.odom_x - odom_start_x
                dy_odom = self.odom_y - odom_start_y
                traveled_m = math.hypot(dx_odom, dy_odom)
            else:
                traveled_m += abs(current_v_cmd) * dt_loop

            remaining_m = target_distance_m - traveled_m

            # ====================================================
            # REFERENCIA PLANIFICADA s(t), v(t)
            # ====================================================
            s_ref, v_ref = self.sample_linear_motion_profile(profile, elapsed)

            # Error respecto a la curva planificada.
            pos_error = s_ref - traveled_m

            # Corrección proporcional suave, limitada.
            correction = LINEAR_PROFILE_KP * pos_error
            correction = max(-MAX_PROFILE_CORRECTION, min(MAX_PROFILE_CORRECTION, correction))

            v_cmd_abs = v_ref + correction

            # Nunca superar la velocidad pico calculada ni la velocidad máxima de usuario.
            v_cmd_abs = min(v_cmd_abs, profile["v_peak"], v_max_user)

            # No permitir velocidad negativa.
            v_cmd_abs = max(0.0, v_cmd_abs)

            # Si queda distancia suficiente, evitar que se muera por rozamiento.
            if remaining_m > FINAL_CRAWL_DISTANCE:
                v_cmd_abs = max(MIN_LINEAR_SPEED, v_cmd_abs)

            # Últimos centímetros: modo arrastre fino.
            if remaining_m <= FINAL_CRAWL_DISTANCE:
                v_cmd_abs = min(v_cmd_abs, FINAL_CRAWL_SPEED)

            # Si ya estamos prácticamente encima, paramos.
            if remaining_m <= POSITION_TOLERANCE:
                print(
                    f"Distancia alcanzada. "
                    f"Recorrido={traveled_m:.3f} m | "
                    f"Restante={remaining_m:.3f} m"
                )
                break

            # Si el perfil terminó y la distancia está muy cerca, paramos.
            if elapsed > profile["total_time"] + 1.0 and abs(remaining_m) <= 0.020:
                print(
                    f"Perfil terminado y objetivo cercano. "
                    f"Recorrido={traveled_m:.3f} m | "
                    f"Restante={remaining_m:.3f} m"
                )
                break

            if elapsed >= timeout:
                print(
                    f"TIMEOUT avance. "
                    f"Recorrido={traveled_m:.3f} m | "
                    f"Restante={remaining_m:.3f} m | "
                    f"s_ref={s_ref:.3f} m"
                )
                break

            # ====================================================
            # LIMITADOR FINAL DE RAMPA DEL COMANDO REAL
            # ====================================================
            # Aunque el perfil/corrección pidan un cambio brusco,
            # nunca dejamos que el cmd_vel cambie de golpe.
            if v_cmd_abs > current_v_cmd:
                current_v_cmd = min(
                    v_cmd_abs,
                    current_v_cmd + MAX_LINEAR_CMD_ACCEL * dt_loop
                )
            else:
                current_v_cmd = max(
                    v_cmd_abs,
                    current_v_cmd - MAX_LINEAR_CMD_DECEL * dt_loop
                )

            linear_cmd = sign * current_v_cmd

            # ====================================================
            # CORRECCIÓN DE HEADING CON IMU
            # ====================================================
            angular_correction = 0.0

            if use_imu_heading_hold and self.imu_map_yaw_deg is not None:
                current_yaw = normalize_angle_deg(self.imu_map_yaw_deg)
                error = normalize_angle_deg(heading_target - current_yaw)

                self.theta_deg = current_yaw

                correction_abs = abs(error) * heading_kp
                correction_abs = min(max_correction_w, correction_abs)

                direction = 1.0 if error > 0.0 else -1.0
                angular_correction = direction * correction_abs

                if inverted:
                    angular_correction *= -1.0

            # ====================================================
            # GEMELO DIGITAL EN TIEMPO REAL
            # ====================================================
            delta_pose_m = traveled_m - last_pose_distance_m

            if delta_pose_m > 0.0005:
                self.update_pose_forward(sign * delta_pose_m)
                last_pose_distance_m = traveled_m

            last_state_update = self.update_realtime_state(last_state_update)

            self.publish_velocity(linear_cmd, angular_correction)

            if now - last_print >= PRINT_PERIOD:
                if use_imu_heading_hold and self.imu_map_yaw_deg is not None:
                    current_yaw = normalize_angle_deg(self.imu_map_yaw_deg)
                    heading_error = normalize_angle_deg(heading_target - current_yaw)

                    print(
                        f"{self.format_pose()} | "
                        f"real={traveled_m:.3f}/{target_distance_m:.3f} | "
                        f"ref={s_ref:.3f} | "
                        f"err={pos_error:+.3f} | "
                        f"rest={remaining_m:.3f} | "
                        f"v_ref={v_ref:.3f} | "
                        f"v_cmd={linear_cmd:+.3f} | "
                        f"yaw={current_yaw:+.2f} | "
                        f"err_yaw={heading_error:+.2f} | "
                        f"corr_z={angular_correction:+.3f}"
                    )
                else:
                    print(
                        f"{self.format_pose()} | "
                        f"real={traveled_m:.3f}/{target_distance_m:.3f} | "
                        f"ref={s_ref:.3f} | "
                        f"err={pos_error:+.3f} | "
                        f"rest={remaining_m:.3f} | "
                        f"v_ref={v_ref:.3f} | "
                        f"v_cmd={linear_cmd:+.3f}"
                    )

                last_print = now

            time.sleep(CONTROL_DT)

        # Parada suave inicial del perfil.
        self.stop_robot()
        time.sleep(0.05)

        # Si el perfil ha quedado corto, hacemos aproximación final lenta.
        if use_odom:
            remaining_after_profile = target_distance_m - traveled_m

            if remaining_after_profile > FINAL_APPROACH_TOLERANCE:
                traveled_m = self.final_crawl_to_distance(
                    target_distance_m=target_distance_m,
                    traveled_m=traveled_m,
                    odom_start_x=odom_start_x,
                    odom_start_y=odom_start_y,
                    sign=sign,
                    heading_target=heading_target,
                    use_odom=use_odom
                )

        self.stop_robot()

        if self.imu_config.get("use_imu_turning", True):
            if self.wait_for_imu(timeout=0.5):
                self.sync_theta_from_imu()

        remaining_pose_m = max(0.0, traveled_m - last_pose_distance_m)

        if remaining_pose_m > 0.0005:
            self.update_pose_forward(sign * remaining_pose_m)

        self.save_state()
        time.sleep(0.15)

    def heading_to_point(self, target):
        dx = float(target["x"]) - self.x
        dy = float(target["y"]) - self.y
        return rad2deg(math.atan2(dy, dx))

    def move_to_point(self, target, final_theta=None):
        print("\n========== MOVE TO POINT ==========")

        if self.imu_config.get("use_imu_turning", True):
            if self.wait_for_imu(timeout=1.0):
                self.sync_theta_from_imu()

        print(f"Actual : {self.format_pose()}")
        print(f"Destino: X={target['x']:.1f} | Y={target['y']:.1f}")

        dx = float(target["x"]) - self.x
        dy = float(target["y"]) - self.y
        dist = math.hypot(dx, dy)

        if dist > 5.0:
            heading = self.heading_to_point(target)
            delta = normalize_angle_deg(heading - self.theta_deg)

            print(f"Heading objetivo: {heading:.1f} deg")
            self.rotate_degrees(delta)
            self.drive_distance_mm(dist, heading_target=heading)

        if final_theta is not None:
            if self.imu_config.get("use_imu_turning", True):
                if self.wait_for_imu(timeout=1.0):
                    self.sync_theta_from_imu()

            delta_final = normalize_angle_deg(float(final_theta) - self.theta_deg)
            print(f"\nOrientación final objetivo: {float(final_theta):.1f} deg")
            self.rotate_degrees(delta_final)

        print("\nLlegada lógica:")
        self.print_pose()

    def build_angular_motion_profile(self, angle_rad, w_max, accel, decel):
        """
        Perfil triangular/trapezoidal para un giro.

        angle_rad: ángulo total positivo en radianes
        w_max: velocidad angular máxima permitida
        accel/decel: aceleración/deceleración angular en rad/s²
        """
        angle_rad = max(0.0, float(angle_rad))
        w_max = max(0.001, abs(float(w_max)))
        accel = max(0.001, abs(float(accel)))
        decel = max(0.001, abs(float(decel)))

        w_peak_possible = math.sqrt(
            max(0.0, 2.0 * angle_rad * accel * decel / (accel + decel))
        )

        w_peak = min(w_max, w_peak_possible)

        a_accel = (w_peak * w_peak) / (2.0 * accel)
        a_decel = (w_peak * w_peak) / (2.0 * decel)

        a_cruise = angle_rad - a_accel - a_decel

        if a_cruise > 0.0 and w_peak >= w_max * 0.999:
            triangular = False
            t_accel = w_peak / accel
            t_cruise = a_cruise / max(w_peak, 0.001)
            t_decel = w_peak / decel
        else:
            triangular = True
            a_cruise = 0.0

            w_peak = min(w_peak_possible, w_max)

            a_accel = (w_peak * w_peak) / (2.0 * accel)
            a_decel = angle_rad - a_accel

            if a_decel < 0.0:
                a_decel = (w_peak * w_peak) / (2.0 * decel)

            t_accel = w_peak / accel
            t_cruise = 0.0
            t_decel = w_peak / decel

        total_time = t_accel + t_cruise + t_decel

        return {
            "angle_rad": angle_rad,
            "angle_deg": math.degrees(angle_rad),
            "w_max_user": w_max,
            "w_peak": w_peak,
            "accel": accel,
            "decel": decel,
            "a_accel": a_accel,
            "a_cruise": a_cruise,
            "a_decel": a_decel,
            "t_accel": t_accel,
            "t_cruise": t_cruise,
            "t_decel": t_decel,
            "total_time": total_time,
            "triangular": triangular
        }


    def sample_angular_motion_profile(self, profile, t):
        """
        Devuelve ángulo de referencia acumulado y velocidad angular de referencia.
        Todo en radianes / rad/s.
        """
        t = max(0.0, float(t))

        A = profile["angle_rad"]
        w_peak = profile["w_peak"]
        accel = profile["accel"]
        decel = profile["decel"]

        t_accel = profile["t_accel"]
        t_cruise = profile["t_cruise"]
        t_decel = profile["t_decel"]

        a_accel = profile["a_accel"]
        a_cruise = profile["a_cruise"]

        if t <= 0.0:
            return 0.0, 0.0

        # Fase aceleración.
        if t < t_accel:
            w_ref = accel * t
            a_ref = 0.5 * accel * t * t
            return min(A, a_ref), max(0.0, w_ref)

        # Fase crucero.
        t2 = t - t_accel

        if t2 < t_cruise:
            w_ref = w_peak
            a_ref = a_accel + w_peak * t2
            return min(A, a_ref), max(0.0, w_ref)

        # Fase deceleración.
        t3 = t2 - t_cruise

        if t3 < t_decel:
            w_ref = max(0.0, w_peak - decel * t3)
            a_ref = a_accel + a_cruise + w_peak * t3 - 0.5 * decel * t3 * t3
            return min(A, a_ref), max(0.0, w_ref)

        return A, 0.0

    # ============================================================
    # SLOW PICK
    # ============================================================

    def slow_pick_enter(self, speed=DEFAULT_SLOW_PICK_SPEED):
        print("\n========== SLOW PICK ENTER ==========")
        print("El robot avanzará lentamente.")
        print("Pulsa ENTER para parar.\n")

        input("Pulsa ENTER para empezar el avance lento...")

        self.stop_robot()
        time.sleep(0.2)

        start = time.time()
        last_print = start
        last_state_update = start
        last_pose_distance_m = 0.0

        try:
            while rclpy.ok():
                self.publish_velocity(float(speed), 0.0)
                rclpy.spin_once(self, timeout_sec=0.001)

                now = time.time()
                elapsed = now - start
                distance_m = float(speed) * elapsed
                delta_pose_m = distance_m - last_pose_distance_m

                if delta_pose_m > 0.0005:
                    self.update_pose_forward(delta_pose_m)
                    last_pose_distance_m = distance_m

                last_state_update = self.update_realtime_state(last_state_update)

                if now - last_print >= PRINT_PERIOD:
                    print(self.format_pose())
                    last_print = now

                if select.select([sys.stdin], [], [], 0.0)[0]:
                    sys.stdin.readline()
                    break

                time.sleep(CONTROL_DT)

        finally:
            self.stop_robot()

        self.save_state()

        print("\nSLOW PICK ENTER terminado.")
        self.print_pose()

    def slow_pick_roi(self, speed=DEFAULT_SLOW_PICK_SPEED, timeout=12.0, min_time=0.4):
        speed = float(speed)

        print("\n========== SLOW PICK ROI ==========")
        print(f"Topic ROI : {ROI_TOPIC}")
        print(f"Velocidad : {speed:.3f} m/s")
        print(f"Timeout   : {timeout:.1f} s")
        print(f"Min time  : {min_time:.1f} s")

        self.stop_robot()
        time.sleep(0.15)

        if self.imu_config.get("use_imu_turning", True):
            if self.wait_for_imu(timeout=1.0):
                self.sync_theta_from_imu()

        self.piece_in_roi = False

        warmup_start = time.time()
        while time.time() - warmup_start < 0.3 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

        start_time = time.time()
        last_print = start_time
        last_state_update = start_time
        last_pose_distance_m = 0.0

        try:
            while rclpy.ok():
                now = time.time()
                elapsed = now - start_time

                rclpy.spin_once(self, timeout_sec=0.02)

                distance_m = speed * elapsed
                delta_pose_m = distance_m - last_pose_distance_m

                if delta_pose_m > 0.0005:
                    self.update_pose_forward(delta_pose_m)
                    last_pose_distance_m = distance_m

                last_state_update = self.update_realtime_state(last_state_update)

                if elapsed >= min_time and self.piece_in_roi:
                    print("ROI detectado: pieza dentro del ROI. Parando.")
                    break

                if elapsed >= timeout:
                    print("TIMEOUT slow pick ROI. Parando por seguridad.")
                    break

                self.publish_velocity(speed, 0.0)

                if now - last_print >= PRINT_PERIOD:
                    print(f"{self.format_pose()} | ROI={self.piece_in_roi}")
                    last_print = now

                time.sleep(CONTROL_DT)

        finally:
            self.stop_robot()

        self.save_state()

        print("\nSLOW PICK ROI terminado.")
        self.print_pose()
        return True

    # ============================================================
    # RUTAS
    # ============================================================

    def get_node_theta(self, node_name):
        if node_name == "START":
            return self.map_config.get("start", {}).get("theta_deg", None)

        if node_name.startswith("AA_"):
            name = node_name.replace("AA_", "", 1)
            return self.map_config.get("approach_almacenes", {}).get(name, {}).get("theta_deg", None)

        if node_name.startswith("AD_"):
            name = node_name.replace("AD_", "", 1)
            return self.map_config.get("approach_despensas", {}).get(name, {}).get("theta_deg", None)

        return None

    def execute_route(self, origin, destination, dry_run=False):
        planner = PathPlanner(MAP_FILE)
        plan = planner.plan(origin, destination)

        route = plan["route"]

        print("\n========== RUTA PLANIFICADA ==========")
        print(f"Origen : {origin}")
        print(f"Destino: {destination}")
        print(f"Distancia: {plan['distance_mm']:.1f} mm")
        print("Nodos:")

        for p in route:
            print(
                f"  {p['name']}: "
                f"X={p['x']:.1f}, Y={p['y']:.1f}, "
                f"Theta={p.get('theta_deg', '-')}"
            )

        if dry_run:
            print("\nDRY RUN: no se mueve el robot.")
            return True

        if not getattr(self, "auto_confirm", False):
            input("\nPulsa ENTER para ejecutar esta ruta en el robot real...")

        for i, point in enumerate(route):
            if i == 0:
                continue

            is_last = i == len(route) - 1

            final_theta = None
            if is_last:
                final_theta = point.get("theta_deg", None)

                if final_theta is None:
                    final_theta = self.get_node_theta(destination)

            print(f"\n--- Tramo {i}/{len(route)-1}: {route[i-1]['name']} → {point['name']} ---")
            self.move_to_point(point, final_theta=final_theta)

        self.stop_robot()
        self.save_state()

        print("\nRuta ejecutada.")
        self.print_pose()
        return True

    def close(self):
        self.stop_robot()
        self.save_state()


def main(args=None):
    parser = argparse.ArgumentParser()

    parser.add_argument("--reset-start", action="store_true")
    parser.add_argument("--pose", action="store_true")
    parser.add_argument("--imu-status", action="store_true")
    parser.add_argument("--zero-imu-here", action="store_true")
    parser.add_argument("--set-pose", nargs=3, type=float, metavar=("X", "Y", "THETA"))

    parser.add_argument("--route", nargs=2, metavar=("ORIGIN", "DESTINATION"))
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--yes", action="store_true")

    parser.add_argument("--slow-pick-enter", action="store_true")
    parser.add_argument("--slow-pick-roi", action="store_true")
    parser.add_argument("--slow-pick-speed", type=float, default=DEFAULT_SLOW_PICK_SPEED)
    parser.add_argument("--slow-pick-timeout", type=float, default=12.0)
    parser.add_argument("--slow-pick-min-time", type=float, default=0.4)

    parser.add_argument("--linear-speed", type=float)
    parser.add_argument("--angular-speed", type=float)

    parsed_args = parser.parse_args()

    rclpy.init(args=args)
    node = BaseRobot()
    node.auto_confirm = parsed_args.yes

    try:
        if parsed_args.linear_speed is not None:
            node.linear_speed = float(parsed_args.linear_speed)

        if parsed_args.angular_speed is not None:
            node.angular_speed = float(parsed_args.angular_speed)

        node.slow_pick_speed = float(parsed_args.slow_pick_speed)

        if parsed_args.reset_start:
            node.reset_to_start()

        elif parsed_args.zero_imu_here:
            node.zero_imu_here_as_theta_zero()
            node.print_imu_status()

        elif parsed_args.pose:
            if node.imu_config.get("use_imu_turning", True):
                if node.wait_for_imu(timeout=1.0):
                    node.sync_theta_from_imu()

            node.print_pose()

        elif parsed_args.imu_status:
            print("\nEsperando /imu_broadcaster/imu...")

            ok = node.wait_for_imu(timeout=5.0)

            if not ok:
                print("\nERROR: No llega /imu_broadcaster/imu.")
                print("Comprueba:")
                print("  ros2 topic info /imu_broadcaster/imu")
                print("  ros2 control list_controllers")
                print("  ros2 topic echo /imu_broadcaster/imu --once")

            node.print_imu_status()

        elif parsed_args.set_pose:
            x, y, theta = parsed_args.set_pose
            node.set_pose(x, y, theta)

        elif parsed_args.route:
            origin, destination = parsed_args.route
            ok = node.execute_route(origin, destination, dry_run=parsed_args.dry_run)

            if not ok:
                sys.exit(1)

        elif parsed_args.slow_pick_enter:
            node.slow_pick_enter(speed=parsed_args.slow_pick_speed)

        elif parsed_args.slow_pick_roi:
            ok = node.slow_pick_roi(
                speed=parsed_args.slow_pick_speed,
                timeout=parsed_args.slow_pick_timeout,
                min_time=parsed_args.slow_pick_min_time
            )

            if not ok:
                sys.exit(1)

        else:
            parser.print_help()

    except KeyboardInterrupt:
        print("\nInterrumpido por usuario.")
        node.stop_robot()

    finally:
        node.close()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()