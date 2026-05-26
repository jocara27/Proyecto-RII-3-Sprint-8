#!/usr/bin/env python3

import os
import signal
import subprocess
import time


PROCESSES = []


def start_process(name, command):
    print(f"\n========== ARRANCANDO {name} ==========")
    print(command)

    process = subprocess.Popen(
        ["bash", "-lc", command],
        preexec_fn=os.setsid
    )

    PROCESSES.append((name, process))
    time.sleep(2.0)


def stop_processes():
    print("\n\n========== PARANDO TODO COMO CTRL+C ==========")

    # Primero servidores auxiliares.
    ordered = list(reversed(PROCESSES))

    for name, process in ordered:
        if process.poll() is None:
            print(f"SIGINT a {name} PID {process.pid}")
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
            except ProcessLookupError:
                pass

    print("\nEsperando cierre limpio...")
    time.sleep(8.0)

    for name, process in ordered:
        if process.poll() is None:
            print(f"SIGTERM a {name} PID {process.pid}")
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass

    print("\nEsperando cierre final...")
    time.sleep(3.0)

    for name, process in ordered:
        if process.poll() is None:
            print(f"SIGKILL a {name} PID {process.pid}")
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass

    print("\nTodo parado.")


def main():
    try:
        start_process(
            "BASE / DIFF_CAR",
            """
            cd ~/robot_ws
            source /opt/ros/humble/setup.bash
            source install/setup.bash
            ros2 launch diff_car diff_car.launch.py
            """
        )

        start_process(
            "BRAZO",
            """
            cd ~/brazo_ws
            source /opt/ros/humble/setup.bash
            source install/setup.bash
            ros2 launch brazo_pkg bringup_arm.launch.py
            """
        )

        start_process(
            "CAMARA ASTRA",
            """
            cd ~/yahboomcar_ros2_ws/software/library_ws
            source /opt/ros/humble/setup.bash
            source install/setup.bash
            ros2 launch astra_camera astro_pro_plus.launch.xml
            """
        )

        start_process(
            "WEB VIDEO SERVER",
            """
            source /opt/ros/humble/setup.bash
            source ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
            ros2 run web_video_server web_video_server
            """
        )

        start_process(
            "CAMERA ROI",
            """
            cd ~/robot_ws/src/diff_car/scripts
            source /opt/ros/humble/setup.bash
            source ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
            python3 camera_roi.py
            """
        )

        start_process(
            "ARM SERVER",
            """
            cd ~/robot_ws/src/diff_car/scripts
            source /opt/ros/humble/setup.bash
            source ~/brazo_ws/install/setup.bash
            python3 arm_server.py
            """
        )

        start_process(
            "CONTROL PANEL",
            """
            cd ~/robot_ws/src/diff_car/scripts
            source /opt/ros/humble/setup.bash
            source ~/robot_ws/install/setup.bash
            source ~/brazo_ws/install/setup.bash
            source ~/yahboomcar_ros2_ws/software/library_ws/install/setup.bash
            python3 control_panel.py
            """
        )

        print("\n\n==============================================")
        print("TODO ARRANCADO")
        print("Abre en el PC: http://192.168.111.122:5000")
        print("Para parar TODO: Ctrl+C en ESTA terminal")
        print("==============================================\n")

        while True:
            time.sleep(1.0)

    except KeyboardInterrupt:
        stop_processes()


if __name__ == "__main__":
    main()