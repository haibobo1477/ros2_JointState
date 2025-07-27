#!/usr/bin/env python3
import rclpy
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import tkinter as tk
from tkinter import ttk
import math
import threading
import time
import numpy as np

class ViperXControllerGUI:
    def __init__(self):
        rclpy.init()
        self.bot = InterbotixManipulatorXS(
            robot_model='vx300s',
            group_name='arm',
            gripper_name='gripper',
        )

        # ---------- Tkinter GUI ----------
        self.root = tk.Tk()
        self.root.title("ViperX-300s Joint Controller (-100° ~ +100°)")

        self.joint_names = [
            "waist", "shoulder", "elbow",
            "forearm_roll", "wrist_angle", "wrist_rotate"
        ]

        self.build_sliders()
        
        self.pose_timer_thread = threading.Thread(target=self.pose_logger_loop, daemon=True)
        self.pose_timer_thread.start()
        
    def pose_logger_loop(self):
        while True:
            try:
                rclpy.spin_once(self.bot.core.robot_node, timeout_sec=0)
                T = self.bot.arm.get_ee_pose()
                print("\n[End Effector Pose] (w.r.t Space frame):\n", np.round(T, 3))
            except Exception as e:
                print("[Error getting EE pose]:", e)
            time.sleep(0.1)

    # ---------- （ -100°  +100°） ----------
    def build_sliders(self):
        for idx, name in enumerate(self.joint_names):
            ttk.Label(self.root, text=name).grid(row=idx, column=0, padx=8, pady=4)

            slider = tk.Scale(
                self.root,
                from_=-100, to=100,               # ←  ±100°
                resolution=1,                     #  1°
                orient=tk.HORIZONTAL, length=320,
                command=lambda deg, joint=name: self.queue_joint_cmd(joint, deg)
            )
            slider.set(0)                        # default 0°
            slider.grid(row=idx, column=1)

    def queue_joint_cmd(self, joint, deg_value):
        try:
            deg = float(deg_value)               # GUI 
            rad = math.radians(deg)              
        except ValueError:
            return

        threading.Thread(
            target=self.bot.arm.set_single_joint_position,
            args=(joint, rad),
            daemon=True
        ).start()

    def run(self):
        self.root.mainloop()
        rclpy.shutdown()


def main():
    gui = ViperXControllerGUI()
    gui.run()


if __name__ == "__main__":
    main()
