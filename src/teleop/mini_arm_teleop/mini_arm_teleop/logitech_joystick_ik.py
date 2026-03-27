#!/usr/bin/env python3
import os
from typing import List

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy

from ament_index_python.packages import get_package_share_directory
from ikpy.chain import Chain


HELP = """
Logitech Extreme 3D Pro Cartesian IK teleop (ikpy)

Publishes: /arm_forward_controller/commands  (Float64MultiArray, 6 joints)

Joystick mapping:
  axis 0: left/right   (left=-1, right=+1) -> Y
  axis 1: forward/back (forward=-1, back=+1) -> X (inverted so forward gives +X)
  axis 2: twist        (CCW=-1, CW=+1) -> Z

Speed knob:
  axis 3: base knob
    +1.0 (most positive) -> speed_min
    -1.0 (most negative) -> speed_max
    linear in between

Buttons:
  btn 2: reset target
  btn 9: quit

Notes:
- Target is a 3D xyz point in base_link frame
- IK uses position-only solve: Chain.inverse_kinematics([x,y,z], initial_position=guess)
- Output is 6 joint positions (rad) to ros2_control forward controller
"""


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def clamp_vec3(v: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    return np.minimum(np.maximum(v, lo), hi)


class LogitechCartesianIKTeleop(Node):
    def __init__(self):
        super().__init__("arm_logitech_cartesian_ik_teleop")

        self.pub = self.create_publisher(Float64MultiArray, "/arm_forward_controller/commands", 10)
        self.create_subscription(Joy, "/joy", self.on_joy, 20)

        pkg = get_package_share_directory("arm_description")
        urdf_path = os.path.join(pkg, "urdf", "mini_arm.urdf")
        self.get_logger().info(f"Loading URDF: {urdf_path}")
        self.chain = Chain.from_urdf_file(urdf_path)

        self.joint_names: List[str] = [
            "base_rotator_joint",
            "shoulder_joint",
            "elbow_joint",
            "wrist_joint",
            "end_joint",
            "gear_right_joint",
        ]

        active_mask = [False] * len(self.chain.links)
        for jn in self.joint_names:
            for i, link in enumerate(self.chain.links):
                if link.name == jn:
                    active_mask[i] = True
                    break
        self.chain.active_links_mask = active_mask

        self.target = np.array([0.00, 0.00, 0.00], dtype=float)
        self.xyz_min = np.array([-0.50, -0.50, 0.00], dtype=float)
        self.xyz_max = np.array([ 0.50,  0.50, 0.50], dtype=float)

        self.q = np.zeros(6, dtype=float)

        self.sign = np.array([1, 1, 1, 1, 1, 1], dtype=float)
        self.offset = np.array([0, 0, 0, 0, 0, 0], dtype=float)

        self.limits_lo = np.array([-1.57] * 6, dtype=float)
        self.limits_hi = np.array([ 1.57] * 6, dtype=float)

        self.axis_lr = int(self.declare_parameter("axis_lr", 2).value)
        self.axis_fb = int(self.declare_parameter("axis_fb", 1).value)
        self.axis_tw = int(self.declare_parameter("axis_tw", 5).value)
        self.axis_speed = int(self.declare_parameter("axis_speed", 3).value)

        self.btn_reset = int(self.declare_parameter("btn_reset", 2).value)
        self.btn_quit = int(self.declare_parameter("btn_quit", 9).value)

        self.rate_hz = float(self.declare_parameter("rate_hz", 50.0).value)
        self.deadband = float(self.declare_parameter("deadband", 0.08).value)

        # Speed range; actual speed is continuously set by axis_speed
        self.speed_min = float(self.declare_parameter("speed_min", 0.01).value)
        self.speed_max = float(self.declare_parameter("speed_max", 0.25).value)
        self.speed = float(self.declare_parameter("speed", 0.08).value)  # initial before first /joy

        self.axes = []
        self.buttons = []
        self.have_joy = False

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.get_logger().info(HELP)
        self.print_state()

        try:
            q6 = self.solve_ik()
            self.q = np.array(q6, dtype=float)
            self.publish(self.q)
        except Exception as e:
            self.get_logger().warn(f"Initial IK failed: {e}")

    def on_joy(self, msg: Joy):
        self.axes = list(msg.axes)
        self.buttons = list(msg.buttons)
        self.have_joy = True

    def _axis_raw(self, idx: int) -> float:
        if idx < 0 or idx >= len(self.axes):
            return 0.0
        return float(self.axes[idx])

    def _axis(self, idx: int) -> float:
        v = self._axis_raw(idx)
        if abs(v) < self.deadband:
            return 0.0
        return v

    def _btn(self, idx: int) -> int:
        if idx < 0 or idx >= len(self.buttons):
            return 0
        return int(self.buttons[idx])

    def print_state(self):
        self.get_logger().info(
            f"target xyz: [{self.target[0]:.3f}, {self.target[1]:.3f}, {self.target[2]:.3f}]  speed: {self.speed:.3f} m/s"
        )

    def publish(self, q6):
        msg = Float64MultiArray()
        msg.data = [float(x) for x in q6]
        self.pub.publish(msg)

    def solve_ik(self):
        target_xyz = np.array(self.target, dtype=float).reshape(3,)

        guess = np.zeros(len(self.chain.links), dtype=float)
        for j, jn in enumerate(self.joint_names):
            for i, link in enumerate(self.chain.links):
                if link.name == jn:
                    guess[i] = float(self.q[j])
                    break

        sol = self.chain.inverse_kinematics(target_xyz, initial_position=guess)

        q6 = np.zeros(6, dtype=float)
        for j, jn in enumerate(self.joint_names):
            for i, link in enumerate(self.chain.links):
                if link.name == jn:
                    q6[j] = float(sol[i])
                    break
            else:
                q6[j] = float(self.q[j])

        q6 = self.sign * q6 + self.offset

        for i in range(6):
            q6[i] = clamp(float(q6[i]), float(self.limits_lo[i]), float(self.limits_hi[i]))

        return q6

    def update_speed_from_knob(self):
        # axis_speed ranges [-1, +1]
        # +1 -> speed_min, -1 -> speed_max
        k = clamp(self._axis_raw(self.axis_speed), -1.0, 1.0)

        # Map to t in [0,1] where:
        #   k = +1 => t=0
        #   k = -1 => t=1
        t = (1.0 - k) * 0.5
        self.speed = self.speed_min + t * (self.speed_max - self.speed_min)

    def on_timer(self):
        if not self.have_joy:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if self._btn(self.btn_quit):
            self.get_logger().info("Exiting teleop.")
            rclpy.shutdown()
            return

        if self._btn(self.btn_reset):
            self.target = np.array([0.00, 0.00, 0.00], dtype=float)

        # Continuous speed control from knob (axis 3)
        prev_speed = self.speed
        self.update_speed_from_knob()
        speed_changed = abs(self.speed - prev_speed) > 1e-6

        lr = self._axis(self.axis_lr)  # left=-, right=+
        fb = self._axis(self.axis_fb)  # forward=-, back=+
        tw = self._axis(self.axis_tw)  # CCW=-, CW=+

        # Desired mapping:
        # axis 1 forward is negative -> +X, so invert fb
        dX = (fb) * self.speed * dt
        dY = (lr) * self.speed * dt
        dZ = (tw) * self.speed * dt

        moved = (abs(dX) + abs(dY) + abs(dZ)) > 0.0
        if not moved and not (self._btn(self.btn_reset) or speed_changed):
            return

        self.target[0] += dX
        self.target[1] += dY
        self.target[2] += dZ
        self.target = clamp_vec3(self.target, self.xyz_min, self.xyz_max)

        try:
            q6 = self.solve_ik()
            self.q = np.array(q6, dtype=float)
            self.publish(self.q)
        except Exception as e:
            self.get_logger().error(f"IK failed: {e}")
            return

        if moved or speed_changed:
            self.print_state()


def main():
    rclpy.init()
    node = LogitechCartesianIKTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

