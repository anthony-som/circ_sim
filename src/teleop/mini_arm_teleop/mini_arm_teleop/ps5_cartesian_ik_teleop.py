#!/usr/bin/env python3
import os
import math
from typing import List

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy

from ament_index_python.packages import get_package_share_directory
from ikpy.chain import Chain


HELP = """
PS5 Cartesian IK teleop (ikpy) (no MoveIt)

Publishes: /arm_forward_controller/commands  (Float64MultiArray, 6 joints)

Left stick:
  Left/Right  -> -Y / +Y
  Up/Down     -> +X / -X

Right stick:
  Up/Down     -> +Z / -Z

Step (m/s):
  L1  decrease speed
  R1  increase speed

Reset target:
  Options (Share) button

Quit:
  PS button (or set quit_button param)

Notes:
- Target is a 3D xyz point in base_link frame
- IK uses position-only solve: Chain.inverse_kinematics([x,y,z], initial_position=guess)
- Output is 6 joint positions (rad) to ros2_control forward controller
"""


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def clamp_vec3(v: np.ndarray, lo: np.ndarray, hi: np.ndarray) -> np.ndarray:
    return np.minimum(np.maximum(v, lo), hi)


class Ps5CartesianIKTeleop(Node):
    def __init__(self):
        super().__init__("mini_arm_ps5_cartesian_ik_teleop")

        # Topics
        self.pub = self.create_publisher(Float64MultiArray, "/arm_forward_controller/commands", 10)
        self.create_subscription(Joy, "/joy", self.on_joy, 20)

        # URDF + IK chain
        pkg = get_package_share_directory("mini_arm_ros2")
        urdf_path = os.path.join(pkg, "urdf", "mini_arm.urdf")
        self.get_logger().info(f"Loading URDF: {urdf_path}")
        self.chain = Chain.from_urdf_file(urdf_path)

        # Commanded joint order MUST match your controller expecting 6 values
        self.joint_names: List[str] = [
            "base_rotator_joint",
            "shoulder_joint",
            "elbow_joint",
            "wrist_joint",
            "end_joint",
            "gear_right_joint",
        ]

        # Activate only those joints inside ikpy chain
        active_mask = [False] * len(self.chain.links)
        for jn in self.joint_names:
            for i, link in enumerate(self.chain.links):
                if link.name == jn:
                    active_mask[i] = True
                    break
        self.chain.active_links_mask = active_mask

        # Target pose and safety workspace box (meters)
        self.target = np.array([0.00, 0.00, 0.00], dtype=float)

        self.xyz_min = np.array([-0.50, -0.50, 0.00], dtype=float)
        self.xyz_max = np.array([ 0.50,  0.50, 0.50], dtype=float)

        # Last commanded joints (rad), also used as IK initial guess
        self.q = np.zeros(6, dtype=float)

        # Mapping: flip directions / add offsets if needed
        self.sign = np.array([1, 1, 1, 1, 1, 1], dtype=float)
        self.offset = np.array([0, 0, 0, 0, 0, 0], dtype=float)

        # Joint limits (rad)
        self.limits_lo = np.array([-1.57] * 6, dtype=float)
        self.limits_hi = np.array([ 1.57] * 6, dtype=float)

        # PS5 mapping params (change if your /joy mapping differs)
        self.axis_lx = int(self.declare_parameter("axis_lx", 0).value)  # left stick L/R
        self.axis_ly = int(self.declare_parameter("axis_ly", 1).value)  # left stick U/D
        self.axis_ry = int(self.declare_parameter("axis_ry", 4).value)  # right stick U/D

        self.btn_l1 = int(self.declare_parameter("btn_l1", 4).value)
        self.btn_r1 = int(self.declare_parameter("btn_r1", 5).value)
        self.btn_reset = int(self.declare_parameter("btn_reset", 8).value)  # Options/Share varies
        self.btn_quit = int(self.declare_parameter("btn_quit", 12).value)   # PS button varies

        # Control rates and speed
        self.rate_hz = float(self.declare_parameter("rate_hz", 50.0).value)
        self.deadband = float(self.declare_parameter("deadband", 0.08).value)

        # "speed" is meters per second at full stick deflection
        self.speed = float(self.declare_parameter("speed", 0.08).value)
        self.speed_min = float(self.declare_parameter("speed_min", 0.01).value)
        self.speed_max = float(self.declare_parameter("speed_max", 0.25).value)
        self.speed_step = float(self.declare_parameter("speed_step", 0.01).value)

        # Stick-to-axis mapping for XYZ
        # Default:
        #   +X when pushing LY up (LY is usually +1 down, -1 up, so we invert)
        #   +Y when pushing LX right
        #   +Z when pushing RY up (invert similarly)
        self.axes = []
        self.buttons = []
        self.have_joy = False

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.get_logger().info(HELP)
        self.print_state()

        # Try initial publish
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

    def _axis(self, idx: int) -> float:
        if idx < 0 or idx >= len(self.axes):
            return 0.0
        v = float(self.axes[idx])
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
        # IKPy expects 3D target position vector (x,y,z)
        target_xyz = np.array(self.target, dtype=float).reshape(3,)

        # Initial guess vector length = number of links
        guess = np.zeros(len(self.chain.links), dtype=float)
        for j, jn in enumerate(self.joint_names):
            for i, link in enumerate(self.chain.links):
                if link.name == jn:
                    guess[i] = float(self.q[j])
                    break

        sol = self.chain.inverse_kinematics(target_xyz, initial_position=guess)

        # Extract our 6 joints back out in the correct order
        q6 = np.zeros(6, dtype=float)
        for j, jn in enumerate(self.joint_names):
            found = False
            for i, link in enumerate(self.chain.links):
                if link.name == jn:
                    q6[j] = float(sol[i])
                    found = True
                    break
            if not found:
                q6[j] = float(self.q[j])

        # Apply sign/offset mapping
        q6 = self.sign * q6 + self.offset

        # Clamp to limits
        for i in range(6):
            q6[i] = clamp(float(q6[i]), float(self.limits_lo[i]), float(self.limits_hi[i]))

        return q6

    def on_timer(self):
        if not self.have_joy:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # Quit
        if self._btn(self.btn_quit):
            self.get_logger().info("Exiting teleop.")
            rclpy.shutdown()
            return

        # Speed adjust
        if self._btn(self.btn_l1):
            self.speed = clamp(self.speed - self.speed_step, self.speed_min, self.speed_max)
        if self._btn(self.btn_r1):
            self.speed = clamp(self.speed + self.speed_step, self.speed_min, self.speed_max)

        # Reset target
        if self._btn(self.btn_reset):
            self.target = np.array([0.00, 0.00, 0.00], dtype=float)

        # Read axes (PS5 typical: LY up gives -1, down gives +1, so invert)
        lx = self._axis(self.axis_lx)         # + right
        ly = self._axis(self.axis_ly)        # + forward/up
        ry = self._axis(self.axis_ry)        # + up

        # Convert to xyz deltas
        # Left stick maps to X/Y, right stick maps to Z
        dX = ly * self.speed * dt
        dY = lx * self.speed * dt
        dZ = ry * self.speed * dt

        moved = (abs(dX) + abs(dY) + abs(dZ)) > 0.0
        if not moved and not (self._btn(self.btn_l1) or self._btn(self.btn_r1) or self._btn(self.btn_reset)):
            return

        # Update target and clamp
        self.target[0] += dX
        self.target[1] += dY
        self.target[2] += dZ
        self.target = clamp_vec3(self.target, self.xyz_min, self.xyz_max)

        # Solve and publish
        try:
            q6 = self.solve_ik()
            self.q = np.array(q6, dtype=float)
            self.publish(self.q)
        except Exception as e:
            self.get_logger().error(f"IK failed: {e}")
            return

        # Print occasionally without spamming
        if moved:
            self.print_state()


def main():
    rclpy.init()
    node = Ps5CartesianIKTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

