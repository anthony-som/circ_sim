#!/usr/bin/env python3
import os
import sys
import termios
import tty
import select
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory

from ikpy.chain import Chain


HELP = """
Cartesian IK teleop (press, no Enter):
  +X / -X :  i / k
  +Y / -Y :  j / l
  +Z / -Z :  o / p

  Step size: [ decrease   ] increase
  Reset target: 0
  Quit: x

Notes:
- Publishes joint positions (rad) to /arm_forward_controller/commands
- Target is clamped to a small safe workspace box
"""


def get_key(timeout=0.1):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        return sys.stdin.read(1)
    return None


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class CartesianIKTeleop(Node):
    def __init__(self):
        super().__init__("mini_arm_cartesian_ik_teleop")

        self.pub = self.create_publisher(Float64MultiArray, "/arm_forward_controller/commands", 10)

        pkg = get_package_share_directory("mini_arm_ros2")
        self.urdf_path = os.path.join(pkg, "urdf", "mini_arm.urdf")
        self.get_logger().info(f"Loading URDF: {self.urdf_path}")

        # Build IK chain
        self.chain = Chain.from_urdf_file(self.urdf_path)

        # Your commanded joint order (MUST match the controller expecting 6 values)
        self.joint_names = [
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

        # Target pose (meters) and step
        self.target = np.array([0.00, 0.00, 0.00], dtype=float)
        self.step = 0.01

        # Safety workspace box (meters)
        self.xyz_min = np.array([-0.5, -0.5, -0.5], dtype=float)
        self.xyz_max = np.array([0.5,  0.5, 0.5], dtype=float)

        # Last commanded joints (rad), used as IK initial guess
        self.q = np.zeros(6, dtype=float)

        # Optional joint mapping (flip directions / add offsets)
        # If your 2nd joint needs flipped, set sign[1] = -1
        self.sign = np.array([1, 1, 1, 1, 1, 1], dtype=float)
        self.offset = np.array([0, 0, 0, 0, 0, 0], dtype=float)

        # URDF limits (you used +/-1.57 everywhere)
        self.limits_lo = np.array([-1.57] * 6, dtype=float)
        self.limits_hi = np.array([ 1.57] * 6, dtype=float)

        self.get_logger().info(HELP)
        self.print_state()

    def print_state(self):
        self.get_logger().info(
            f"target xyz: [{self.target[0]:.3f}, {self.target[1]:.3f}, {self.target[2]:.3f}]  step: {self.step:.3f}"
        )

    def clamp_target(self):
        self.target = np.minimum(np.maximum(self.target, self.xyz_min), self.xyz_max)

    def publish(self, q6):
        msg = Float64MultiArray()
        msg.data = [float(x) for x in q6]
        self.pub.publish(msg)

    def solve_ik(self):
        # IMPORTANT FIX:
        # ikpy Chain.inverse_kinematics expects a 3D target position vector (x,y,z),
        # NOT a 4x4 matrix.
        target_xyz = np.array(self.target, dtype=float).reshape(3,)

        # Build initial guess vector of length = number of links
        guess = np.zeros(len(self.chain.links), dtype=float)

        # Put current 6 joint values into the guess where those joints exist in the chain
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

    def apply_key(self, k: str):
        moved = False

        if k == "x":
            self.get_logger().info("Exiting teleop.")
            rclpy.shutdown()
            return

        if k == "0":
            self.target = np.array([0.10, 0.00, 0.20], dtype=float)
            moved = True

        elif k == "[":
            self.step = max(0.002, self.step / 1.25)
            self.print_state()
            return

        elif k == "]":
            self.step = min(0.05, self.step * 1.25)
            self.print_state()
            return

        elif k == "i":
            self.target[0] += self.step
            moved = True
        elif k == "k":
            self.target[0] -= self.step
            moved = True
        elif k == "j":
            self.target[1] += self.step
            moved = True
        elif k == "l":
            self.target[1] -= self.step
            moved = True
        elif k == "o":
            self.target[2] += self.step
            moved = True
        elif k == "p":
            self.target[2] -= self.step
            moved = True

        if not moved:
            return

        self.clamp_target()
        self.print_state()

        try:
            q6 = self.solve_ik()
            self.q = np.array(q6, dtype=float)
            self.publish(self.q)
        except Exception as e:
            self.get_logger().error(f"IK failed: {e}")


def main():
    rclpy.init()
    node = CartesianIKTeleop()

    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        node.get_logger().info("Focus this terminal and press keys (no Enter).")

        # Publish initial pose
        try:
            node.q = node.solve_ik()
            node.publish(node.q)
        except Exception as e:
            node.get_logger().warn(f"Initial IK failed: {e}")

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            k = get_key(0.1)
            if not k:
                continue
            node.apply_key(k)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

