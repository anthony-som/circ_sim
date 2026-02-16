#!/usr/bin/env python3

import sys
import math
import select
import termios
import tty
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
from ikpy.chain import Chain


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class CartesianIKTeleop(Node):
    def __init__(self):
        super().__init__("mini_arm_cartesian_ik_teleop")

        # URDF
        pkg_share = get_package_share_directory("mini_arm_ros2")
        urdf_path = pkg_share + "/urdf/mini_arm.urdf"
        self.get_logger().info(f"Loading URDF: {urdf_path}")

        self.chain = Chain.from_urdf_file(urdf_path, base_elements=["base_link"])

        # Your joint order used by ros2_control forward controller
        self.joint_order = [
            "base_rotator_joint",
            "shoulder_joint",
            "elbow_joint",
            "wrist_joint",
            "end_joint",
            "gear_right_joint",
        ]

        # Map IK output to your hardware convention
        # You already found shoulder direction needs flipping, so start with that.
        self.sign = np.array([1, -1, 1, 1, 1, 1], dtype=float)
        self.offset = np.array([0, 0, 0, 0, 0, 0], dtype=float)

        # Joint limits (radians), replace if you want different
        self.limits_lo = np.array([-1.57, -1.57, -1.57, -1.57, -1.57, -1.57], dtype=float)
        self.limits_hi = np.array([ 1.57,  1.57,  1.57,  1.57,  1.57,  1.57], dtype=float)

        # Publisher to your controller
        self.cmd_pub = self.create_publisher(Float64MultiArray, "/arm_forward_controller/commands", 10)

        # Joint states
        self.current_q = np.zeros(6, dtype=float)
        self.have_js = False
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 50)

        # Pick an end effector target point (meters, base_link frame)
        # Start somewhere reachable and not too aggressive.
        self.target = np.array([0.10, 0.00, 0.20], dtype=float)

        # Workspace clamp (meters), keeps you from typing nonsense
        self.xyz_min = np.array([0.02, -0.20, 0.02], dtype=float)
        self.xyz_max = np.array([0.25,  0.20, 0.35], dtype=float)

        # Step size in meters
        self.step = 0.01

        # Terminal raw mode
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.get_logger().info(
            "Cartesian IK teleop ready\n"
            "Arrows: +/-X, +/-Y   |  W/S: +Z/-Z\n"
            "[ ]: step down/up    |  R: reset target\n"
            "ESC: quit"
        )

        # Run at fixed rate so it feels responsive
        self.timer = self.create_timer(0.05, self.loop_20hz)  # 20 Hz

        # Build a robust joint index mapping from ikpy chain link names
        self.chain_names = [l.name for l in self.chain.links]
        self.joint_to_chain_index = {}
        for j in self.joint_order:
            if j in self.chain_names:
                self.joint_to_chain_index[j] = self.chain_names.index(j)

        if len(self.joint_to_chain_index) < 3:
            self.get_logger().warn(
                "Could not directly match joint names into ikpy chain.\n"
                "If movement is wrong, paste the chain names list and we will map indices exactly."
            )

    def destroy_node(self):
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        except Exception:
            pass
        super().destroy_node()

    def on_joint_state(self, msg: JointState):
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        q = []
        for j in self.joint_order:
            q.append(float(name_to_pos.get(j, 0.0)))
        self.current_q = np.array(q, dtype=float)
        self.have_js = True

    def read_key(self):
        if select.select([sys.stdin], [], [], 0.0)[0]:
            return sys.stdin.read(1)
        return None

    def read_key_blocking(self, timeout=0.02):
        if select.select([sys.stdin], [], [], timeout)[0]:
            return sys.stdin.read(1)
        return None

    def loop_20hz(self):
        # Read keys
        k = self.read_key()
        if k is not None:
            # Handle arrow sequences first
            if k == "\x1b":
                k2 = self.read_key_blocking()
                k3 = self.read_key_blocking()
                if k2 == "[" and k3 in ("A", "B", "C", "D"):
                    if k3 == "C":   # right
                        self.target[0] += self.step
                    elif k3 == "D": # left
                        self.target[0] -= self.step
                    elif k3 == "A": # up
                        self.target[1] += self.step
                    elif k3 == "B": # down
                        self.target[1] -= self.step
                else:
                    # Bare ESC (not part of an arrow key)
                    self.get_logger().info("Exiting teleop.")
                    rclpy.shutdown()
                    return

            elif k in ("w", "W"):
                self.target[2] += self.step
            elif k in ("s", "S"):
                self.target[2] -= self.step
            elif k == "]":
                self.step = min(0.05, self.step * 1.25)
            elif k == "[":
                self.step = max(0.002, self.step / 1.25)
            elif k in ("r", "R"):
                self.target = np.array([0.10, 0.00, 0.20], dtype=float)
            else:
                # ignore other keys
                pass

            self.target = np.minimum(np.maximum(self.target, self.xyz_min), self.xyz_max)

            self.get_logger().info(
                f"target xyz: [{self.target[0]:.3f}, {self.target[1]:.3f}, {self.target[2]:.3f}]  step: {self.step:.3f}"
            )

        # Do IK and publish
        if not self.have_js:
            return

        self.solve_and_publish(self.target)

    def solve_and_publish(self, xyz):
        x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])

        # ikpy in your version wants a 3-vector target
        target = np.array([x, y, z], dtype=float)

        # Initial guess: build a full vector sized to chain
        q0 = np.zeros(len(self.chain.links), dtype=float)

        # If chain indices are known, seed them
        for i, j in enumerate(self.joint_order):
            idx = self.joint_to_chain_index.get(j, None)
            if idx is not None:
                q0[idx] = self.current_q[i]

        try:
            q_sol = self.chain.inverse_kinematics(target, initial_position=q0)
        except Exception as e:
            self.get_logger().warn(f"IK failed: {e}")
            return

        # Extract the 6 joints
        q_cmd = np.copy(self.current_q)

        for i, j in enumerate(self.joint_order):
            idx = self.joint_to_chain_index.get(j, None)
            if idx is not None:
                q_cmd[i] = float(q_sol[idx])

        # apply sign and offset
        q_cmd = self.sign * q_cmd + self.offset

        # clamp joint limits
        for i in range(6):
            q_cmd[i] = clamp(float(q_cmd[i]), float(self.limits_lo[i]), float(self.limits_hi[i]))

        msg = Float64MultiArray()
        msg.data = [float(v) for v in q_cmd]
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = CartesianIKTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
