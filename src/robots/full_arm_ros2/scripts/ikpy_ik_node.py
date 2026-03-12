#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from ament_index_python.packages import get_package_share_directory

from ikpy.chain import Chain


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class MiniArmIkpyIK(Node):
    """
    URDF-based IK using ikpy.
    Inputs: target xyz in meters, in base_link frame.
    Output: publishes joint positions (radians) to arm_forward_controller.
    """

    def __init__(self):
        super().__init__("mini_arm_ikpy_ik")

        # Parameters you may want to tweak
        self.declare_parameter("urdf_pkg", "mini_arm_ros2")
        self.declare_parameter("urdf_relpath", "urdf/mini_arm.urdf")
        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("ee_link", "end_base")

        # Your ros2_control joint order
        self.joint_order = [
            "base_rotator_joint",
            "shoulder_joint",
            "elbow_joint",
            "wrist_joint",
            "end_joint",
            "gear_right_joint",
        ]

        # Per-joint sign and offset to map IK solution into your robot convention.
        # Start with all +1 and 0, then adjust if a joint moves opposite or has a 90-degree neutral.
        self.sign = np.array([1, 1, 1, 1, 1, 1], dtype=float)

        # If your "neutral" physical pose is servos at 90 degrees, you often need offsets.
        # Offsets are in radians added after sign flip:
        # q_cmd = sign*q_ik + offset
        self.offset = np.array([0, 0, 0, 0, 0, 0], dtype=float)

        # Safety joint limits (radians). Replace with your real limits if different.
        self.limits_lo = np.array([-1.57, -1.57, -1.57, -1.57, -1.57, -1.57], dtype=float)
        self.limits_hi = np.array([ 1.57,  1.57,  1.57,  1.57,  1.57,  1.57], dtype=float)

        self.current_q = np.zeros(6, dtype=float)
        self.have_joint_state = False

        # Load URDF
        pkg = self.get_parameter("urdf_pkg").get_parameter_value().string_value
        rel = self.get_parameter("urdf_relpath").get_parameter_value().string_value
        urdf_path = get_package_share_directory(pkg) + "/" + rel
        self.get_logger().info(f"Loading URDF for IK: {urdf_path}")

        # Build chain from URDF
        # This creates a kinematic chain following the URDF tree to the end effector link.
        self.chain = Chain.from_urdf_file(
            urdf_path,
            base_elements=[self.get_parameter("base_link").value],
            last_link_vector=None,
            active_links_mask=None,
        )
        
        active = []
        for link in self.chain.links:
            active.append(any(jn == link.name for jn in self.joint_order))
        self.chain.active_links_mask = active



        # Validate end effector link name exists in chain
        ee_link = self.get_parameter("ee_link").get_parameter_value().string_value
        self.ee_link = ee_link
        link_names = [l.name for l in self.chain.links]
        if ee_link not in link_names:
            self.get_logger().warn(
                f"End effector link '{ee_link}' not found in ikpy chain links. "
                f"Available tail links include: {link_names[-6:]}"
            )

        # ROS I/O
        self.pub = self.create_publisher(Float64MultiArray, "/arm_forward_controller/commands", 10)
        self.sub = self.create_subscription(JointState, "/joint_states", self.on_joint_state, 20)

        self.get_logger().info(
            "IK ready. Type a target like: x y z (meters). Example: 0.15 0.00 0.20"
        )

        # Simple REPL timer
        self.create_timer(0.2, self.prompt_once)
        self.waiting_for_input = False

    def on_joint_state(self, msg: JointState):
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        q = []
        for j in self.joint_order:
            if j in name_to_pos:
                q.append(float(name_to_pos[j]))
            else:
                q.append(0.0)
        self.current_q = np.array(q, dtype=float)
        self.have_joint_state = True

    def prompt_once(self):
        # Avoid spamming prompt while user is typing
        if self.waiting_for_input:
            return
        self.waiting_for_input = True
        try:
            s = input("\nTarget xyz (m): ").strip()
        except EOFError:
            self.get_logger().info("EOF, exiting.")
            rclpy.shutdown()
            return
        finally:
            self.waiting_for_input = False

        if not s:
            return
        if s.lower() in ["q", "quit", "exit"]:
            rclpy.shutdown()
            return

        parts = s.split()
        if len(parts) != 3:
            self.get_logger().warn("Please enter exactly 3 numbers: x y z in meters.")
            return

        try:
            x, y, z = [float(p) for p in parts]
        except ValueError:
            self.get_logger().warn("Invalid numbers.")
            return

        if not self.have_joint_state:
            self.get_logger().warn("No /joint_states yet. Move the arm or wait a second, then try again.")
            return

        self.solve_and_publish(x, y, z)

    def solve_and_publish(self, x, y, z):
        target = np.array([x, y, z], dtype=float)

        # Initial guess: current joints.
        # ikpy chain includes extra joints for fixed links, so we need a full chain vector length.
        # We will inject our 6 joints into the right places by matching names.
        q0 = np.zeros(len(self.chain.links), dtype=float)

        # Map current_q into chain by joint/link naming. This is best-effort.
        # Many ikpy URDF links are named after joints or links, so we do a simple heuristic.
        for i, link in enumerate(self.chain.links):
            nm = link.name
            for jn, qv in zip(self.joint_order, self.current_q):
                if jn in nm:
                    q0[i] = qv

        # Compute IK
        try:
            q_sol = self.chain.inverse_kinematics(target, initial_position=q0)
        except Exception as e:
            self.get_logger().error(f"IK failed: {e}")
            return

        # Extract 6 joint values from q_sol using name matching.
        q_cmd = np.zeros(6, dtype=float)
        for idx, jn in enumerate(self.joint_order):
            found = False
            for i, link in enumerate(self.chain.links):
                if jn in link.name:
                    q_cmd[idx] = float(q_sol[i])
                    found = True
                    break
            if not found:
                q_cmd[idx] = self.current_q[idx]

        # Apply sign and offset mapping
        q_cmd = self.sign * q_cmd + self.offset

        # Clamp to limits
        for i in range(6):
            q_cmd[i] = clamp(q_cmd[i], float(self.limits_lo[i]), float(self.limits_hi[i]))

        msg = Float64MultiArray()
        msg.data = [float(v) for v in q_cmd]
        self.pub.publish(msg)

        self.get_logger().info(f"Published joints (rad): {[round(v, 3) for v in msg.data]}")


def main():
    rclpy.init()
    node = MiniArmIkpyIK()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
