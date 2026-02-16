#!/usr/bin/env python3

from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class Ps5ArmTeleop(Node):
    """
    Subscribes: /joy, /joint_states
    Publishes:  /arm_forward_controller/commands  (Float64MultiArray, 6 joints)

    It integrates joystick axes into joint position targets (position control).
    """

    def __init__(self):
        super().__init__("ps5_arm_teleop")

        # Output topic expected by your current ros2_control setup
        self.cmd_topic = self.declare_parameter(
            "cmd_topic", "/arm_forward_controller/commands"
        ).value

        # Joint order must match your ros2_control joint_order / forward controller order (6)
        self.joint_names = self.declare_parameter(
            "joint_names",
            [
                "base_rotator_joint",
                "shoulder_joint",
                "elbow_joint",
                "wrist_joint",
                "end_joint",
                "gear_right_joint",
            ],
        ).value

        # Limits (radians). Keep aligned with URDF limits if possible.
        self.lower = self.declare_parameter(
            "lower", [-1.57, -1.57, -1.57, -1.57, -1.57, -1.57]
        ).value
        self.upper = self.declare_parameter(
            "upper", [1.57, 1.57, 1.57, 1.57, 1.57, 1.57]
        ).value

        # Rates
        self.rate_hz = float(self.declare_parameter("rate_hz", 50.0).value)

        # Deadband
        self.axis_deadband = float(self.declare_parameter("axis_deadband", 0.08).value)

        # PS5 axis mapping defaults (verify by echoing /joy)
        self.axis_base = int(self.declare_parameter("axis_base", 0).value)        # yaw
        self.axis_shoulder = int(self.declare_parameter("axis_shoulder", 1).value)
        self.axis_elbow = int(self.declare_parameter("axis_elbow", 4).value)
        self.axis_wrist = int(self.declare_parameter("axis_wrist", 3).value)
        self.axis_end = int(self.declare_parameter("axis_end", 2).value)

        # Scale (rad/s) for each axis when fully deflected
        self.scale_base = float(self.declare_parameter("scale_base", 1.2).value)
        self.scale_shoulder = float(self.declare_parameter("scale_shoulder", 1.0).value)
        self.scale_elbow = float(self.declare_parameter("scale_elbow", 1.0).value)
        self.scale_wrist = float(self.declare_parameter("scale_wrist", 1.2).value)
        self.scale_end = float(self.declare_parameter("scale_end", 1.0).value)

        # Buttons for gripper open/close (controls joint 6 by stepping)
        self.btn_grip_open = int(self.declare_parameter("btn_grip_open", 4).value)
        self.btn_grip_close = int(self.declare_parameter("btn_grip_close", 5).value)
        self.grip_step = float(self.declare_parameter("grip_step", 0.04).value)

        # Optional enable button (set to -1 to always enabled)
        self.enable_button = int(self.declare_parameter("enable_button", -1).value)

        self.pub = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)

        self.axes: List[float] = []
        self.buttons: List[int] = []

        self.joint_pos: Dict[str, float] = {}
        self.have_joint_state = False

        self.target: Optional[List[float]] = None

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.get_logger().info(f"Publishing commands to {self.cmd_topic}")
        self.get_logger().info("Waiting for /joint_states and /joy...")

    def on_joy(self, msg: Joy):
        self.axes = list(msg.axes)
        self.buttons = list(msg.buttons)

    def on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_pos[name] = float(pos)
        self.have_joint_state = True

        if self.target is None:
            self.target = [self.joint_pos.get(j, 0.0) for j in self.joint_names]
            self.publish(self.target)
            self.get_logger().info("Initialized targets from current joint states.")

    def _axis(self, idx: int) -> float:
        if idx < 0 or idx >= len(self.axes):
            return 0.0
        v = float(self.axes[idx])
        if abs(v) < self.axis_deadband:
            return 0.0
        return v

    def _btn(self, idx: int) -> int:
        if idx < 0 or idx >= len(self.buttons):
            return 0
        return int(self.buttons[idx])

    def _enabled(self) -> bool:
        if self.enable_button < 0:
            return True
        return self._btn(self.enable_button) == 1

    def publish(self, q6: List[float]):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in q6]
        self.pub.publish(msg)

    def on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if not self.have_joint_state or self.target is None:
            return
        if not self._enabled():
            return

        v_base = self._axis(self.axis_base) * self.scale_base
        v_sh = -self._axis(self.axis_shoulder) * self.scale_shoulder
        v_el = -self._axis(self.axis_elbow) * self.scale_elbow
        v_wr = self._axis(self.axis_wrist) * self.scale_wrist
        v_end = self._axis(self.axis_end) * self.scale_end

        v = [v_base, v_sh, v_el, v_wr, v_end]

        # Integrate joints 0..4
        for i in range(5):
            self.target[i] = clamp(
                self.target[i] + v[i] * dt,
                float(self.lower[i]),
                float(self.upper[i]),
            )

        # Gripper joint (index 5) stepped by buttons
        if self._btn(self.btn_grip_open):
            self.target[5] = clamp(self.target[5] + self.grip_step, float(self.lower[5]), float(self.upper[5]))
        if self._btn(self.btn_grip_close):
            self.target[5] = clamp(self.target[5] - self.grip_step, float(self.lower[5]), float(self.upper[5]))

        self.publish(self.target)


def main():
    rclpy.init()
    node = Ps5ArmTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

