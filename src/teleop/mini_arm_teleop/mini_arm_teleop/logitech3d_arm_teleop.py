#!/usr/bin/env python3
"""
Logitech Extreme 3D Pro — 6-DOF arm teleop (industry-standard scheme).

Hardware (Linux /joy):
  Axis 0: Stick X      (left/right,  springs to centre)
  Axis 1: Stick Y      (fwd/back,    springs to centre)
  Axis 2: Stick twist  (CW/CCW,      springs to centre)
  Axis 3: Throttle     (back=-1 → front=+1, latches)
  Axis 4: Hat X        (discrete -1/0/+1)
  Axis 5: Hat Y        (discrete -1/0/+1)

  Button 0:  Trigger   — dead-man switch (hold to move)
  Button 1:  Thumb     — mode toggle: PROXIMAL ↔ DISTAL
  Button 2:  Top-L     — home all joints (send to 0)
  Button 3+: unused

Control mapping:
  ┌──────────────┬────────────────────┬────────────────────┐
  │ Axis         │ PROXIMAL (mode 0)  │ DISTAL   (mode 1)  │
  ├──────────────┼────────────────────┼────────────────────┤
  │ Stick X      │ base_rotator_joint │ wrist_joint        │
  │ Stick Y      │ shoulder_joint     │ end_joint          │
  │ Stick twist  │ elbow_joint        │ gripper            │
  └──────────────┴────────────────────┴────────────────────┘

  Throttle → speed override  (back = 10 %, front = 100 % of max_rate)
"""

from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class Logitech3dArmTeleop(Node):

    PROXIMAL = 0  # joints: base / shoulder / elbow
    DISTAL   = 1  # joints: wrist / end / gripper

    # Fixed hardware indices — Logitech Extreme 3D Pro
    AXIS_X        = 0
    AXIS_Y        = 1
    AXIS_TWIST    = 2
    AXIS_THROTTLE = 3
    BTN_DEADMAN   = 0   # trigger
    BTN_MODE      = 1   # thumb
    BTN_HOME      = 2   # top-left

    def __init__(self):
        super().__init__("logitech3d_arm_teleop")

        self.cmd_topic: str = self.declare_parameter(
            "cmd_topic", "/arm_forward_controller/commands"
        ).value

        self.joint_names: List[str] = self.declare_parameter(
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

        self.lower: List[float] = self.declare_parameter(
            "lower", [-1.57, -1.57, -1.57, -1.57, -1.57, -1.57]
        ).value
        self.upper: List[float] = self.declare_parameter(
            "upper", [1.57, 1.57, 1.57, 1.57, 1.57, 1.57]
        ).value

        self.rate_hz:    float = float(self.declare_parameter("rate_hz", 50.0).value)
        self.deadband:   float = float(self.declare_parameter("axis_deadband", 0.08).value)
        self.max_rate:   float = float(self.declare_parameter("max_rate_rad_s", 1.5).value)

        # Runtime state
        self.axes:         List[float] = []
        self.buttons:      List[int]   = []
        self.prev_buttons: List[int]   = []
        self.joint_pos:    Dict[str, float] = {}
        self.have_joint_state = False
        self.target: Optional[List[float]] = None
        self.mode = self.PROXIMAL

        # ROS interfaces
        self.pub = self.create_publisher(Float64MultiArray, self.cmd_topic, 10)
        self.create_subscription(Joy, "/joy", self._on_joy, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.last_time = self.get_clock().now()
        self.create_timer(1.0 / self.rate_hz, self._on_timer)

        self.get_logger().info("=" * 55)
        self.get_logger().info("  Logitech 3D Pro — 6-DOF arm teleop")
        self.get_logger().info("  TRIGGER (hold)  : enable movement")
        self.get_logger().info("  THUMB           : toggle Proximal / Distal")
        self.get_logger().info("  TOP-L button    : home all joints")
        self.get_logger().info("  Throttle        : speed override (back=10% front=100%)")
        self.get_logger().info("=" * 55)
        self.get_logger().info("MODE: PROXIMAL  [base | shoulder | elbow]")
        self.get_logger().info("Waiting for /joy and /joint_states ...")

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _on_joy(self, msg: Joy):
        self.prev_buttons = list(self.buttons)
        self.axes    = list(msg.axes)
        self.buttons = list(msg.buttons)

    def _on_joint_state(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_pos[name] = float(pos)
        if not self.have_joint_state:
            self.have_joint_state = True
            self.target = [self.joint_pos.get(j, 0.0) for j in self.joint_names]
            self._publish(self.target)
            self.get_logger().info("Joint states received — ready.")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _axis(self, idx: int) -> float:
        if idx < 0 or idx >= len(self.axes):
            return 0.0
        v = float(self.axes[idx])
        return 0.0 if abs(v) < self.deadband else v

    def _btn(self, idx: int) -> int:
        if idx < 0 or idx >= len(self.buttons):
            return 0
        return int(self.buttons[idx])

    def _rising_edge(self, idx: int) -> bool:
        """True only on the first cycle a button is pressed (press event)."""
        prev = int(self.prev_buttons[idx]) if idx < len(self.prev_buttons) else 0
        return self._btn(idx) == 1 and prev == 0

    def _speed_scale(self) -> float:
        """Map throttle (latching, back=-1, front=+1) → scale [0.10, 1.00]."""
        if self.AXIS_THROTTLE >= len(self.axes):
            return 1.0
        raw = float(self.axes[self.AXIS_THROTTLE])
        return clamp((raw + 1.0) / 2.0, 0.10, 1.0)

    def _publish(self, q: List[float]):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in q]
        self.pub.publish(msg)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _on_timer(self):
        if not self.have_joint_state or self.target is None:
            return

        # Button edge events — processed regardless of dead-man state
        if self._rising_edge(self.BTN_MODE):
            self.mode = self.DISTAL if self.mode == self.PROXIMAL else self.PROXIMAL
            label = ("PROXIMAL  [base | shoulder | elbow]"
                     if self.mode == self.PROXIMAL
                     else "DISTAL    [wrist | end | gripper]")
            self.get_logger().info(f"MODE: {label}")

        if self._rising_edge(self.BTN_HOME):
            self.target = [0.0] * len(self.joint_names)
            self._publish(self.target)
            self.get_logger().info("HOME: all joints → 0 rad")
            return

        # Dead-man switch — must hold trigger to move
        if self._btn(self.BTN_DEADMAN) != 1:
            return

        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        speed = self._speed_scale() * self.max_rate

        vx = -self._axis(self.AXIS_X)     * speed
        vy = self._axis(self.AXIS_Y)      * speed
        vt = -self._axis(self.AXIS_TWIST) * speed

        if self.mode == self.PROXIMAL:
            deltas = [vx, vy, vt, 0.0, 0.0, 0.0]
        else:
            deltas = [0.0, 0.0, 0.0, vx, vy, vt]

        for i, d in enumerate(deltas):
            self.target[i] = clamp(
                self.target[i] + d * dt,
                float(self.lower[i]),
                float(self.upper[i]),
            )

        self._publish(self.target)


def main():
    rclpy.init()
    node = Logitech3dArmTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
