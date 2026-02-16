#!/usr/bin/env python3
import sys
import math
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

LIMIT = 1.57  # rad

HELP = """
Teleop keys (press, no Enter):
  Joint 1 (base):      q / a
  Joint 2 (shoulder):  w / s
  Joint 3 (elbow):     e / d
  Joint 4 (wrist):     r / f
  Joint 5 (end):       t / g
  Joint 6 (gripper):   y / h

  Step size:           [  decrease   ] increase
  Reset all:           0
  Quit:                x
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('mini_arm_keyboard_teleop')
        self.pub = self.create_publisher(Float64MultiArray, '/arm_forward_controller/commands', 10)
        self.q = [0.0] * 6
        self.step = 0.05
        self.get_logger().info(HELP)

    def clamp(self, v: float) -> float:
        return max(-LIMIT, min(LIMIT, v))

    def publish(self):
        msg = Float64MultiArray()
        msg.data = list(self.q)
        self.pub.publish(msg)

    def bump(self, idx: int, delta: float):
        self.q[idx] = self.clamp(self.q[idx] + delta)
        self.publish()

def get_key(timeout=0.1):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        return sys.stdin.read(1)
    return None

def main():
    rclpy.init()
    node = KeyboardTeleop()

    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        node.get_logger().info("Focus this terminal and press keys (no Enter).")
        node.publish()  # send initial zeros

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            k = get_key(0.1)
            if not k:
                continue

            if k == 'x':
                node.get_logger().info("Exiting teleop.")
                break

            if k == '0':
                node.q = [0.0] * 6
                node.publish()
                node.get_logger().info("Reset all joints to 0")
                continue

            if k == '[':
                node.step = max(0.01, node.step - 0.01)
                node.get_logger().info(f"step = {node.step:.2f} rad")
                continue
            if k == ']':
                node.step = min(0.50, node.step + 0.01)
                node.get_logger().info(f"step = {node.step:.2f} rad")
                continue

            # Joint mapping
            keys = {
                'q': (0, +1), 'a': (0, -1),
                'w': (1, +1), 's': (1, -1),
                'e': (2, +1), 'd': (2, -1),
                'r': (3, +1), 'f': (3, -1),
                't': (4, +1), 'g': (4, -1),
                'y': (5, +1), 'h': (5, -1),
            }

            if k in keys:
                idx, sign = keys[k]
                node.bump(idx, sign * node.step)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

