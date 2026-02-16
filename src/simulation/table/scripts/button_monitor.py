#!/usr/bin/env python3
import subprocess
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String


# Material color definitions (ambient, diffuse, specular)
GREEN = (
    'ambient: {r: 0, g: 0.5, b: 0, a: 1}, '
    'diffuse: {r: 0, g: 0.8, b: 0, a: 1}, '
    'specular: {r: 0.4, g: 0.9, b: 0.4, a: 1}'
)
RED = (
    'ambient: {r: 0.5, g: 0, b: 0, a: 1}, '
    'diffuse: {r: 0.8, g: 0, b: 0, a: 1}, '
    'specular: {r: 0.9, g: 0.4, b: 0.4, a: 1}'
)


class ButtonMonitor(Node):
    def __init__(self):
        super().__init__('button_monitor')
        self.button_pub = self.create_publisher(String, '/button_events', 10)
        self.button_pressed = {}

        for i in range(1, 7):
            topic = (
                f'/world/empty/model/ButtonPanel/model/Button_{i}'
                f'/link/link/sensor/button_contact/contact'
            )
            self.create_subscription(
                Contacts, topic,
                lambda msg, btn=i: self.contact_cb(msg, btn),
                10,
            )
            self.button_pressed[i] = False

        self.get_logger().info('Button monitor ready')

    def contact_cb(self, msg, button_id):
        has_contact = len(msg.contacts) > 0

        if has_contact and not self.button_pressed[button_id]:
            self.button_pressed[button_id] = True
            self.get_logger().info(f'Button_{button_id} pressed')
            event = String()
            event.data = f'Button_{button_id} pressed'
            self.button_pub.publish(event)
            self.set_button_color(button_id, GREEN)

        elif not has_contact and self.button_pressed[button_id]:
            self.button_pressed[button_id] = False
            self.get_logger().info(f'Button_{button_id} released')
            self.set_button_color(button_id, RED)

    def set_button_color(self, button_id, material):
        visual = f'ButtonPanel::Button_{button_id}::link::visual'
        parent = f'ButtonPanel::Button_{button_id}::link'
        req = f'name: "{visual}", parent_name: "{parent}", material: {{{material}}}'
        try:
            subprocess.Popen(
                [
                    'ign', 'service',
                    '-s', '/world/empty/visual_config',
                    '--reqtype', 'ignition.msgs.Visual',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '100',
                    '--req', req,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as e:
            self.get_logger().warn(f'Failed to set color: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ButtonMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
