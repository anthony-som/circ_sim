#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String


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

        elif not has_contact and self.button_pressed[button_id]:
            self.button_pressed[button_id] = False
            self.get_logger().info(f'Button_{button_id} released')


def main(args=None):
    rclpy.init(args=args)
    node = ButtonMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
