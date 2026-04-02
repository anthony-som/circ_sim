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

        for panel_name in ('ButtonPanel', 'ServicePanel'):
            for i in range(1, 7):
                topic = (
                    f'/world/empty/model/{panel_name}/model/Button_{i}'
                    f'/link/link/sensor/button_contact/contact'
                )
                self.create_subscription(
                    Contacts, topic,
                    lambda msg, panel=panel_name, btn=i: self.contact_cb(msg, panel, btn),
                    10,
                )
                self.button_pressed[(panel_name, i)] = False

        self.get_logger().info('Button monitor ready')

    def contact_cb(self, msg, panel_name, button_id):
        has_contact = len(msg.contacts) > 0
        key = (panel_name, button_id)

        if has_contact and not self.button_pressed[key]:
            self.button_pressed[key] = True
            self.get_logger().info(f'{panel_name}/Button_{button_id} pressed')
            event = String()
            event.data = f'{panel_name}/Button_{button_id} pressed'
            self.button_pub.publish(event)

        elif not has_contact and self.button_pressed[key]:
            self.button_pressed[key] = False
            self.get_logger().info(f'{panel_name}/Button_{button_id} released')


def main(args=None):
    rclpy.init(args=args)
    node = ButtonMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
