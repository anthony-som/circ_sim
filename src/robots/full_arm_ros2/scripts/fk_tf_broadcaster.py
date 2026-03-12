#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


def rpy_to_quat(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


class DhFkBroadcaster(Node):
    def __init__(self):
        super().__init__("dh_fk_broadcaster")

        # DH link values (your table). Units are cm here.
        self.declare_parameter("a2", 10.5)
        self.declare_parameter("a3", 10.0)
        self.declare_parameter("a4", 2.75)
        self.declare_parameter("d4", 6.0)

        # Joint mapping
        self.declare_parameter("joint_names", [
            "base_rotator_joint",
            "shoulder_joint",
            "elbow_joint",
            "wrist_joint",
            "end_joint",
        ])

        # Signs and offsets (radians)
        # You said shoulder needs flipping, so default shoulder sign = -1.
        self.declare_parameter("signs", [1.0, -1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("offsets_rad", [0.0, 0.0, 0.0, 0.0, 0.0])

        self.base_frame = "base_link"
        self.child_frame = "dh_tool0"

        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, "/dh_fk_pose", 10)
        self.sub = self.create_subscription(JointState, "/joint_states", self.cb, 10)

        self.get_logger().info("DH FK broadcaster ready. Publishing TF frame 'dh_tool0' and /dh_fk_pose.")

    def cb(self, msg: JointState):
        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}

        joint_names = self.get_parameter("joint_names").get_parameter_value().string_array_value
        signs = self.get_parameter("signs").get_parameter_value().double_array_value
        offsets = self.get_parameter("offsets_rad").get_parameter_value().double_array_value

        # Ensure all joints exist
        q = []
        for i, jn in enumerate(joint_names):
            if jn not in name_to_pos:
                return
            qi = name_to_pos[jn]
            qi = signs[i] * qi + offsets[i]
            q.append(qi)

        th0, th1, th2, th3, th4 = q

        a2 = float(self.get_parameter("a2").value)
        a3 = float(self.get_parameter("a3").value)
        a4 = float(self.get_parameter("a4").value)
        d4 = float(self.get_parameter("d4").value)

        # Use the closed form we derived:
        phi = th0 + th1
        psi = th2 + th3
        chi = th2 + th3 + th4

        cphi = math.cos(phi)
        sphi = math.sin(phi)
        c2 = math.cos(th2)
        s2 = math.sin(th2)
        cpsi = math.cos(psi)
        spsi = math.sin(psi)
        cchi = math.cos(chi)
        schi = math.sin(chi)

        # Position (cm)
        x = d4 * sphi + cphi * (a2 * c2 + a3 * cpsi + a4 * cchi)
        y = -d4 * cphi + sphi * (a2 * c2 + a3 * cpsi + a4 * cchi)
        z = a2 * s2 + a3 * spsi + a4 * schi

        # Rotation matrix from T_04
        r11 = cphi * cchi
        r12 = sphi
        r13 = cphi * schi
        r21 = sphi * cchi
        r22 = -cphi
        r23 = sphi * schi
        r31 = schi
        r32 = 0.0
        r33 = -cchi

        # Convert rotation matrix to roll,pitch,yaw (simple stable method)
        # Here we do a basic extraction, good enough for visualization.
        pitch = math.asin(-r31)
        roll = math.atan2(r32, r33)
        yaw = math.atan2(r21, r11)

        qx, qy, qz, qw = rpy_to_quat(roll, pitch, yaw)

        # Publish TF (convert cm to meters for RViz)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = x / 100.0
        t.transform.translation.y = y / 100.0
        t.transform.translation.z = z / 100.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # Publish pose topic too
        p = PoseStamped()
        p.header = t.header
        p.pose.position.x = t.transform.translation.x
        p.pose.position.y = t.transform.translation.y
        p.pose.position.z = t.transform.translation.z
        p.pose.orientation = t.transform.rotation
        self.pose_pub.publish(p)


def main():
    rclpy.init()
    node = DhFkBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
