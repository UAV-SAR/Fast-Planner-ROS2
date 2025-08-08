#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster
from rclpy.duration import Duration


class OdometryConverter:
    def __init__(self, node, index, params, tf_broadcaster):
        self.node = node
        self.tf_broadcaster = tf_broadcaster

        self.frame_id_in = params['frame_id_in']
        self.frame_id_out = params['frame_id_out']
        self.broadcast_tf = params['broadcast_tf']
        self.body_frame_id = params['body_frame_id']
        self.intermediate_frame_id = params['intermediate_frame_id']
        self.world_frame_id = params['world_frame_id']

        self.tf_pub_flag = True
        self.path = []

        prefix = f"/converter{index}/"
        self.out_odom_pub = node.create_publisher(Odometry, f"{prefix}out_odom", 10)
        self.out_path_pub = node.create_publisher(Path, f"{prefix}out_path", 10)

        self.in_odom_sub = node.create_subscription(
            Odometry, f"{prefix}in_odom", self.in_odom_callback, 10)

        self.tf_pub_timer = node.create_timer(0.1, self.tf_pub_callback)
        self.path_pub_timer = node.create_timer(0.5, self.path_pub_callback)

        if self.broadcast_tf:
            node.get_logger().info(f'TF enabled: {self.frame_id_in} -> {self.frame_id_out}, frames: {self.body_frame_id}, {self.intermediate_frame_id}, {self.world_frame_id}')
        else:
            node.get_logger().info(f'TF disabled: {self.frame_id_in} -> {self.frame_id_out}')

    def in_odom_callback(self, in_odom_msg):
        q = np.array([in_odom_msg.pose.pose.orientation.x,
                      in_odom_msg.pose.pose.orientation.y,
                      in_odom_msg.pose.pose.orientation.z,
                      in_odom_msg.pose.pose.orientation.w])
        p = np.array([in_odom_msg.pose.pose.position.x,
                      in_odom_msg.pose.pose.position.y,
                      in_odom_msg.pose.pose.position.z])

        e = euler_from_quaternion(q, 'rzyx')
        wqb = quaternion_from_euler(e[0], e[1], e[2], 'rzyx')
        wqc = quaternion_from_euler(e[0], 0.0, 0.0, 'rzyx')

        # Republish odometry with new frame_id
        odom_msg = in_odom_msg
        assert in_odom_msg.header.frame_id == self.frame_id_in
        odom_msg.header.frame_id = self.frame_id_out
        odom_msg.child_frame_id = ""
        self.out_odom_pub.publish(odom_msg)

        # Publish transforms if enabled
        if self.broadcast_tf and self.tf_pub_flag:
            self.tf_pub_flag = False

            now = odom_msg.header.stamp

            if self.frame_id_in != self.frame_id_out:
                self._send_transform([0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                                     now, self.frame_id_in, self.frame_id_out)

            if self.world_frame_id != self.frame_id_out:
                self._send_transform([0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                                     now, self.world_frame_id, self.frame_id_out)

            self._send_transform(p, wqb, now, self.body_frame_id, self.world_frame_id)
            self._send_transform(p, wqc, now, self.intermediate_frame_id, self.world_frame_id)

        # Store path
        pose = PoseStamped()
        pose.header = odom_msg.header
        pose.pose.position.x = p[0]
        pose.pose.position.y = p[1]
        pose.pose.position.z = p[2]
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        self.path.append(pose)

    def _send_transform(self, trans, rot_euler, stamp, child_frame_id, parent_frame_id):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent_frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = trans[0]
        t.transform.translation.y = trans[1]
        t.transform.translation.z = trans[2]

        q = quaternion_from_euler(*rot_euler, axes='rzyx')
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def tf_pub_callback(self):
        self.tf_pub_flag = True

    def path_pub_callback(self):
        if self.path:
            path = Path()
            path.header = self.path[-1].header
            path.poses = self.path[-30000:]
            self.out_path_pub.publish(path)


class TfAssistNode(Node):
    def __init__(self):
        super().__init__('tf_assist')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.converters = []
        self.load_converters()

    def load_converters(self):
        index = 0
        while True:
            prefix = f"converter{index}."
            try:
                frame_id_in = self.declare_parameter(prefix + "frame_id_in").value
                frame_id_out = self.declare_parameter(prefix + "frame_id_out").value
                broadcast_tf = self.declare_parameter(prefix + "broadcast_tf", False).value
                body_frame_id = self.declare_parameter(prefix + "body_frame_id", "body").value
                intermediate_frame_id = self.declare_parameter(prefix + "intermediate_frame_id", "intermediate").value
                world_frame_id = self.declare_parameter(prefix + "world_frame_id", "world").value

                params = {
                    "frame_id_in": frame_id_in,
                    "frame_id_out": frame_id_out,
                    "broadcast_tf": broadcast_tf,
                    "body_frame_id": body_frame_id,
                    "intermediate_frame_id": intermediate_frame_id,
                    "world_frame_id": world_frame_id
                }

                converter = OdometryConverter(self, index, params, self.tf_broadcaster)
                self.converters.append(converter)
                index += 1
            except rclpy.exceptions.ParameterNotDeclaredException:
                if index == 0:
                    raise RuntimeError(f"No parameters found for converter0.")
                else:
                    self.get_logger().info(f"Loaded {index} converter(s).")
                    break


def main(args=None):
    rclpy.init(args=args)
    node = TfAssistNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
