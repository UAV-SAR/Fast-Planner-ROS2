#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from builtin_interfaces.msg import Time
from rclpy.time import Time as RclpyTime


class OdomSender(Node):
    def __init__(self):
        super().__init__('odom_sender')

        self.pub = self.create_publisher(Odometry, 'odom', 10)

        self.msg = Odometry()
        self.msg.header.frame_id = "world"

        q = quaternion_from_euler(0, 0, 0, axes='rzyx')

        self.msg.pose.pose.position.x = 0.0
        self.msg.pose.pose.position.y = 0.0
        self.msg.pose.pose.position.z = 0.0

        self.msg.twist.twist.linear.x = 0.0
        self.msg.twist.twist.linear.y = 0.0
        self.msg.twist.twist.linear.z = 0.0

        self.msg.pose.pose.orientation.x = q[0]
        self.msg.pose.pose.orientation.y = q[1]
        self.msg.pose.pose.orientation.z = q[2]
        self.msg.pose.pose.orientation.w = q[3]

        self.counter = 0
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now()
        past_time = now - rclpy.duration.Duration(seconds=0.2)

        self.msg.header.stamp = past_time.to_msg()

        self.pub.publish(self.msg)
        self.counter += 1
        self.get_logger().info(f"Sent {self.counter:3d} msg(s).")


def main(args=None):
    rclpy.init(args=args)
    node = OdomSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
