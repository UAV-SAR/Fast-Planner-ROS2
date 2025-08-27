import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage

def pose_from_transform_stamped(ts) -> PoseStamped:
        ps = PoseStamped()
        ps.header = ts.header
        ps.pose.position.x = ts.transform.translation.x
        ps.pose.position.y = ts.transform.translation.y
        ps.pose.position.z = ts.transform.translation.z
        ps.pose.orientation = ts.transform.rotation
        return ps

class TargetPosePublisher(Node):
    def __init__(self):
        super().__init__('target_pose_publisher')

        self.declare_parameter('target_child_frame', 'base_link')
        self.declare_parameter('input_topic')
        self.declare_parameter('output_topic')

        self.target_child_frame_ = self.get_parameter('target_child_frame').get_parameter_value().string_value
        self.input_topic_ = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic_ = self.get_parameter('output_topic').get_parameter_value().string_value

        if self.input_topic_ is None or self.output_topic_ is None:
             self.get_logger().error('One or more of the required parameters (\'input_topic\', \'output_topic\') are missing.')

        self.sub_ = self.create_subscription(TFMessage, self.input_topic_, self.tf_callback, 10)
        self.pub_ = self.create_publisher(PoseStamped, self.output_topic_, 10)

        self.get_logger().info(f'Extracting PoseStamped for child_frame_id="{self.target_child_frame_}" from {self.input_topic_}, publishing on {self.output_topic_}')

    def tf_callback(self, msg: TFMessage):
        match_ps = None
        for ts in msg.transforms:
            if ts.child_frame_id == self.target_child_frame_:
                match_ps = pose_from_transform_stamped(ts)

            if match_ps is not None:
                 self.pub_.publish(match_ps)

def main():
     rclpy.init()
     node = TargetPosePublisher()
     try:
        rclpy.spin(node)
     finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()