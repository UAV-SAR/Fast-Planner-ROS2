import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from std_srvs.srv import Trigger
from util_interfaces.srv import SetFrameAsGoal

def pose_from_transform_stamped(ts) -> PoseStamped:
        ps = PoseStamped()
        ps.header = ts.header
        ps.pose.position.x = ts.transform.translation.x
        ps.pose.position.y = ts.transform.translation.y
        ps.pose.position.z = ts.transform.translation.z
        ps.pose.orientation = ts.transform.rotation
        return ps

class GoalServer(Node):
    def __init__(self):
        super().__init__('goal_server')

        self.latest_poses_ = {}

        self.declare_parameter('input_topic')
        self.declare_parameter('output_topic')

        self.input_topic_ = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic_ = self.get_parameter('output_topic').get_parameter_value().string_value

        if self.input_topic_ is None or self.output_topic_ is None:
            self.get_logger().error('One of the required parameters: \'input_topic\', \'output_topic\' is missing.')
            rclpy.shutdown()
            return

        self.sub_ = self.create_subscription(TFMessage, self.input_topic_, self.tf_callback, 10)
        self.pub_ = self.create_publisher(PoseStamped, self.output_topic_, 1)
        self.srv_ = self.create_service(SetFrameAsGoal, '/send_goal', self.send_goal_callback)

        self.get_logger().info(f"Goal server started.")
        self.get_logger().info("Call the '/send_goal' service to publish a goal.")

    def tf_callback(self, msg: TFMessage):
        for ts in msg.transforms:
                    self.latest_poses_[ts.child_frame_id] = pose_from_transform_stamped(ts)

    def send_goal_callback(self, request, response):
        target_frame = request.frame_id
        self.get_logger().info(f"Received request to set goal for frame: '{target_frame}'")

        if target_frame in self.latest_poses_:
            goal_pose = self.latest_poses_[target_frame]
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            self.pub_.publish(goal_pose)

            self.get_logger().info(f"Successfully published goal for '{target_frame}'.")
            response.success = True
            response.message = "Goal published."
        else:
            self.get_logger().warn(f"Could not send goal because pose for '{target_frame}' has not been received yet.")
            response.success = False
            response.message = "Target frame pose not available."
        
        return response
    
def main():
    rclpy.init()
    node = GoalServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()