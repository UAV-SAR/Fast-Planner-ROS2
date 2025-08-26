#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

class TfChildPoseRepublisher : public rclcpp::Node {
public:
  TfChildPoseRepublisher() : Node("tf_child_pose_republisher") {
    input_tf_topic_ = declare_parameter<std::string>("input_tf_topic", "/tf");
    target_child_   = declare_parameter<std::string>("target_frame", "camera_link");
    parent_fallback_= declare_parameter<std::string>("reference_frame_fallback", "world");
    output_topic_   = declare_parameter<std::string>("output_topic", "/camera/pose");
    use_now_if_zero_= declare_parameter<bool>("use_now_if_zero_stamp", true);

    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(output_topic_, 10);
    sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      input_tf_topic_, rclcpp::QoS(50),
      std::bind(&TfChildPoseRepublisher::cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Republishing child='%s' -> %s (fallback parent='%s')",
                target_child_.c_str(), output_topic_.c_str(), parent_fallback_.c_str());
  }

private:
  void cb(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    for (const auto &tf : msg->transforms) {
      if (tf.child_frame_id != target_child_) continue;

      geometry_msgs::msg::PoseStamped out;
      out.header.frame_id = tf.header.frame_id.empty() ? parent_fallback_ : tf.header.frame_id;

      if (use_now_if_zero_ && tf.header.stamp.sec == 0 && tf.header.stamp.nanosec == 0) {
        out.header.stamp = this->get_clock()->now();  // works with /clock if use_sim_time=true
      } else {
        out.header.stamp = tf.header.stamp;
      }

      out.pose.position.x = tf.transform.translation.x;
      out.pose.position.y = tf.transform.translation.y;
      out.pose.position.z = tf.transform.translation.z;
      out.pose.orientation = tf.transform.rotation;

      pub_->publish(out);
      // Only need the first match in this message
      break;
    }
  }

  std::string input_tf_topic_, target_child_, parent_fallback_, output_topic_;
  bool use_now_if_zero_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfChildPoseRepublisher>());
  rclcpp::shutdown();
  return 0;
}
