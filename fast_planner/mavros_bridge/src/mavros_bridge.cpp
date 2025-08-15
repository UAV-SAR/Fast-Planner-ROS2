// File: so3_to_mavros_bridge.cpp
// Purpose: Bridge quadrotor_msgs/msg/SO3Command to MAVROS AttitudeTarget (PX4/ArduPilot)
// ROS 2 Humble, C++
// Motion-only: publishes attitude + thrust. No arming/mode switching.
// Improvements: fresh timestamps, quaternion validation, re-publish timer,
// thrust shaping (bounds + optional low-pass), required critical parameters.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/header.hpp>

#include <mavros_msgs/msg/attitude_target.hpp>

#include <quadrotor_msgs/msg/so3_command.hpp>
#include <quadrotor_msgs/msg/aux_command.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <chrono>

using std::placeholders::_1;

class SO3ToMavrosBridge : public rclcpp::Node {
public:
  SO3ToMavrosBridge() : Node("so3_to_mavros_bridge") {
    this->declare_parameter<double>("vehicle_mass");
    this->declare_parameter<double>("hover_thrust");
    this->gravity_ = this->declare_parameter<double>("gravity", 9.80665);
    this->clamp_thrust_ = this->declare_parameter<bool>("clamp_thrust", true);
    this->max_thrust_ = this->declare_parameter<double>("max_thrust", 0.95);
    this->min_thrust_ = this->declare_parameter<double>("min_thrust", 0.05);
    this->max_kf_correction_ = this->declare_parameter<double>("max_kf_correction", 0.2);
    this->thrust_lpf_alpha_  = this->declare_parameter<double>("thrust_lpf_alpha", 1.0);
    this->setpoint_topic_ = this->declare_parameter<std::string>("attitude_target_topic", "/mavros/setpoint_raw/attitude");
    this->publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);

    if (!this->get_parameter("vehicle_mass", this->mass_) || !this->get_parameter("hover_thrust", this->hover_thrust_)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to get required parameters: vehicle_mass or hover_thrust");
      throw std::runtime_error("Missing required parameters");
    }

    if (this->min_thrust_ >= this->max_thrust_) {
      RCLCPP_FATAL(this->get_logger(), "min_thrust (%.3f) must be < max_thrust (%.3f)", this->min_thrust_, this->max_thrust_);
      throw std::runtime_error("Invalid thrust clamp range");
    }

    this->att_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>(this->setpoint_topic_, 50);
    this->so3_sub_ = this->create_subscription<quadrotor_msgs::msg::SO3Command>("so3_cmd", 50, std::bind(&SO3ToMavrosBridge::so3Callback, this, _1));

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, this->publish_rate_hz_));
    this->timer_ = this->create_wall_timer(period, std::bind(&SO3ToMavrosBridge::timerPublish, this));

    RCLCPP_INFO(this->get_logger(), "SO3 → MAVROS bridge started. mass=%.3f kg, hover_thrust=%.2f, rate=%.1f Hz", this->mass_, this->hover_thrust_, this->publish_rate_hz_);
  }

private:
  static bool finite3(const tf2::Vector3 &v) {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
  }

  void so3Callback(const quadrotor_msgs::msg::SO3Command::SharedPtr msg) {
    const double mg = std::max(this->mass_ * this->gravity_, 1e-6);
    if (this->mass_ <= 0.0 || this->gravity_ <= 0.0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Invalid mass/gravity (m=%.3f, g=%.3f). Using mg clamp to avoid division by zero.", this->mass_, this->gravity_);
    }

    tf2::Quaternion q_wb;
    tf2::fromMsg(msg->orientation, q_wb);
    const double n = q_wb.length();
    if (!std::isfinite(n) || n < 1e-6) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Received invalid quaternion; using identity.");
      q_wb.setValue(0, 0, 0, 1);
    } else {
      q_wb.normalize();
    }
    tf2::Matrix3x3 R_wb(q_wb);

    const tf2::Vector3 z_b_world = R_wb * tf2::Vector3(0.0, 0.0, 1.0);

    const tf2::Vector3 F_world(msg->force.x, msg->force.y, msg->force.z);
    if (!finite3(F_world)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Non-finite force received; skipping publish.");
      return;
    }

    const double u1 = F_world.dot(z_b_world);

    double kf_corr = msg->aux.kf_correction;
    kf_corr = std::clamp(kf_corr, -std::abs(max_kf_correction_), std::abs(max_kf_correction_));

    double thrust = (u1 / mg) * hover_thrust_;
    thrust *= (1.0 + kf_corr);

    if (clamp_thrust_) {
      thrust = std::min(std::max(thrust, min_thrust_), max_thrust_);
    }
    thrust = std::clamp(thrust, 0.0, 1.0);

    if (this->thrust_lpf_alpha_ > 0.0 && this->thrust_lpf_alpha_ < 1.0 && this->have_last_att_) {
      const double prev = this->last_att_.thrust;
      thrust = this->thrust_lpf_alpha_ * thrust + (1.0 - this->thrust_lpf_alpha_) * prev;
    }

    mavros_msgs::msg::AttitudeTarget att;
    att.header.stamp = this->now();
    att.header.frame_id = msg->header.frame_id;
    att.orientation = tf2::toMsg(q_wb);
    att.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
                    mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
                    mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;
    att.thrust = static_cast<float>(thrust);

    this->att_pub_->publish(att);
    this->last_att_ = att;
    this->have_last_att_ = true;
  }

  void timerPublish() {
    if (!this->have_last_att_) return;
    this->last_att_.header.stamp = this->now();
    this->att_pub_->publish(this->last_att_);
  }

  double mass_, gravity_, hover_thrust_, max_thrust_, min_thrust_, max_kf_correction_, thrust_lpf_alpha_, publish_rate_hz_;
  bool clamp_thrust_;
  bool have_last_att_ = false;
  std::string setpoint_topic_;

  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr att_pub_;
  rclcpp::Subscription<quadrotor_msgs::msg::SO3Command>::SharedPtr so3_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  mavros_msgs::msg::AttitudeTarget last_att_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SO3ToMavrosBridge>());
  rclcpp::shutdown();
  return 0;
}
