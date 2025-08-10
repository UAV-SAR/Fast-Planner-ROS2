/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rviz_plugins/goal_tool.h"

// Required for tf2::Quaternion and its conversion to geometry_msgs
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // Note the .hpp extension

namespace rviz_plugins
{

Goal3DTool::Goal3DTool()
{
  shortcut_key_ = 'g';

  topic_property_ = new rviz_common::properties::StringProperty( "Topic", "goal",
                                        "The topic on which to publish navigation goals.",
                                        getPropertyContainer(), SLOT( updateTopic() ), this );
}

void Goal3DTool::onInitialize()
{
  Pose3DTool::onInitialize();
  // We need to get the ROS node from the display context to create a publisher and get the clock.
  nh_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  setName( "3D Nav Goal" );
  updateTopic();
}

void Goal3DTool::updateTopic()
{
  // Ensure the node handle is valid before creating a publisher
  if (nh_) {
    pub_ = nh_->create_publisher<geometry_msgs::msg::PoseStamped>( topic_property_->getStdString(), 1 );
  }
}

void Goal3DTool::onPoseSet(double x, double y, double z, double theta)
{
  // Ensure the node handle and publisher are valid
  if (!nh_ || !pub_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Goal3DTool"),
      "Cannot publish goal, the ROS node or publisher is not properly initialized.");
    return;
  }

  RCLCPP_INFO(nh_->get_logger(), "3D Nav Goal set.");

  // Create the PoseStamped message
  auto goal = std::make_unique<geometry_msgs::msg::PoseStamped>();

  // 1. Set the header
  goal->header.frame_id = context_->getFixedFrame().toStdString();
  goal->header.stamp = nh_->get_clock()->now();

  // 2. Set the position from the function arguments
  goal->pose.position.x = x;
  goal->pose.position.y = y;
  goal->pose.position.z = z;

  // 3. Create a tf2::Quaternion, set its value from yaw, and convert to a geometry_msgs::msg::Quaternion
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta); // Roll, Pitch, Yaw
  goal->pose.orientation = tf2::toMsg(quat);

  // Log the goal being published
  RCLCPP_INFO(nh_->get_logger(), "Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f)",
      goal->header.frame_id.c_str(),
      goal->pose.position.x, goal->pose.position.y, goal->pose.position.z,
      goal->pose.orientation.x, goal->pose.orientation.y, goal->pose.orientation.z, goal->pose.orientation.w);
  
  // Publish the message
  pub_->publish(std::move(goal));
}

} // end namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::Goal3DTool, rviz_common::Tool)