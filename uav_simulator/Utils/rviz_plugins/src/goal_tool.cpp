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

    topic_property_ = new rviz_common::properties::StringProperty("Topic", "goal",
                                                                  "The topic on which to publish navigation goals.",
                                                                  getPropertyContainer(), SLOT(updateTopic()), this);
  }

  Goal3DTool::~Goal3DTool()
  {
    delete arrow_;
    for (auto arrow : arrow_array_)
    {
      delete arrow;
    }
  }

  void Goal3DTool::onInitialize()
  {
    arrow_ = new rviz_rendering::Arrow(context_->getSceneManager(), nullptr, 2.0f, 0.2f, 0.5f, 0.35f);
    arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
    arrow_->getSceneNode()->setVisible(false);
    nh_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
    setName("3D Nav Goal");
    updateTopic();
  }

  void Goal3DTool::activate()
  {
    setStatus("Click and drag mouse to set position/orientation.");
    state_ = Position;
  }

  void Goal3DTool::deactivate()
  {
    arrow_->getSceneNode()->setVisible(false);
    for (auto arrow : arrow_array_)
    {
      arrow->getSceneNode()->setVisible(false);
    }
  }

  void Goal3DTool::updateTopic()
  {
    if (nh_)
    {
      pub_ = nh_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_property_->getStdString(), 1);
    }
  }

  void Goal3DTool::onPoseSet(double x, double y, double z, double theta)
  {
    if (!nh_ || !pub_)
    {
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

  int Goal3DTool::processMouseEvent(rviz_common::ViewportMouseEvent &event)
  {
    int flags = 0;
    static double prevangle = 0.0;
    static double initz = 0.0;
    static double prevy = 0.0;
    const double z_scale = 50.0;
    const double z_interval = 0.5;

    Ogre::Quaternion orient_x =
        Ogre::Quaternion(Ogre::Radian(Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Z);

    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);

    rviz_common::ViewController *view_controller = context_->getViewManager()->getCurrent();
    Ogre::Camera *camera = view_controller ? view_controller->getCamera() : nullptr;
    if (!camera)
    {
      return flags;
    }

    Ogre::Ray ray = camera->getCameraToViewportRay(
        static_cast<float>(event.x) / camera->getViewport()->getActualWidth(),
        static_cast<float>(event.y) / camera->getViewport()->getActualHeight());

    auto result = ray.intersects(ground_plane);

    if (event.leftDown())
    {
      if (result.first)
      {
        pos_ = ray.getPoint(result.second);
        arrow_->setPosition(pos_);
        state_ = Orientation;
        flags |= Render;
      }
    }
    else if (event.type == QEvent::MouseMove && event.left())
    {
      if (state_ == Orientation)
      {
        if (result.first)
        {
          Ogre::Vector3 cur_pos = ray.getPoint(result.second);
          double angle = atan2(cur_pos.y - pos_.y, cur_pos.x - pos_.x);
          arrow_->getSceneNode()->setVisible(true);
          arrow_->setOrientation(Ogre::Quaternion(Ogre::Radian(angle), Ogre::Vector3::UNIT_Z) * orient_x);

          if (event.right())
          {
            state_ = Height;
            initz = pos_.z;
            prevy = event.y;
          }
          prevangle = angle;
          flags |= Render;
        }
      }
      else if (state_ == Height)
      {
        double dy = event.y - prevy;
        prevy = event.y;
        pos_.z -= dy / z_scale;
        arrow_->setPosition(pos_);

        for (auto arrow : arrow_array_)
        {
          delete arrow;
        }
        arrow_array_.clear();

        int cnt = static_cast<int>(ceil(fabs(initz - pos_.z) / z_interval));
        for (int k = 0; k < cnt; k++)
        {
          rviz_rendering::Arrow *arrow__;
          arrow__ = new rviz_rendering::Arrow(context_->getSceneManager(), nullptr, 0.5f, 0.1f, 0.0f, 0.1f);
          arrow__->setColor(0.0f, 1.0f, 0.0f, 1.0f);
          arrow__->getSceneNode()->setVisible(true);
          Ogre::Vector3 arr_pos = pos_;
          arr_pos.z = initz - ((initz - pos_.z > 0) ? 1 : -1) * k * z_interval;
          arrow__->setPosition(arr_pos);
          arrow__->setOrientation(
              Ogre::Quaternion(Ogre::Radian(prevangle), Ogre::Vector3::UNIT_Z) * orient_x);
          arrow_array_.push_back(arrow__);
        }
        flags |= Render;
      }
    }
    else if (event.leftUp())
    {
      // Corrected the syntax error from '| |' to '||'
      if (state_ == Orientation || state_ == Height)
      {
        for (auto arrow : arrow_array_)
        {
          delete arrow;
        }
        arrow_array_.clear();
        onPoseSet(pos_.x, pos_.y, pos_.z, prevangle);
        flags |= (Finished | Render);
      }
    }

    return flags;
  }
} // end namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::Goal3DTool, rviz_common::Tool)

#include <memory>

namespace {
  // Force instantiation of the class to emit vtable
  [[maybe_unused]]
  static auto force_goal3dtool_link = [] {
    return std::make_unique<rviz_plugins::Goal3DTool>();
  }();
}
