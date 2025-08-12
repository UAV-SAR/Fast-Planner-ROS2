/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef RVIZ_GOAL_TOOL_H
#define RVIZ_GOAL_TOOL_H

#include "rviz_plugins/pose_tool.h"

#include <cmath>
#include <vector>

#include <QObject>
#include <QCursor>

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreViewport.h>
#include <OgreCamera.h>
#include <OgreSceneManager.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "rviz_common/tool.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/view_manager.hpp"

namespace rviz_plugins
{

class Goal3DTool: public rviz_common::Tool
{
Q_OBJECT
public:
  Goal3DTool();
  virtual ~Goal3DTool();
  virtual void onInitialize();
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

protected:
  virtual void onPoseSet(double x, double y, double z, double theta);
  rviz_rendering::Arrow * arrow_;
  std::vector<rviz_rendering::Arrow *> arrow_array_;

  enum State
  {
    Position,
    Orientation,
    Height
  };
  State state_;

  Ogre::Vector3 pos_;

private Q_SLOTS:
  void updateTopic();

private:
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  rviz_common::properties::StringProperty* topic_property_;
};

}

#endif


