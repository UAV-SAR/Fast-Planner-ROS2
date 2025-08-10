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

#ifndef MULTI_PROB_MAP_DISPLAY_H
#define MULTI_PROB_MAP_DISPLAY_H

#include <OgreMaterial.h>
#include <OgreTexture.h>
#include <OgreVector3.h>
#include <OgreTechnique.h>

#include <nav_msgs/msg/map_meta_data.hpp>
#include <rclcpp/rclcpp.hpp>

#include <multi_map_msgs/msg/multi_occupancy_grid.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "rviz_common/display.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/property.hpp"

namespace Ogre
{
class ManualObject;
}

namespace rviz_plugins
{

/**
 * \class MultiProbMapDisplay
 * \brief Displays a map along the XY plane.
 */
class MultiProbMapDisplay : public rviz_common::Display
{
  Q_OBJECT
public:
  MultiProbMapDisplay();
  virtual ~MultiProbMapDisplay();

  // Overrides from Display
  virtual void onInitialize();
  virtual void reset();
  virtual void update(float wall_dt, float ros_dt);

protected Q_SLOTS:
  void updateTopic();
  void updateDrawUnder();

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  virtual void subscribe();
  virtual void unsubscribe();

  void incomingMap(const multi_map_msgs::msg::MultiOccupancyGrid::SharedPtr msg);

  void clear();

  std::vector<Ogre::ManualObject*> manual_object_;
  std::vector<Ogre::TexturePtr>    texture_;
  std::vector<Ogre::MaterialPtr>   material_;

  bool loaded_;

  std::string topic_;

  rclcpp::Subscription<multi_map_msgs::msg::MultiOccupancyGrid>::SharedPtr map_sub_;

  rviz_common::properties::RosTopicProperty* topic_property_;
  rviz_common::properties::Property*         draw_under_property_;

  multi_map_msgs::msg::MultiOccupancyGrid::SharedPtr updated_map_;
  multi_map_msgs::msg::MultiOccupancyGrid::SharedPtr current_map_;
  std::mutex                                   mutex_;
  bool                                           new_map_;

  rclcpp::Node::SharedPtr update_nh_;
};

} // namespace rviz

#endif
