#include "rviz_plugins/pose_tool.h"

#include <cmath>

#include <OgreCamera.h>
#include <OgreSceneManager.h>

#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/view_manager.hpp"

namespace rviz_plugins
{

// Corrected constructor: Removed initialization of 'context_'
Pose3DTool::Pose3DTool()
: rviz_common::Tool(), arrow_(nullptr), state_(Position)
{
}

Pose3DTool::~Pose3DTool()
{
  delete arrow_;
  for (auto arrow : arrow_array_) {
    delete arrow;
  }
}

void Pose3DTool::onInitialize()
{
  // Use the 'context_' member inherited from the base Tool class.
  arrow_ = new rviz_rendering::Arrow(context_->getSceneManager(), nullptr, 2.0f, 0.2f, 0.5f, 0.35f);
  arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
  arrow_->getSceneNode()->setVisible(false);
}

void Pose3DTool::activate()
{
  setStatus("Click and drag mouse to set position/orientation.");
  state_ = Position;
}

void Pose3DTool::deactivate()
{
  arrow_->getSceneNode()->setVisible(false);
  for (auto arrow : arrow_array_) {
    arrow->getSceneNode()->setVisible(false);
  }
}

// Added a default implementation to make the class concrete and loadable.
void Pose3DTool::onPoseSet(double x, double y, double z, double theta)
{
  RVIZ_COMMON_LOG_INFO_STREAM(
    "Pose3DTool: Pose set to " <<
    "x: " << x <<
    ", y: " << y <<
    ", z: " << z <<
    ", theta: " << theta);
}

int Pose3DTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
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

  rviz_common::ViewController * view_controller = context_->getViewManager()->getCurrent();
  Ogre::Camera * camera = view_controller? view_controller->getCamera() : nullptr;
  if (!camera) {
    return flags;
  }

  Ogre::Ray ray = camera->getCameraToViewportRay(
    static_cast<float>(event.x) / camera->getViewport()->getActualWidth(),
    static_cast<float>(event.y) / camera->getViewport()->getActualHeight());

  auto result = ray.intersects(ground_plane);

  if (event.leftDown()) {
    if (result.first) {
      pos_ = ray.getPoint(result.second);
      arrow_->setPosition(pos_);
      state_ = Orientation;
      flags |= Render;
    }
  } else if (event.type == QEvent::MouseMove && event.left()) {
    if (state_ == Orientation) {
      if (result.first) {
        Ogre::Vector3 cur_pos = ray.getPoint(result.second);
        double angle = atan2(cur_pos.y - pos_.y, cur_pos.x - pos_.x);
        arrow_->getSceneNode()->setVisible(true);
        arrow_->setOrientation(Ogre::Quaternion(Ogre::Radian(angle), Ogre::Vector3::UNIT_Z) * orient_x);

        if (event.right()) {
          state_ = Height;
          initz = pos_.z;
          prevy = event.y;
        }
        prevangle = angle;
        flags |= Render;
      }
    } else if (state_ == Height) {
      double dy = event.y - prevy;
      prevy = event.y;
      pos_.z -= dy / z_scale;
      arrow_->setPosition(pos_);

      for (auto arrow : arrow_array_) {
        delete arrow;
      }
      arrow_array_.clear();

      int cnt = static_cast<int>(ceil(fabs(initz - pos_.z) / z_interval));
      for (int k = 0; k < cnt; k++) {
        rviz_rendering::Arrow * arrow__;
        arrow__ = new rviz_rendering::Arrow(context_->getSceneManager(), nullptr, 0.5f, 0.1f, 0.0f, 0.1f);
        arrow__->setColor(0.0f, 1.0f, 0.0f, 1.0f);
        arrow__->getSceneNode()->setVisible(true);
        Ogre::Vector3 arr_pos = pos_;
        arr_pos.z = initz - ((initz - pos_.z > 0)? 1 : -1) * k * z_interval;
        arrow__->setPosition(arr_pos);
        arrow__->setOrientation(
          Ogre::Quaternion(Ogre::Radian(prevangle), Ogre::Vector3::UNIT_Z) * orient_x);
        arrow_array_.push_back(arrow__);
      }
      flags |= Render;
    }
  } else if (event.leftUp()) {
    // Corrected the syntax error from '| |' to '||'
    if (state_ == Orientation || state_ == Height) {
      for (auto arrow : arrow_array_) {
        delete arrow;
      }
      arrow_array_.clear();
      onPoseSet(pos_.x, pos_.y, pos_.z, prevangle);
      flags |= (Finished | Render);
    }
  }

  return flags;
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::Pose3DTool, rviz_common::Tool)