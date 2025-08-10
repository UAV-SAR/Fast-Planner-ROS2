#ifndef RVIZ_PLUGINS__POSE_TOOL_H_
#define RVIZ_PLUGINS__POSE_TOOL_H_

#include <vector>

#include <QCursor>

#include <OgrePlane.h>
#include <OgreRay.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <OgreViewport.h>

#include "rviz_common/tool.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_rendering/objects/arrow.hpp"

namespace rviz_plugins
{

class Pose3DTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  Pose3DTool();
  ~Pose3DTool() override;

  void onInitialize() override;

  void activate() override;
  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

protected:
  /**
   * @brief This function is called when the user completes the pose selection.
   * A derived class would typically override this to publish the pose as a ROS message.
   * It is now a regular virtual function with a default implementation.
   */
  virtual void onPoseSet(double x, double y, double z, double theta);

  // Member variables for the main arrow and the array for height visualization
  rviz_rendering::Arrow * arrow_;
  std::vector<rviz_rendering::Arrow *> arrow_array_;

  // The state machine for the tool's behavior
  enum State
  {
    Position,
    Orientation,
    Height
  };
  State state_;

  // The current 3D position being manipulated
  Ogre::Vector3 pos_;

  // No 'context_' member here; it is inherited from the base class.
};

}  // namespace rviz_plugins

#endif  // RVIZ_PLUGINS__POSE_TOOL_H_