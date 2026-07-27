/******************************************************************************
 * Copyright 2019, Martin Idel · Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include <memory>

#include <OgreQuaternion.h>
#include <OgreVector3.h>

namespace Ogre {
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

class OgreArrow;
class OgreBillboardLine;

/** rviz_rendering::WrenchVisual — force/torque arrow visualization. */
class OgreWrenchVisual {
 public:
  OgreWrenchVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node);
  ~OgreWrenchVisual();

  void setWrench(const Ogre::Vector3& force, const Ogre::Vector3& torque);
  void setFramePosition(const Ogre::Vector3& position);
  void setFrameOrientation(const Ogre::Quaternion& orientation);
  void setForceColor(float r, float g, float b, float a);
  void setTorqueColor(float r, float g, float b, float a);
  void setForceScale(float scale);
  void setTorqueScale(float scale);
  void setWidth(float width);
  void setVisible(bool visible);

 private:
  void createTorqueDirectionCircle(const Ogre::Quaternion& orientation) const;
  void setTorqueDirectionArrow(const Ogre::Quaternion& orientation) const;
  Ogre::Quaternion getDirectionOfRotationRelativeToTorque(const Ogre::Vector3& torque,
                                                          const Ogre::Vector3& axis_z) const;
  void updateForceArrow() const;
  void updateTorque() const;

  std::unique_ptr<OgreArrow> arrow_force_;
  std::unique_ptr<OgreArrow> arrow_torque_;
  std::unique_ptr<OgreBillboardLine> circle_torque_;
  std::unique_ptr<OgreArrow> circle_arrow_torque_;
  Ogre::Vector3 force_arrow_direction_;
  Ogre::Vector3 torque_arrow_direction_;
  float force_scale_ = 1.f;
  float torque_scale_ = 1.f;
  float width_ = 1.f;
  Ogre::SceneNode* frame_node_ = nullptr;
  Ogre::SceneNode* force_node_ = nullptr;
  Ogre::SceneNode* torque_node_ = nullptr;
  Ogre::SceneManager* scene_manager_ = nullptr;
};

}  // namespace rendering
}  // namespace autoviz

#endif
