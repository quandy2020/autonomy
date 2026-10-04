/******************************************************************************
 * Copyright 2017, Ellon Paiva Mendes @ LAAS-CNRS
 * Copyright 2018, Bosch Software Innovations GmbH.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

/**
 * @file ogre_covariance_visual.hpp
 * @brief Pose covariance ellipsoids (rviz CovarianceVisual subset).
 */

#pragma once

#include <array>
#include <memory>

#include <OgreColourValue.h>
#include <OgreQuaternion.h>
#include <OgreVector.h>

#include "autoviz/rendering/objects/ogre_shape.hpp"

namespace Ogre {
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

enum class Frame { Local, Fixed };
enum class ColorStyle { Unique, RGB };

struct CovarianceUserData {
  bool visible = true;
  bool position_visible = true;
  Ogre::ColourValue position_color = Ogre::ColourValue::White;
  float position_scale = 1.f;
  bool orientation_visible = true;
  Frame orientation_frame = Frame::Local;
  ColorStyle orientation_color_style = ColorStyle::RGB;
  Ogre::ColourValue orientation_color = Ogre::ColourValue::White;
  float orientation_offset = 0.1f;
  float orientation_scale = 0.1f;
};

class OgreCovarianceVisual {
 public:
  OgreCovarianceVisual(Ogre::SceneManager* scene_manager,
                       Ogre::SceneNode* parent_node,
                       bool is_local_rotation = false, bool is_visible = true,
                       float pos_scale = 1.0f, float ori_scale = 0.1f,
                       float ori_offset = 0.1f);
  ~OgreCovarianceVisual();

  void updateUserData(const CovarianceUserData& user_data);
  void setCovariance(const Ogre::Quaternion& pose_orientation,
                     const std::array<double, 36>& covariances);
  void setPosition(const Ogre::Vector3& position);
  void setOrientation(const Ogre::Quaternion& orientation);
  void setVisible(bool visible);

 private:
  Ogre::SceneManager* scene_manager_ = nullptr;
  Ogre::SceneNode* root_node_ = nullptr;
  Ogre::SceneNode* position_node_ = nullptr;
  std::unique_ptr<OgreShape> position_shape_;
  bool local_rotation_ = false;
  bool visible_ = true;
  float position_scale_ = 1.f;
  float orientation_scale_ = 0.1f;
  float orientation_offset_ = 0.1f;
};

}  // namespace rendering
}  // namespace autoviz

