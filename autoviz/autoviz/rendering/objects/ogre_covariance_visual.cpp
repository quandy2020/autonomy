/******************************************************************************
 * Copyright 2017, Ellon Paiva Mendes @ LAAS-CNRS
 * Copyright 2018, Bosch Software Innovations GmbH.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#include "autoviz/rendering/objects/ogre_covariance_visual.hpp"

#include <cmath>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

namespace autoviz {
namespace rendering {

OgreCovarianceVisual::OgreCovarianceVisual(Ogre::SceneManager* scene_manager,
                                           Ogre::SceneNode* parent_node,
                                           bool is_local_rotation,
                                           bool is_visible, float pos_scale,
                                           float ori_scale, float ori_offset)
    : scene_manager_(scene_manager),
      local_rotation_(is_local_rotation),
      visible_(is_visible),
      position_scale_(pos_scale),
      orientation_scale_(ori_scale),
      orientation_offset_(ori_offset) {
  if (scene_manager_ == nullptr || parent_node == nullptr) {
    return;
  }
  root_node_ = parent_node->createChildSceneNode();
  position_node_ = root_node_->createChildSceneNode();
  position_shape_ = std::make_unique<OgreShape>(OgreShape::kSphere,
                                                scene_manager_, position_node_);
  position_shape_->setScale(
      Ogre::Vector3(position_scale_, position_scale_, position_scale_));
  setVisible(visible_);
}

OgreCovarianceVisual::~OgreCovarianceVisual() {
  position_shape_.reset();
  if (scene_manager_ != nullptr && root_node_ != nullptr) {
    scene_manager_->destroySceneNode(root_node_);
    root_node_ = nullptr;
  }
}

void OgreCovarianceVisual::updateUserData(const CovarianceUserData& user_data) {
  visible_ = user_data.visible;
  position_scale_ = user_data.position_scale;
  orientation_scale_ = user_data.orientation_scale;
  orientation_offset_ = user_data.orientation_offset;
  if (position_shape_ != nullptr) {
    position_shape_->setColor(user_data.position_color);
    position_shape_->setScale(
        Ogre::Vector3(position_scale_, position_scale_, position_scale_));
  }
  if (position_node_ != nullptr) {
    position_node_->setVisible(user_data.visible && user_data.position_visible);
  }
  setVisible(user_data.visible);
}

void OgreCovarianceVisual::setCovariance(
    const Ogre::Quaternion& /*pose_orientation*/,
    const std::array<double, 36>& covariances) {
  if (position_shape_ == nullptr) {
    return;
  }
  // Diagonal position variances → axis-aligned ellipsoid (orientation axes later).
  const float sx =
      position_scale_ * static_cast<float>(std::sqrt(std::max(covariances[0], 1e-9)));
  const float sy =
      position_scale_ * static_cast<float>(std::sqrt(std::max(covariances[7], 1e-9)));
  const float sz =
      position_scale_ * static_cast<float>(std::sqrt(std::max(covariances[14], 1e-9)));
  position_shape_->setScale(Ogre::Vector3(sx, sy, sz));
}

void OgreCovarianceVisual::setPosition(const Ogre::Vector3& position) {
  if (root_node_ != nullptr) {
    root_node_->setPosition(position);
  }
}

void OgreCovarianceVisual::setOrientation(const Ogre::Quaternion& orientation) {
  if (root_node_ != nullptr) {
    root_node_->setOrientation(orientation);
  }
}

void OgreCovarianceVisual::setVisible(bool visible) {
  visible_ = visible;
  if (root_node_ != nullptr) {
    root_node_->setVisible(visible);
  }
}

}  // namespace rendering
}  // namespace autoviz

