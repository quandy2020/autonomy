/******************************************************************************
 * Copyright 2008, Willow Garage, Inc. · Copyright 2017, Open Source Robotics
 * Foundation, Inc. · Adapted for Autoviz (BSD-3-Clause).
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from this
 *     software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND; see the
 * RViz BSD-3-Clause license for the full disclaimer.
 *****************************************************************************/


/**
 * @file ogre_covariance_visual.hpp
 * @brief Position / orientation covariance ellipsoids for pose displays.
 *
 * Port of @c rviz_rendering::CovarianceVisual using Eigen: an ellipsoid for the
 * position covariance and 2D ellipses along each axis for the orientation
 * covariance (cone for planar 2D poses).
 *
 * @see OgreShape
 * @see OgreSceneHost::setDisplayCovariance()
 */

#pragma once

#include <array>
#include <memory>
#include <vector>

// GCC 11 has a false positive warning about uninitialized variables in Eigen
// (https://gitlab.com/libeigen/eigen/-/issues/2304), and Eigen 3.4.0 triggers
// -Wclass-memaccess on aarch64. Silence both while including Eigen.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#elif defined(_WIN32)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
#include <Eigen/Dense>
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC diagnostic pop
#elif defined(_WIN32)
#pragma warning(pop)
#endif

#include <OgreAxisAlignedBox.h>
#include <OgreColourValue.h>
#include <OgreQuaternion.h>
#include <OgreVector.h>

#include "autoviz/rendering/objects/ogre_shape.hpp"

namespace Ogre {
class SceneManager;
class SceneNode;
class Any;
}  // namespace Ogre

namespace Eigen {
using Matrix6d = Matrix<double, 6, 6>;
}

namespace autoviz {
namespace rendering {

/** @brief Frame the orientation covariance is attached to. */
enum class Frame { Local, Fixed };

/** @brief Color style of the orientation covariance shapes. */
enum class ColorStyle { Unique, RGB };

/**
 * @struct CovarianceUserData
 * @brief User-adjustable display parameters (rviz CovarianceProperty values).
 */
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

/**
 * @class OgreCovarianceVisual
 * @brief Covariance visual: ellipsoid for position, 2D ellipses along each
 *        axis for orientation.
 */
class OgreCovarianceVisual {
 public:
  enum ShapeIndex {
    kRoll = 0,
    kPitch = 1,
    kYaw = 2,
    kYaw2D = 3,
    kNumOrientationShapes
  };

  /**
   * @param scene_manager Scene manager used to construct objects.
   * @param parent_node Node this covariance is attached to (must be non-null).
   * @param is_local_rotation Initial attachment of the rotation part.
   * @param is_visible Initial visibility.
   * @param pos_scale Scale of the position covariance.
   * @param ori_scale Scale of the orientation covariance.
   * @param ori_offset Offset of the orientation shapes along their axis.
   * @throws std::runtime_error if @p scene_manager or @p parent_node is null.
   */
  OgreCovarianceVisual(Ogre::SceneManager* scene_manager,
                       Ogre::SceneNode* parent_node,
                       bool is_local_rotation = false, bool is_visible = true,
                       float pos_scale = 1.0f, float ori_scale = 0.1f,
                       float ori_offset = 0.1f);

  virtual ~OgreCovarianceVisual();

  OgreCovarianceVisual(const OgreCovarianceVisual&) = delete;
  OgreCovarianceVisual& operator=(const OgreCovarianceVisual&) = delete;

  /** @brief Applies user-adjustable display parameters. */
  void updateUserData(const CovarianceUserData& user_data);

  /**
   * @brief Sets the covariance (row-major 6x6: x,y,z,roll,pitch,yaw).
   *
   * Changes the orientation and scale of the position and orientation shapes.
   */
  virtual void setCovariance(const Ogre::Quaternion& pose_orientation,
                             const std::array<double, 36>& covariances);

  /** @brief Sets the position of the frame this covariance is attached to. */
  void setPosition(const Ogre::Vector3& position);

  /** @brief Sets the orientation of the frame this covariance is attached to. */
  void setOrientation(const Ogre::Quaternion& orientation);

  /** @brief Sets visibility of both position and orientation parts. */
  virtual void setVisible(bool visible);

  /** @brief Root scene node of the position part. */
  Ogre::SceneNode* getPositionSceneNode() { return position_scale_node_; }

  /** @brief Root scene node of the orientation part. */
  Ogre::SceneNode* getOrientationSceneNode() { return orientation_root_node_; }

  Ogre::AxisAlignedBox getPositionBoundingBox();
  std::vector<Ogre::AxisAlignedBox> getOrientationBoundingBoxes();

 private:
  void setScales(float pos_scale, float ori_scale);
  void setPositionScale(float pos_scale);
  void setOrientationOffset(float ori_offset);
  void setOrientationScale(float ori_scale);
  void setPositionColor(const Ogre::ColourValue& color);
  void setOrientationColor(const Ogre::ColourValue& color);
  void setOrientationColorToRGB(float alpha);

  /** @brief Shape used to display the position covariance. */
  std::shared_ptr<OgreShape> getPositionShape() { return position_shape_; }

  virtual void setPositionVisible(bool visible);
  virtual void setOrientationVisible(bool visible);
  virtual void setRotatingFrame(bool use_rotating_frame);

  void updatePosition(const Eigen::Matrix6d& covariance);
  void updateOrientation(const Eigen::Matrix6d& covariance, ShapeIndex index);
  void updateOrientationVisibility();

  Ogre::SceneManager* scene_manager_;
  Ogre::SceneNode* root_node_;
  Ogre::SceneNode* fixed_orientation_node_;
  Ogre::SceneNode* position_scale_node_;
  Ogre::SceneNode* position_node_;
  Ogre::SceneNode* orientation_root_node_;
  std::array<Ogre::SceneNode*, kNumOrientationShapes> orientation_offset_nodes_;

  /// Ellipse used for the position covariance.
  std::shared_ptr<OgreShape> position_shape_;
  /// Cylinders and cone used for the orientation covariance.
  std::array<std::shared_ptr<OgreShape>, kNumOrientationShapes>
      orientation_shapes_;

  bool local_rotation_;
  bool pose_2d_;
  bool orientation_visible_;  ///< If the orientation component is visible.

  std::array<Ogre::Vector3, kNumOrientationShapes> current_orientation_scales_;
  float current_orientation_scale_factor_ = 1.0f;

  static constexpr float kMaxDegrees = 89.0f;
};

}  // namespace rendering
}  // namespace autoviz
