/******************************************************************************
 * Copyright 2012, Willow Garage, Inc. · Copyright 2017–2018, Open Source Robotics Foundation, Inc.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#pragma once

#include <utility>

#include <QMatrix4x4>
#include <QVector3D>

#ifdef AUTOVIZ_USE_OGRE
namespace Ogre {
class Plane;
class Viewport;
}  // namespace Ogre
#endif

namespace autoviz {
namespace rendering {

/** rviz_rendering::ViewportProjectionFinder — screen pixel to world on a plane. */
class ViewportProjectionFinder {
 public:
  ViewportProjectionFinder() = default;
  ~ViewportProjectionFinder() = default;

  /** Intersect view ray with Z=0 plane (fixed-frame ground). */
  static std::pair<bool, QVector3D> projectOnGroundPlane(
      int pixel_x, int pixel_y, int viewport_width, int viewport_height,
      const QMatrix4x4& view, const QMatrix4x4& projection);

#ifdef AUTOVIZ_USE_OGRE
  static std::pair<bool, QVector3D> projectOgreViewportOnGroundPlane(
      Ogre::Viewport* viewport, int pixel_x, int pixel_y);

  static std::pair<bool, QVector3D> projectOgreViewportOnPlane(
      Ogre::Viewport* viewport, int pixel_x, int pixel_y, Ogre::Plane& plane);
#endif
};

}  // namespace rendering
}  // namespace autoviz
