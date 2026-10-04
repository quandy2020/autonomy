/******************************************************************************
 * Copyright 2012, Willow Garage, Inc. · Copyright 2017–2018, Open Source Robotics Foundation, Inc.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

/**
 * @file viewport_projection_finder.hpp
 * @brief Project screen pixels onto a world plane (rviz ViewportProjectionFinder).
 *
 * Converts a viewport pixel into a 3D hit on the ground plane (Z=0) or an
 * arbitrary Ogre plane — used by Publish Point, Measure, and pose tools.
 *
 * @see ViewController::pickGroundPoint()
 * @see geometry.hpp
 */

#pragma once

#include <utility>

#include <QMatrix4x4>
#include <QVector3D>

namespace Ogre {
class Plane;
class Viewport;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

/**
 * @class ViewportProjectionFinder
 * @brief rviz_rendering::ViewportProjectionFinder — screen pixel to world on a plane.
 *
 * Stateless helper; all methods are static. Qt matrix overloads work for both
 * OpenGL and Ogre hosts; Ogre viewport overloads require Ogre.
 */
class ViewportProjectionFinder {
 public:
  ViewportProjectionFinder() = default;
  ~ViewportProjectionFinder() = default;

  /**
   * @brief Intersects the view ray with the Z=0 ground plane (fixed frame).
   *
   * @param pixel_x Pixel X in the viewport.
   * @param pixel_y Pixel Y in the viewport.
   * @param viewport_width Viewport width in pixels.
   * @param viewport_height Viewport height in pixels.
   * @param view Camera view matrix.
   * @param projection Camera projection matrix.
   * @return Pair of (hit, world_position). @c first is @c false if the ray is
   *         parallel to the plane or behind the camera.
   */
  static std::pair<bool, QVector3D> projectOnGroundPlane(
      int pixel_x, int pixel_y, int viewport_width, int viewport_height,
      const QMatrix4x4& view, const QMatrix4x4& projection);

  /**
   * @brief Same as @ref projectOnGroundPlane() using an Ogre viewport's camera.
   *
   * @param viewport Non-null Ogre viewport.
   * @param pixel_x Pixel X.
   * @param pixel_y Pixel Y.
   * @return Pair of (hit, world_position).
   */
  static std::pair<bool, QVector3D> projectOgreViewportOnGroundPlane(
      Ogre::Viewport* viewport, int pixel_x, int pixel_y);

  /**
   * @brief Intersects the Ogre viewport ray with an arbitrary plane.
   *
   * @param viewport Non-null Ogre viewport.
   * @param pixel_x Pixel X.
   * @param pixel_y Pixel Y.
   * @param plane Target plane in the same space as the camera (modified only
   *        if the Ogre API requires a non-const reference).
   * @return Pair of (hit, world_position).
   */
  static std::pair<bool, QVector3D> projectOgreViewportOnPlane(
      Ogre::Viewport* viewport, int pixel_x, int pixel_y, Ogre::Plane& plane);
};

}  // namespace rendering
}  // namespace autoviz
