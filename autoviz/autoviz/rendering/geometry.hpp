/******************************************************************************
 * Copyright 2012, Willow Garage, Inc.
 * Copyright 2017, Open Source Robotics Foundation, Inc.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

/**
 * @file geometry.hpp
 * @brief Small Ogre geometry helpers (angle wrap, world-to-viewport projection).
 *
 * Adapted from rviz_rendering utilities used by overlays and tools that need
 * screen-space placement of 3D points.
 *
 * @see ViewportProjectionFinder
 * @see project3DPointToViewportXY()
 */

#pragma once

#include <OgreVector.h>

namespace Ogre {
class Viewport;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

/**
 * @brief Wraps an angle in radians into the half-open interval \f$[0, 2\pi)\f$.
 *
 * @param angle Input angle in radians (any real value).
 * @return Equivalent angle in \f$[0, 2\pi)\f$.
 */
float mapAngleTo0_2Pi(float angle);

/**
 * @brief Projects a world-space point to viewport pixel coordinates.
 *
 * Uses the viewport's active camera projection. Origin is typically top-left
 * in Ogre viewport space.
 *
 * @param view Non-null Ogre viewport with an attached camera.
 * @param pos World-space position (same frame as the camera).
 * @return 2D pixel coordinates within the viewport.
 *
 * @see ViewportProjectionFinder
 */
Ogre::Vector2 project3DPointToViewportXY(const Ogre::Viewport* view,
                                         const Ogre::Vector3& pos);

}  // namespace rendering
}  // namespace autoviz

