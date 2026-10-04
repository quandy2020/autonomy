/******************************************************************************
 * Copyright 2008, Willow Garage, Inc.
 * Copyright 2017, Open Source Robotics Foundation, Inc.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

/**
 * @file orthographic.hpp
 * @brief Build a scaled orthographic projection matrix for Ogre cameras.
 *
 * Adapted from rviz_rendering; used by TopDownOrtho / orthographic view paths
 * when constructing @c Ogre::Camera projection matrices.
 *
 * @see ViewController::projectionMatrix()
 * @see ViewportProjectionFinder
 */

#pragma once

#include <OgreMatrix4.h>

namespace autoviz {
namespace rendering {

/**
 * @brief Builds an Ogre orthographic projection matrix with explicit bounds.
 *
 * Equivalent to a scaled ortho frustum: maps the box
 * \f$[\mathit{left},\mathit{right}]\times[\mathit{bottom},\mathit{top}]\times
 * [\mathit{near},\mathit{far}]\f$ into NDC.
 *
 * @param left Left clip plane (world / view units).
 * @param right Right clip plane.
 * @param bottom Bottom clip plane.
 * @param top Top clip plane.
 * @param near_plane Near clip distance.
 * @param far_plane Far clip distance.
 * @return Column-major @c Ogre::Matrix4 suitable for @c Camera::setCustomProjectionMatrix.
 */
Ogre::Matrix4 buildScaledOrthoMatrix(float left, float right, float bottom, float top,
                                     float near_plane, float far_plane);

}  // namespace rendering
}  // namespace autoviz

