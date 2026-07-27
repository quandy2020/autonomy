/******************************************************************************
 * Copyright 2008, Willow Garage, Inc.
 * Copyright 2017, Open Source Robotics Foundation, Inc.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include <OgreMatrix4.h>

namespace autoviz {
namespace rendering {

Ogre::Matrix4 buildScaledOrthoMatrix(float left, float right, float bottom, float top,
                                     float near_plane, float far_plane);

}  // namespace rendering
}  // namespace autoviz

#endif
