/******************************************************************************
 * Copyright 2008, Willow Garage, Inc.
 * Copyright 2017, Open Source Robotics Foundation, Inc.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#include "autoviz/rendering/orthographic.hpp"

#ifdef AUTOVIZ_USE_OGRE

namespace autoviz {
namespace rendering {

Ogre::Matrix4 buildScaledOrthoMatrix(float left, float right, float bottom, float top,
                                     float near_plane, float far_plane) {
  const float inverse_width = 1 / (right - left);
  const float inverse_height = 1 / (top - bottom);
  const float inverse_distance = 1 / (far_plane - near_plane);

  auto proj = Ogre::Matrix4::ZERO;
  proj[0][0] = 2 * inverse_width;
  proj[0][3] = -(right + left) * inverse_width;
  proj[1][1] = 2 * inverse_height;
  proj[1][3] = -(top + bottom) * inverse_height;
  proj[2][2] = -2 * inverse_distance;
  proj[2][3] = -(far_plane + near_plane) * inverse_distance;
  proj[3][3] = 1;
  return proj;
}

}  // namespace rendering
}  // namespace autoviz

#endif
