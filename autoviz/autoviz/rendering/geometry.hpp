/******************************************************************************
 * Copyright 2012, Willow Garage, Inc.
 * Copyright 2017, Open Source Robotics Foundation, Inc.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include <OgreVector.h>

namespace Ogre {
class Viewport;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

float mapAngleTo0_2Pi(float angle);
Ogre::Vector2 project3DPointToViewportXY(const Ogre::Viewport* view,
                                         const Ogre::Vector3& pos);

}  // namespace rendering
}  // namespace autoviz

#endif
