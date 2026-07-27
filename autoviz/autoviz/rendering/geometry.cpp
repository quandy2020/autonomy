/******************************************************************************
 * Copyright 2012, Willow Garage, Inc.
 * Copyright 2017, Open Source Robotics Foundation, Inc.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#include "autoviz/rendering/geometry.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <OgreCamera.h>
#include <OgreViewport.h>

namespace autoviz {
namespace rendering {

float mapAngleTo0_2Pi(float angle) {
  angle = fmod(angle, Ogre::Math::TWO_PI);
  if (angle < 0.0f) {
    angle = Ogre::Math::TWO_PI + angle;
  }
  return angle;
}

Ogre::Vector2 project3DPointToViewportXY(const Ogre::Viewport* view,
                                         const Ogre::Vector3& pos) {
  Ogre::Camera* cam = view->getCamera();
  Ogre::Vector3 pos2d = cam->getProjectionMatrix() * (cam->getViewMatrix() * pos);
  const Ogre::Real x = static_cast<Ogre::Real>((pos2d.x * 0.5) + 0.5);
  const Ogre::Real y = static_cast<Ogre::Real>(1 - ((pos2d.y * 0.5) + 0.5));
  return Ogre::Vector2(x * view->getActualWidth(), y * view->getActualHeight());
}

}  // namespace rendering
}  // namespace autoviz

#endif
