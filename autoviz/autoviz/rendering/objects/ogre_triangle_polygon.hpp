/******************************************************************************
 * Copyright 2024, Open Source Robotics Foundation, Inc.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include <string>

#include <OgreColourValue.h>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector.h>

namespace autoviz {
namespace rendering {

/** rviz_rendering::TrianglePolygon — single textured triangle ManualObject. */
class OgreTrianglePolygon {
 public:
  OgreTrianglePolygon(Ogre::SceneManager* manager, Ogre::SceneNode* node,
                      const Ogre::Vector3& O, const Ogre::Vector3& A,
                      const Ogre::Vector3& B, const std::string& name,
                      const Ogre::ColourValue& color, bool use_color,
                      bool upper_triangle);
  ~OgreTrianglePolygon();

  Ogre::ManualObject* manualObject() { return manual_; }

 private:
  Ogre::ManualObject* manual_ = nullptr;
  Ogre::SceneManager* manager_ = nullptr;
};

}  // namespace rendering
}  // namespace autoviz

#endif
