/******************************************************************************
 * Copyright 2024, Open Source Robotics Foundation, Inc.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#include "autoviz/rendering/objects/ogre_triangle_polygon.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <stdexcept>

#include <OgreRenderOperation.h>

namespace autoviz {
namespace rendering {

OgreTrianglePolygon::OgreTrianglePolygon(Ogre::SceneManager* manager,
                                         Ogre::SceneNode* node,
                                         const Ogre::Vector3& O,
                                         const Ogre::Vector3& A,
                                         const Ogre::Vector3& B,
                                         const std::string& name,
                                         const Ogre::ColourValue& color,
                                         bool use_color, bool upper_triangle)
    : manager_(manager) {
  if (manager == nullptr || node == nullptr) {
    throw std::invalid_argument("SceneManager and SceneNode must not be null.");
  }
  manual_ = manager->createManualObject();
  manual_->clear();
  manual_->begin(name, Ogre::RenderOperation::OT_TRIANGLE_STRIP);
  manual_->position(O.x(), O.y(), O.z());
  manual_->textureCoord(upper_triangle ? 0.f : 1.f, 0.f);
  if (use_color) {
    manual_->colour(color);
  }
  manual_->position(A.x(), A.y(), A.z());
  manual_->textureCoord(1.f, upper_triangle ? 0.f : 1.f);
  if (use_color) {
    manual_->colour(color);
  }
  manual_->position(B.x(), B.y(), B.z());
  manual_->textureCoord(0.f, 1.f);
  if (use_color) {
    manual_->colour(color);
  }
  manual_->end();
  node->attachObject(manual_);
}

OgreTrianglePolygon::~OgreTrianglePolygon() {
  if (manual_ != nullptr) {
    manual_->detachFromParent();
  }
}

}  // namespace rendering
}  // namespace autoviz

#endif
