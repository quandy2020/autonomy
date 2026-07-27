/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/apply_visibility_bits.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <Ogre.h>

namespace autoviz {
namespace rendering {

void applyVisibilityBits(uint32_t bits, Ogre::SceneNode* node) {
  if (node == nullptr) {
    return;
  }
  Ogre::SceneNode::ObjectIterator objects = node->getAttachedObjectIterator();
  while (objects.hasMoreElements()) {
    objects.getNext()->setVisibilityFlags(bits);
  }
  Ogre::SceneNode::ChildNodeIterator children = node->getChildIterator();
  while (children.hasMoreElements()) {
    applyVisibilityBits(bits, static_cast<Ogre::SceneNode*>(children.getNext()));
  }
}

}  // namespace rendering
}  // namespace autoviz

#endif
