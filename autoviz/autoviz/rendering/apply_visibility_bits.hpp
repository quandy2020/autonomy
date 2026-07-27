/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include <cstdint>

namespace Ogre {
class SceneNode;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

/** rviz_rendering::applyVisibilityBits — set Ogre::MovableObject visibility flags. */
void applyVisibilityBits(uint32_t bits, Ogre::SceneNode* node);

}  // namespace rendering
}  // namespace autoviz

#endif
