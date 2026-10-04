/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file apply_visibility_bits.hpp
 * @brief Apply Ogre visibility masks to all movables under a scene node.
 *
 * Port of @c rviz_rendering::applyVisibilityBits: walks @p node and sets
 * @c Ogre::MovableObject visibility flags so display layers can be masked
 * for pick / overlay passes.
 *
 * @see OgreSceneHost::setDisplayVisibilityBits()
 */

#pragma once

#include <cstdint>

namespace Ogre {
class SceneNode;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

/**
 * @brief Sets @c Ogre::MovableObject visibility flags under @p node.
 *
 * Recursively applies @p bits to every movable attached to @p node and its
 * descendants (rviz_rendering behavior).
 *
 * @param bits Visibility bit mask (typically from a Display property).
 * @param node Root scene node for the display entry; may be @c nullptr (no-op).
 *
 * @see OgreSceneHost::setDisplayVisibilityBits()
 */
void applyVisibilityBits(uint32_t bits, Ogre::SceneNode* node);

}  // namespace rendering
}  // namespace autoviz

