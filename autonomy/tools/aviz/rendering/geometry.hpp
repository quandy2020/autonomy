/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <OGRE/OgreVector2.h>
#include <OGRE/OgreVector3.h>

namespace Ogre {
class Viewport;
class Plane;
}  // namespace Ogre

namespace aviz {
namespace rendering {

/**
 * @brief Geometry utility functions
 * Geometry utilities for rendering
 */

/// Return the input angle mapped back to the range 0 to 2*PI
float mapAngleTo0_2Pi(float angle);

/// Project a 3D point into the view plane based on a given 3D position and a Viewport
/**
 * \return The 2D floating-point pixel position of the projection.
 */
Ogre::Vector2 project3DPointToViewportXY(const Ogre::Viewport* view, const Ogre::Vector3& pos);

}  // namespace rendering
}  // namespace aviz
