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

#include "autonomy/tools/aviz/rendering/geometry.hpp"

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreMath.h>
#include <OGRE/OgreViewport.h>

#include <cmath>

namespace aviz {
namespace rendering {

float mapAngleTo0_2Pi(float angle) {
    angle = std::fmod(angle, Ogre::Math::TWO_PI);
    if (angle < 0.0f) {
        angle = Ogre::Math::TWO_PI + angle;
    }
    return angle;
}

Ogre::Vector2 project3DPointToViewportXY(const Ogre::Viewport* view,
                                         const Ogre::Vector3& pos) {
    Ogre::Camera* cam = view->getCamera();
    Ogre::Vector3 pos2D =
        cam->getProjectionMatrix() * (cam->getViewMatrix() * pos);

    Ogre::Real x = static_cast<Ogre::Real>((pos2D.x * 0.5) + 0.5);
    Ogre::Real y = static_cast<Ogre::Real>(1 - ((pos2D.y * 0.5) + 0.5));

    return Ogre::Vector2(x * view->getActualWidth(),
                         y * view->getActualHeight());
}

}  // namespace rendering
}  // namespace aviz
