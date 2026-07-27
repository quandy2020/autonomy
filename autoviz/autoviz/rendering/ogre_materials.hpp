/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

namespace Ogre {
class SceneManager;
}  // namespace Ogre

namespace autoviz {
namespace rendering {

/** Register Autoviz Ogre materials: point sprite disc + GLSL PBR. */
void EnsureOgreMaterials(Ogre::SceneManager* scene);

}  // namespace rendering
}  // namespace autoviz

#endif
