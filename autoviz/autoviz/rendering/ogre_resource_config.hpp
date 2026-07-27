/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include <string>

namespace autoviz {
namespace rendering {

/** Ogre media root (contains plugins.cfg, materials/, fonts/, …). */
std::string ogreResourceDirectory();

/** Directory containing RenderSystem_GL and Codec_STBI plugins. */
std::string ogrePluginDirectory();

void setOgreResourceDirectory(const std::string& path);
void setOgrePluginDirectory(const std::string& path);

}  // namespace rendering
}  // namespace autoviz

#endif
