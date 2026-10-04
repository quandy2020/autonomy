/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file ogre_resource_config.hpp
 * @brief Resolve and override Ogre media / plugin directory paths.
 *
 * Defaults come from build-time install layout; tests and alternative installs
 * can override via @ref setOgreResourceDirectory() /
 * @ref setOgrePluginDirectory() before @ref RenderSystem::ensureInitialized().
 *
 * @see RenderSystem
 */

#pragma once

#include <string>

namespace autoviz {
namespace rendering {

/**
 * @brief Returns the Ogre media root directory.
 *
 * Contains @c plugins.cfg, @c materials/, @c fonts/, and related assets.
 *
 * @return Absolute or configured path string.
 * @see setOgreResourceDirectory()
 */
std::string ogreResourceDirectory();

/**
 * @brief Returns the directory containing Ogre RenderSystem / Codec plugins.
 *
 * Typically holds @c RenderSystem_GL and @c Codec_STBI (and siblings).
 *
 * @return Absolute or configured path string.
 * @see setOgrePluginDirectory()
 */
std::string ogrePluginDirectory();

/**
 * @brief Overrides the media root used by @ref ogreResourceDirectory().
 *
 * @param path Directory containing Ogre media assets.
 * @note Call before @ref RenderSystem::ensureInitialized().
 */
void setOgreResourceDirectory(const std::string& path);

/**
 * @brief Overrides the plugin directory used by @ref ogrePluginDirectory().
 *
 * @param path Directory containing Ogre `.so` / `.dylib` plugins.
 * @note Call before @ref RenderSystem::ensureInitialized().
 */
void setOgrePluginDirectory(const std::string& path);

/**
 * @brief Reads a text file under @ref ogreResourceDirectory().
 *
 * @param relative_path Path relative to the media root
 *        (e.g. @c "materials/glsl330/colored.vert").
 * @return File contents, or empty string if missing / unreadable.
 */
std::string loadOgreMediaText(const std::string& relative_path);

}  // namespace rendering
}  // namespace autoviz

