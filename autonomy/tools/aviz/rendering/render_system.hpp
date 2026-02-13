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

#define OGRE_VERSION_HIGHER_OR_EQUAL_1_9_0 (OGRE_VERSION >= ((1 << 16) | (9 << 8) | 0))

#include <cstdint>
#include <string>

#include <OGRE/OgrePrerequisites.h>
#include <OGRE/OgreRoot.h>

#if OGRE_VERSION_HIGHER_OR_EQUAL_1_9_0
// OgreOverlaySystem.h includes OgreOverlayPrerequisites.h which uses relative path
// So we need to ensure OgrePrerequisites.h is already included
// Include Overlay headers after OgrePrerequisites.h
#include <OGRE/Overlay/OgreOverlayPrerequisites.h>
#include <OGRE/Overlay/OgreOverlaySystem.h>
#endif

#ifdef __linux__
#include <GL/glx.h>
#include <X11/Xutil.h>
#endif

// Local visibility macro (previously from visibility_control.hpp)
#ifndef AVIZ_RENDERING_PUBLIC
#define AVIZ_RENDERING_PUBLIC
#endif

namespace aviz {
namespace common {
class Display;  // Forward declaration to avoid X11 Display conflict
}  // namespace common
namespace rendering {

class RenderSystem
{
public:
#if defined(__APPLE__) || defined(_WIN32)
    typedef size_t WindowIDType;
#else
    typedef unsigned long WindowIDType;  // NOLINT: we need to use C longs here
#endif

    AVIZ_RENDERING_PUBLIC
    static RenderSystem* get();

    AVIZ_RENDERING_PUBLIC
    Ogre::RenderWindow* makeRenderWindow(WindowIDType window_id, unsigned int width, unsigned int height,
                                         double pixel_ratio = 1.0);

    Ogre::Root* getOgreRoot() {
        return ogre_root_;
    }

    AVIZ_RENDERING_PUBLIC
    ~RenderSystem();

    void Destroy();

#if OGRE_VERSION_HIGHER_OR_EQUAL_1_9_0
    // Prepare a scene_manager to render overlays
    void prepareOverlays(Ogre::SceneManager* scene_manager);
#endif

    /// return OpenGL Version as integer, e.g. 320 for OpenGL 3.20
    int getGlVersion();

    /// return GLSL Version as integer, e.g. 150 for GLSL 1.50
    AVIZ_RENDERING_PUBLIC
    int getGlslVersion();

    /// Disables the use of Anti Aliasing
    static void disableAntiAliasing();

    /// Force to use the provided OpenGL version on startup
    static void forceGlVersion(int version);

    /// Disable stereo rendering even if supported in HW
    static void forceNoStereo();

    /// True if we can render stereo on this device
    bool isStereoSupported();

private:
    RenderSystem();

    void setupDummyWindowId();
    void loadOgrePlugins();
    void setupRenderSystem();
    void setResourceDirectory();
    void setPluginDirectory();
    void setupResources();

    Ogre::RenderWindow* tryMakeRenderWindow(const std::string& name, unsigned int width, unsigned int height,
                                            const Ogre::NameValuePairList* params, int max_attempts);

    static RenderSystem* instance_;
    static int force_gl_version_;
    static bool force_no_stereo_;
    static bool use_anti_aliasing_;

    Ogre::Root* ogre_root_;
#if OGRE_VERSION_HIGHER_OR_EQUAL_1_9_0
    Ogre::OverlaySystem* ogre_overlay_system_;
#endif

#ifdef __linux__
    aviz::common::Display* dummyDisplay;
    Window dummyWindow;
    GLXContext dummyContext;
#endif
};

}  // namespace rendering
}  // namespace aviz
