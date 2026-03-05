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

#include "autonomy/tools/aviz/rendering/render_system.hpp"

#include <OGRE/OgreConfigFile.h>
#include <OGRE/OgreLogManager.h>
#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgrePrerequisites.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreResourceGroupManager.h>

#include <iostream>
#include <memory>
#include <string>

namespace aviz {
namespace rendering {

RenderSystem* RenderSystem::instance_ = nullptr;
int RenderSystem::force_gl_version_ = 0;
bool RenderSystem::force_no_stereo_ = false;

// Disable anti aliasing on Windows for now
#ifndef _WIN32
bool RenderSystem::use_anti_aliasing_ = true;
#else
bool RenderSystem::use_anti_aliasing_ = false;
#endif

RenderSystem* RenderSystem::get() {
  if (instance_ == nullptr) {
    instance_ = new RenderSystem();
  }
  return instance_;
}

RenderSystem::RenderSystem()
    : ogre_root_(nullptr)
#if OGRE_VERSION_HIGHER_OR_EQUAL_1_9_0
      ,
      ogre_overlay_system_(nullptr)
#endif
#ifdef __linux__
      ,
      dummyDisplay(nullptr),
      dummyWindow(0),
      dummyContext(nullptr)
#endif
{
  // Initialize Ogre::Root
  ogre_root_ = new Ogre::Root("", "", "aviz.log");

  // Setup render system
  setupRenderSystem();

  // Load plugins
  loadOgrePlugins();

  // Initialize root
  // Newer Ogre versions require a ConfigDialog* argument; pass nullptr to
  // use the default dialog implementation when available.
  if (!ogre_root_->restoreConfig() && !ogre_root_->showConfigDialog(nullptr)) {
    // Use default configuration if no config file exists
    ogre_root_->setRenderSystem(ogre_root_->getAvailableRenderers().at(0));
  }

  ogre_root_->initialise(false);

#if OGRE_VERSION_HIGHER_OR_EQUAL_1_9_0
  ogre_overlay_system_ = new Ogre::OverlaySystem();
#endif
}

RenderSystem::~RenderSystem() { Destroy(); }

void RenderSystem::Destroy() {
#if OGRE_VERSION_HIGHER_OR_EQUAL_1_9_0
  if (ogre_overlay_system_) {
    delete ogre_overlay_system_;
    ogre_overlay_system_ = nullptr;
  }
#endif

  if (ogre_root_) {
    try {
      delete ogre_root_;
    } catch (...) {
    }
    ogre_root_ = nullptr;
  }

#ifdef __linux__
  if (dummyDisplay) {
    // Cleanup dummy display
  }
#endif
}

Ogre::RenderWindow* RenderSystem::makeRenderWindow(WindowIDType window_id, unsigned int width, unsigned int height,
                                                   double pixel_ratio) {
  std::string name = "AVizRenderWindow";
  Ogre::NameValuePairList params;

  params["externalWindowHandle"] = std::to_string(window_id);
  params["parentWindowHandle"] = std::to_string(window_id);

  return tryMakeRenderWindow(name, width, height, &params, 5);
}

Ogre::RenderWindow* RenderSystem::tryMakeRenderWindow(const std::string& name, unsigned int width, unsigned int height,
                                                      const Ogre::NameValuePairList* params, int max_attempts) {
  for (int i = 0; i < max_attempts; ++i) {
    try {
      std::string window_name = name;
      if (i > 0) {
        window_name += std::to_string(i);
      }

      Ogre::NameValuePairList local_params;
      if (params) {
        local_params = *params;
      }

      return ogre_root_->createRenderWindow(window_name, width, height, false, &local_params);
    } catch (Ogre::Exception& e) {
      if (i == max_attempts - 1) {
        std::cerr << "Failed to create render window after " << max_attempts << " attempts: " << e.what() << std::endl;
      }
    }
  }
  return nullptr;
}

void RenderSystem::setupRenderSystem() {
  // Configure render system options
  Ogre::RenderSystemList renderers = ogre_root_->getAvailableRenderers();
  if (renderers.empty()) {
    std::cerr << "No render systems available!" << std::endl;
    return;
  }

  // Use first available render system (typically GL3+)
  Ogre::RenderSystem* render_system = renderers[0];

  if (force_gl_version_ > 0) {
    render_system->setConfigOption("OpenGL Version", std::to_string(force_gl_version_ / 100) + "." +
                                                         std::to_string((force_gl_version_ % 100) / 10));
  }

  if (force_no_stereo_) {
    render_system->setConfigOption("stereoMode", "None");
  }

  ogre_root_->setRenderSystem(render_system);
}

void RenderSystem::loadOgrePlugins() {
  // Load essential plugins
  // This is a simplified version - in practice, you'd want to configure plugin paths
}

void RenderSystem::setResourceDirectory() {
  // Set resource directories
}

void RenderSystem::setPluginDirectory() {
  // Set plugin directories
}

void RenderSystem::setupResources() {
  // Setup resource groups
}

#if OGRE_VERSION_HIGHER_OR_EQUAL_1_9_0
void RenderSystem::prepareOverlays(Ogre::SceneManager* scene_manager) {
  if (ogre_overlay_system_) {
    scene_manager->addRenderQueueListener(ogre_overlay_system_);
  }
}
#endif

int RenderSystem::getGlVersion() {
  // Extract GL version from render system
  return 330;  // Default to 3.30
}

int RenderSystem::getGlslVersion() {
  // Extract GLSL version from render system
  return 330;  // Default to 3.30
}

void RenderSystem::disableAntiAliasing() { use_anti_aliasing_ = false; }

void RenderSystem::forceGlVersion(int version) { force_gl_version_ = version; }

void RenderSystem::forceNoStereo() { force_no_stereo_ = true; }

bool RenderSystem::isStereoSupported() {
  return false;  // Simplified implementation
}

void RenderSystem::setupDummyWindowId() {
  // Setup dummy window for headless rendering if needed
#ifdef __linux__
  // X11 setup code would go here
#endif
}

}  // namespace rendering
}  // namespace aviz
