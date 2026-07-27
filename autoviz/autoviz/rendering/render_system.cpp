/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/render_system.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include <Ogre.h>
#include <Overlay/OgreOverlaySystem.h>

#include <QOpenGLContext>

#include <glog/logging.h>

#include "autoviz/rendering/gpu_capabilities.hpp"
#include "autoviz/rendering/ogre_logging.hpp"
#include "autoviz/rendering/ogre_material_manager.hpp"
#include "autoviz/rendering/ogre_procedural_shape.hpp"
#include "autoviz/rendering/ogre_resource_config.hpp"

namespace autoviz {
namespace rendering {
namespace {

constexpr char kResourceGroup[] = "rviz_rendering";

void AddResourceLocation(const std::string& path) {
  if (!std::filesystem::exists(path)) {
    LOG(WARNING) << "Ogre media path missing: " << path;
    return;
  }
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
      path, "FileSystem", kResourceGroup);
}

void AddResourceLocation(const std::string& base, const std::string& subpath) {
  AddResourceLocation((std::filesystem::path(base) / subpath).string());
}

}  // namespace

RenderSystem* RenderSystem::instance_ = nullptr;
RenderSystem::WindowHandle RenderSystem::test_window_handle_ = 0;
bool RenderSystem::skip_render_window_ = false;
bool RenderSystem::use_anti_aliasing_ = true;
int RenderSystem::force_gl_version_ = 0;
bool RenderSystem::force_no_stereo_ = false;

RenderSystem* RenderSystem::instance() {
  if (instance_ == nullptr) {
    instance_ = new RenderSystem();
  }
  return instance_;
}

void RenderSystem::destroyInstance() {
  delete instance_;
  instance_ = nullptr;
  test_window_handle_ = 0;
  skip_render_window_ = false;
}

void RenderSystem::setTestWindowHandle(WindowHandle handle) {
  test_window_handle_ = handle;
}

void RenderSystem::setSkipRenderWindow(bool skip) {
  skip_render_window_ = skip;
}

void RenderSystem::disableAntiAliasing() {
  use_anti_aliasing_ = false;
  AUTOVIZ_OGRE_LOG_INFO("Disabling Anti-Aliasing");
}

void RenderSystem::forceGlVersion(int version) {
  force_gl_version_ = version;
  AUTOVIZ_OGRE_LOG_INFO_STREAM("Forcing OpenGl version " << version / 100.0 << ".");
}

void RenderSystem::forceNoStereo() {
  force_no_stereo_ = true;
  AUTOVIZ_OGRE_LOG_INFO("Forcing Stereo OFF");
}

RenderSystem::RenderSystem() = default;

RenderSystem::~RenderSystem() { shutdown(); }

bool RenderSystem::ensureInitialized() {
  if (initialized_) {
    return true;
  }

  const std::string resource_dir = ogreResourceDirectory();
  if (resource_dir.empty()) {
    LOG(ERROR) << "Autoviz ogre_media directory not found. Set AUTOVIZ_OGRE_MEDIA_PATH.";
    return false;
  }

  const std::string plugins_cfg =
      (std::filesystem::path(resource_dir) / "plugins.cfg").string();
  OgreOgreLogging::instance()->noLog();
  OgreOgreLogging::instance()->configureLogging();
  ogre_root_ = new Ogre::Root(plugins_cfg);
  overlay_system_ = new Ogre::OverlaySystem();

  loadOgrePlugins();
  setupRenderSystem();
  ogre_root_->initialise(false);
  if (!skip_render_window_) {
    ensureHeadlessRenderWindow();
  }
  detectGlVersion();
  setupResources();

  try {
#ifdef AUTOVIZ_OGRE_RVIZ_MEDIA
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
#else
    Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(
        kResourceGroup);
#endif
  } catch (const Ogre::Exception& ex) {
    LOG(ERROR) << "Failed to initialise Ogre resource groups: " << ex.getFullDescription();
    shutdown();
    return false;
  }

  OgreMaterialManager::ensureDefaultMaterials();
#ifdef AUTOVIZ_OGRE_RVIZ_MEDIA
  OgreMaterialManager::ensureRvizMediaMaterials();
#else
  OgreMaterialManager::ensureStubRvizMaterials();
#endif
  ensureRvizPrimitiveMeshes();
  initialized_ = true;
  const char* visual_mode =
#ifdef AUTOVIZ_OGRE_RVIZ_MEDIA
      "rviz_glsl";
#else
      "stub";
#endif
  LOG(INFO) << "Autoviz RenderSystem ready (GL " << gl_version_ / 100.0 << ", GLSL "
            << glsl_version_ / 100.0 << ", visual=" << visual_mode
            << ", media=" << resource_dir << ")";
  return true;
}

void RenderSystem::shutdown() {
  if (ogre_root_ != nullptr && headless_window_ != nullptr) {
    ogre_root_->destroyRenderTarget(headless_window_);
    headless_window_ = nullptr;
  }
  if (overlay_system_ != nullptr) {
    delete overlay_system_;
    overlay_system_ = nullptr;
  }
  if (ogre_root_ != nullptr) {
    delete ogre_root_;
    ogre_root_ = nullptr;
  }
  initialized_ = false;
}

void RenderSystem::ensureHeadlessRenderWindow() {
  if (headless_window_ != nullptr || ogre_root_ == nullptr) {
    return;
  }
  static int window_counter = 0;
  Ogre::NameValuePairList params;
  if (test_window_handle_ != 0) {
    params["currentGLContext"] = "False";
    params["externalWindowHandle"] =
        Ogre::StringConverter::toString(static_cast<size_t>(test_window_handle_));
    params["parentWindowHandle"] = params["externalWindowHandle"];
  } else {
    params["currentGLContext"] = "False";
  }
  headless_window_ = ogre_root_->createRenderWindow(
      "AvizHeadless" + Ogre::StringConverter::toString(window_counter++), 1, 1,
      false, &params);
}

Ogre::Root* RenderSystem::ogreRoot() { return ogre_root_; }

Ogre::OverlaySystem* RenderSystem::overlaySystem() { return overlay_system_; }

void RenderSystem::loadOgrePlugins() {
  const std::filesystem::path plugin_prefix(ogrePluginDirectory());
  if (plugin_prefix.empty()) {
    LOG(WARNING) << "Ogre plugin directory unknown; relying on linked RenderSystem.";
    return;
  }
  const char* render_plugins[] = {
      "RenderSystem_GL3Plus", "RenderSystem_GL", "RenderSystem_GLES2"};
  for (const char* plugin_name : render_plugins) {
    const std::filesystem::path plugin_path = plugin_prefix / plugin_name;
    const std::filesystem::path plugin_so =
        plugin_prefix / (std::string(plugin_name) + ".so");
    if (std::filesystem::exists(plugin_so)) {
      ogre_root_->loadPlugin(plugin_so.string());
    } else if (std::filesystem::exists(plugin_path)) {
      ogre_root_->loadPlugin(plugin_path.string());
    }
  }
  const std::filesystem::path stbi_plugin = plugin_prefix / "Codec_STBI.so";
  if (std::filesystem::exists(stbi_plugin)) {
    ogre_root_->loadPlugin(stbi_plugin.string());
  } else {
    const std::filesystem::path stbi = plugin_prefix / "Codec_STBI";
    if (std::filesystem::exists(stbi)) {
      ogre_root_->loadPlugin(stbi.string());
    }
  }
}

void RenderSystem::setupRenderSystem() {
  Ogre::RenderSystem* render_system = nullptr;
  const std::vector<std::string> preferred = {"OpenGL 3+", "OpenGL 3 Plus",
                                              "OpenGL", "OpenGL ES"};
  for (const std::string& token : preferred) {
    for (Ogre::RenderSystem* candidate : ogre_root_->getAvailableRenderers()) {
      if (candidate->getName().find(token) != std::string::npos) {
        render_system = candidate;
        break;
      }
    }
    if (render_system != nullptr) {
      break;
    }
  }
  if (render_system == nullptr) {
    throw std::runtime_error("Could not find Ogre OpenGL render system.");
  }
  render_system->setConfigOption("Full Screen", "No");
  if (use_anti_aliasing_) {
    render_system->setConfigOption("FSAA", "4");
  }
  ogre_root_->setRenderSystem(render_system);
  GpuCapabilities::instance().probeFromRendererString(render_system->getName());
}

void RenderSystem::detectGlVersion() {
  if (force_gl_version_ != 0) {
    gl_version_ = force_gl_version_;
  } else if (QOpenGLContext::currentContext() != nullptr) {
    GpuCapabilities::instance().probeFromOpenGL();
  }
  Ogre::RenderSystem* render_system = ogre_root_->getRenderSystem();
  if (render_system != nullptr) {
    const Ogre::RenderSystemCapabilities* active = render_system->getCapabilities();
    if (active != nullptr) {
      gl_version_ = active->getDriverVersion().major * 100 +
                    active->getDriverVersion().minor * 10;
    }
  }
  switch (gl_version_) {
    case 200:
      glsl_version_ = 110;
      break;
    case 210:
      glsl_version_ = 120;
      break;
    case 300:
      glsl_version_ = 130;
      break;
    case 310:
      glsl_version_ = 140;
      break;
    case 320:
      glsl_version_ = 150;
      break;
    default:
      glsl_version_ = gl_version_ > 320 ? gl_version_ : 120;
      break;
  }
  if (glsl_version_ < 120) {
    throw std::runtime_error(
        "OpenGL 2.1+ required for rviz ogre_media shaders (GLSL 1.20).");
  }
}

void RenderSystem::setupResources() {
  auto& group_manager = Ogre::ResourceGroupManager::getSingleton();
  if (!group_manager.resourceGroupExists("AvizOgre")) {
    group_manager.createResourceGroup("AvizOgre");
  }
  const std::string base = ogreResourceDirectory();
  AddResourceLocation(base, ".");
  AddResourceLocation(base, "textures");
  AddResourceLocation(base, "fonts");
  AddResourceLocation(base, "fonts/liberation-sans");
  AddResourceLocation(base, "models");
#ifdef AUTOVIZ_OGRE_RVIZ_MEDIA
  AddResourceLocation(base, "materials");
  AddResourceLocation(base, "materials/scripts");
  AddResourceLocation(base, "materials/glsl120");
  AddResourceLocation(base, "materials/glsl120/include");
  AddResourceLocation(base, "materials/glsl120/nogp");
  if (glsl_version_ >= 120) {
    AddResourceLocation(base, "materials/scripts120");
  }
#endif
}

void RenderSystem::prepareOverlays(Ogre::SceneManager* scene_manager) {
  if (overlay_system_ != nullptr && scene_manager != nullptr) {
    scene_manager->addRenderQueueListener(overlay_system_);
  }
}

Ogre::RenderWindow* RenderSystem::makeRenderWindow(WindowHandle window_id,
                                                   unsigned width,
                                                   unsigned height,
                                                   double pixel_ratio) {
  if (!ensureInitialized()) {
    return nullptr;
  }
  static int window_counter = 0;
  Ogre::NameValuePairList params;
  params["currentGLContext"] = "False";
  params["externalWindowHandle"] =
      Ogre::StringConverter::toString(static_cast<size_t>(window_id));
  params["parentWindowHandle"] = params["externalWindowHandle"];
  params["left"] = "0";
  params["top"] = "0";
  params["contentScalingFactor"] = Ogre::StringConverter::toString(pixel_ratio);
#if defined(__APPLE__)
  params["macAPI"] = "cocoa";
  params["macAPICocoaUseNSView"] = "true";
#endif
  if (use_anti_aliasing_) {
    params["FSAA"] = "4";
  }
#if !defined(OGRE_STEREO_ENABLE)
  force_no_stereo_ = true;
#endif
  Ogre::RenderWindow* window = nullptr;
  if (!force_no_stereo_) {
    params["stereoMode"] = "Frame Sequential";
    window = ogre_root_->createRenderWindow(
        "AvizRenderWindow" + Ogre::StringConverter::toString(window_counter++),
        width, height, false, &params);
    params.erase("stereoMode");
#if defined(OGRE_STEREO_ENABLE)
    if (window != nullptr && window->isStereoEnabled()) {
      stereo_supported_ = true;
      return window;
    }
#endif
    if (window != nullptr) {
      ogre_root_->destroyRenderTarget(window);
      window = nullptr;
    }
  }
  window = ogre_root_->createRenderWindow(
      "AvizRenderWindow" + Ogre::StringConverter::toString(window_counter++),
      width, height, false, &params);
  stereo_supported_ = false;
  return window;
}

}  // namespace rendering
}  // namespace autoviz

#endif
