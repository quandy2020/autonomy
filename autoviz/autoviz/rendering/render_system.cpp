/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/rendering/render_system.hpp"

#include <algorithm>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include <Ogre.h>
#include <OgreOverlaySystem.h>

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

constexpr char kResourceGroup[] = "aviz_rendering";

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

  // Prefer manual plugin load via AUTOVIZ_OGRE_PLUGIN_DIR; only use plugins.cfg
  // when present so a missing media copy does not spam Ogre errors.
  std::string plugins_cfg;
  const std::filesystem::path plugins_cfg_path =
      std::filesystem::path(resource_dir) / "plugins.cfg";
  if (std::filesystem::exists(plugins_cfg_path)) {
    plugins_cfg = plugins_cfg_path.string();
  }
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
#ifdef AUTOVIZ_OGRE_AVIZ_MEDIA
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
#ifdef AUTOVIZ_OGRE_AVIZ_MEDIA
  OgreMaterialManager::ensureAvizMediaMaterials();
#else
  OgreMaterialManager::ensureStubAvizMaterials();
#endif
  ensureAvizPrimitiveMeshes();
  initialized_ = true;
  const char* visual_mode =
#ifdef AUTOVIZ_OGRE_AVIZ_MEDIA
      "aviz_glsl";
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
  // Bootstrap GL context only. Without hidden=true Ogre creates a top-level
  // X11/Win32 window that shows up as a second application next to Aviz.
  params["hidden"] = "true";
  params["border"] = "none";
  if (test_window_handle_ != 0) {
    params["currentGLContext"] = "False";
    params["parentWindowHandle"] =
        Ogre::StringConverter::toString(static_cast<size_t>(test_window_handle_));
  } else {
    params["currentGLContext"] = "False";
  }
  headless_window_ = ogre_root_->createRenderWindow(
      "AvizHeadless" + Ogre::StringConverter::toString(window_counter++), 1, 1,
      false, &params);
  if (headless_window_ != nullptr) {
    headless_window_->setVisible(false);
    headless_window_->setAutoUpdated(false);
  }
}

Ogre::Root* RenderSystem::ogreRoot() { return ogre_root_; }

Ogre::OverlaySystem* RenderSystem::overlaySystem() { return overlay_system_; }

void RenderSystem::loadOgrePlugins() {
  std::vector<std::filesystem::path> prefixes;
  const std::filesystem::path primary(ogrePluginDirectory());
  if (!primary.empty()) {
    prefixes.push_back(primary);
  }
#ifdef AUTOVIZ_OGRE_PLUGIN_DIR
  {
    const std::filesystem::path baked(AUTOVIZ_OGRE_PLUGIN_DIR);
    if (!baked.empty() &&
        std::find(prefixes.begin(), prefixes.end(), baked) == prefixes.end()) {
      prefixes.push_back(baked);
    }
  }
#endif

  auto try_load = [this](const std::filesystem::path& prefix,
                         const char* name) {
    if (prefix.empty() || !std::filesystem::is_directory(prefix)) {
      return false;
    }
    std::vector<std::filesystem::path> candidates = {
        prefix / (std::string(name) + ".dylib"),
        prefix / (std::string(name) + ".so"),
        prefix / name,
    };
    // Ogre may install versioned sonames (RenderSystem_GL.so.1.12.10).
    try {
      for (const auto& entry : std::filesystem::directory_iterator(prefix)) {
        if (!entry.is_regular_file() && !entry.is_symlink()) {
          continue;
        }
        const std::string filename = entry.path().filename().string();
        if (filename == name || filename.rfind(std::string(name) + ".", 0) == 0) {
          candidates.push_back(entry.path());
        }
      }
    } catch (const std::filesystem::filesystem_error&) {
      // Directory vanished between is_directory and iterate; ignore.
    }
    for (const std::filesystem::path& candidate : candidates) {
      if (!std::filesystem::exists(candidate)) {
        continue;
      }
      try {
        ogre_root_->loadPlugin(candidate.string());
        LOG(INFO) << "Loaded Ogre plugin: " << candidate;
        return true;
      } catch (const Ogre::Exception& ex) {
        LOG(WARNING) << "Failed to load Ogre plugin " << candidate << ": "
                     << ex.getFullDescription();
      }
    }
    return false;
  };

  const char* render_plugins[] = {
      // Classic GL RenderSystem is required for rviz ogre_media GLSL 1.20.
      // This is Ogre's GPU driver plugin — not Autoviz's retired QOpenGLWidget
      // viewport.
      "RenderSystem_GL", "RenderSystem_GL3Plus", "RenderSystem_GLES2"};
  bool loaded_rs = false;
  for (const std::filesystem::path& plugin_prefix : prefixes) {
    for (const char* plugin_name : render_plugins) {
      if (try_load(plugin_prefix, plugin_name)) {
        loaded_rs = true;
        break;
      }
    }
    if (loaded_rs) {
      try_load(plugin_prefix, "Codec_STBI");
      break;
    }
  }
  if (!loaded_rs) {
    LOG(ERROR) << "No Ogre RenderSystem plugin found. Searched:";
    for (const std::filesystem::path& plugin_prefix : prefixes) {
      LOG(ERROR) << "  " << plugin_prefix;
    }
    LOG(ERROR) << "Set AUTOVIZ_OGRE_PLUGIN_DIR to the directory that contains "
                  "RenderSystem_GL.so (built with Autoviz's Ogre 1.12).";
  }
}

void RenderSystem::setupRenderSystem() {
  Ogre::RenderSystem* render_system = nullptr;
  Ogre::RenderSystem* gl3plus_fallback = nullptr;
  for (Ogre::RenderSystem* candidate : ogre_root_->getAvailableRenderers()) {
    const Ogre::String& name = candidate->getName();
    if (name.find("OpenGL 3+") != Ogre::String::npos ||
        name.find("OpenGL 3 Plus") != Ogre::String::npos) {
      gl3plus_fallback = candidate;
      continue;
    }
    if (name.find("OpenGL") != Ogre::String::npos) {
      render_system = candidate;
      break;
    }
    if (name.find("OpenGL ES") != Ogre::String::npos &&
        render_system == nullptr) {
      render_system = candidate;
    }
  }
  if (render_system == nullptr) {
    render_system = gl3plus_fallback;
  }
  if (render_system == nullptr) {
    throw std::runtime_error(
        "Could not find an Ogre RenderSystem plugin (need RenderSystem_GL).");
  }
  LOG(INFO) << "Using Ogre render system: " << render_system->getName();
  render_system->setConfigOption("Full Screen", "No");
  if (use_anti_aliasing_) {
    try {
      render_system->setConfigOption("FSAA", "4");
    } catch (const Ogre::Exception&) {
      // Some drivers omit FSAA; ignore.
    }
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
#ifdef AUTOVIZ_OGRE_AVIZ_MEDIA
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
  // parentWindowHandle embeds into the Qt widget. Do not also set
  // externalWindowHandle — with both set, Ogre takes the parent path and can
  // still leave a stray top-level drawable on some GLX builds.
  params["parentWindowHandle"] =
      Ogre::StringConverter::toString(static_cast<size_t>(window_id));
  params["left"] = "0";
  params["top"] = "0";
  params["border"] = "none";
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

