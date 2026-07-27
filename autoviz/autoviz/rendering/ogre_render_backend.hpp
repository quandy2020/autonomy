/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <memory>

#include <QWidget>

#include <QColor>

#include "autoviz/common/pick_handle.hpp"
#include "autoviz/common/pick_registry.hpp"
#include "autoviz/rendering/gl_pick_framebuffer.hpp"
#include "autoviz/rendering/grid_renderer.hpp"
#include "autoviz/rendering/render_settings.hpp"
#include "autoviz/rendering/scene_overlay.hpp"
#include "autoviz/rendering/view_controller.hpp"

namespace autoviz {
namespace rendering {

class OgreSceneHost;

/** Optional Ogre rendering backend (enabled with AUTOVIZ_USE_OGRE). */
class OgreRenderBackend {
 public:
  explicit OgreRenderBackend(QWidget* host);
  ~OgreRenderBackend();

  OgreRenderBackend(const OgreRenderBackend&) = delete;
  OgreRenderBackend& operator=(const OgreRenderBackend&) = delete;

  bool initialize();
  void resize(int width, int height);
  void render(bool show_grid, const ReferenceGridSettings& grid_settings,
              SceneOverlay* overlay, const ViewController& view_controller,
              float aspect_ratio);
  void setBackgroundColor(const QColor& color);
  void shutdown();

  /** Reads depth at pixel after the last render (OpenGL context required). */
  bool pickDepthAt(int pixel_x, int pixel_y, int viewport_width,
                   int viewport_height, const QMatrix4x4& view,
                   const QMatrix4x4& projection, QVector3D* world_out) const;

  common::PickHandle pickHandleAt(int pixel_x, int pixel_y, int viewport_width,
                                  int viewport_height,
                                  common::PickRegistry* pick_registry = nullptr) const;

  OgreSceneHost* ogreSceneHost();
  const OgreSceneHost* ogreSceneHost() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  QWidget* host_ = nullptr;
};

}  // namespace rendering
}  // namespace autoviz
