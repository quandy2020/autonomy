/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <functional>
#include <string>

#include <QColor>
#include <QCursor>
#include <QWidget>

#include "autoviz/rendering/grid_renderer.hpp"
#include "autoviz/rendering/ogre_render_backend.hpp"
#include "autoviz/rendering/render_settings.hpp"
#include "autoviz/rendering/scene_overlay.hpp"
#include "autoviz/rendering/view_controller.hpp"

namespace autoviz {
namespace common {
class ToolManager;
}

namespace rendering {

/** QWidget viewport backed by Ogre (requires AUTOVIZ_USE_OGRE). */
class OgreRenderWindow : public QWidget {
  Q_OBJECT

 public:
  explicit OgreRenderWindow(QWidget* parent = nullptr);
  ~OgreRenderWindow() override;

  void setSceneOverlay(SceneOverlay* overlay) { scene_overlay_ = overlay; }
  void setGridVisible(bool visible) { grid_renderer_.setVisible(visible); }
  void setReferenceGridSettings(const ReferenceGridSettings& settings) {
    reference_grid_settings_ = settings;
  }
  void setBackgroundColor(const QColor& color) {
    grid_renderer_.setBackgroundColor(color);
    ogre_backend_.setBackgroundColor(color);
  }

  ViewController& viewController() { return view_controller_; }
  const ViewController& viewController() const { return view_controller_; }

  void setToolManager(common::ToolManager* tool_manager) {
    tool_manager_ = tool_manager;
  }
  void setViewportToolId(std::string tool_id) {
    viewport_tool_id_ = std::move(tool_id);
  }
  const std::string& viewportToolId() const { return viewport_tool_id_; }
  void setToolCursor(const QCursor& cursor) {
    tool_cursor_ = cursor;
    setCursor(cursor);
  }

  void syncToolCursorFromManager();

  void tick(float delta_seconds);

  void setViewInteractionCallbacks(std::function<void()> on_drag_started,
                                   std::function<void()> on_drag_ended);
  void setViewportActivationCallback(std::function<void()> on_activate) {
    on_viewport_activate_ = std::move(on_activate);
  }

  bool readDepthPick(int pixel_x, int pixel_y, const QMatrix4x4& view,
                     const QMatrix4x4& projection, QVector3D* world) const;
  common::PickHandle readPickHandleAt(int pixel_x, int pixel_y) const;

  OgreSceneHost* ogreSceneHost() { return ogre_backend_.ogreSceneHost(); }
  const OgreSceneHost* ogreSceneHost() const {
    return ogre_backend_.ogreSceneHost();
  }

 signals:
  void viewDragStarted();
  void viewDragUpdated();
  void viewDragEnded();
  /** Letter shortcut switched the active tool (g/p/…). */
  void toolShortcutTriggered();

 protected:
  void resizeEvent(QResizeEvent* event) override;
  void paintEvent(QPaintEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void mouseReleaseEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;
  void keyPressEvent(QKeyEvent* event) override;
  void keyReleaseEvent(QKeyEvent* event) override;
  void enterEvent(QEnterEvent* event) override;

 private:
  GridRenderer grid_renderer_;
  SceneOverlay* scene_overlay_ = nullptr;
  ViewController view_controller_;
  common::ToolManager* tool_manager_ = nullptr;
  std::string viewport_tool_id_ = "Interact";
  QCursor tool_cursor_ = Qt::ArrowCursor;
  OgreRenderBackend ogre_backend_;
  ReferenceGridSettings reference_grid_settings_;
  std::function<void()> on_view_drag_started_;
  std::function<void()> on_view_drag_ended_;
  std::function<void()> on_viewport_activate_;
};

}  // namespace rendering
}  // namespace autoviz
