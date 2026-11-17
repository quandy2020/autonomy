/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file viewport_panel.hpp
 * @brief Per-dock viewport state: render window, overlays, and local tool.
 *
 * @ref ViewportPanelEntry is the value type stored in
 * @ref FrameViewport::viewport_panels_. It does not own the dock (the frame
 * does); it holds non-owning pointers to widgets created under that dock.
 *
 * @see FrameViewport
 * @see ViewportHudOverlay
 * @see rendering::OgreRenderWindow
 */

#pragma once

#include <memory>
#include <optional>
#include <string>

#include <QString>

#include "autoviz/rendering/scene_overlay.hpp"
#include "autoviz/rendering/view_controller.hpp"

class QGridLayout;
class QToolButton;
class QWidget;

namespace autoviz {

class PanelDockWidget;
class ViewportHudOverlay;

namespace rendering {
class OgreRenderWindow;
}

/**
 * @struct ViewportPanelEntry
 * @brief One 3D/2D viewport dock and its Ogre render window + overlays.
 *
 * @c ogre_viewport is non-null after
 * @ref FrameViewport::createRenderWindowInEntry().
 * @ref viewController() returns the controller from that window.
 *
 * ## Local tools
 *
 * @c local_tool_id is per-split (Select / Measure / Interact) and is not shared
 * across 3D panels — switching the active dock pushes its local tool into the
 * shared tool context via @ref FrameViewport::pushViewportLocalTool().
 *
 * @see FrameViewport::wireViewportPanel()
 */
struct ViewportPanelEntry {
  PanelDockWidget* dock = nullptr;  /**< Host dock widget (non-owning). */
  /**
   * Cached @c dock->objectName() at registration. Prefer this over calling
   * @c objectName() during render ticks — a disposed dock left in
   * @c viewport_panels_ would otherwise SIGSEGV in @c QObject::objectName.
   */
  QString object_name;
  QWidget* host = nullptr;          /**< Content host inside the dock. */
  QGridLayout* layout = nullptr;    /**< Layout stacking GL + overlays. */
  QWidget* widget = nullptr;        /**< Render window's QWidget surface. */
  rendering::OgreRenderWindow* ogre_viewport = nullptr; /**< Ogre viewport. */
  /**
   * Per-view scene overlay (shared_ptr so QHash copies stay safe). Split 3D
   * Views must not share the manager global overlay.
   */
  std::shared_ptr<rendering::SceneOverlay> scene_overlay =
      std::make_shared<rendering::SceneOverlay>();
  /** Visibility bit allocated for this view's Display ownership. */
  uint32_t visibility_bit = 0;
  ViewportHudOverlay* hud_overlay = nullptr;            /**< Top-left HUD. */
  QToolButton* expand_button = nullptr;   /**< Title-bar expand toggle. */
  QToolButton* settings_button = nullptr; /**< Title-bar settings toggle. */
  QToolButton* float_button = nullptr;    /**< Title-bar free-float toggle. */
  QToolButton* content_only_button = nullptr; /**< Hide chrome while floating. */

  /**
   * Per-split tool (Select / Measure / Interact); not shared across 3D panels.
   */
  std::string local_tool_id = "Interact";

  /**
   * @c true after @ref FrameSession::safeguardNativeViewportsBeforeHide —
   * GL widget is unparented so @c hideChildren cannot walk @c WA_PaintOnScreen.
   */
  bool detached_for_window_hide = false;

  /**
   * Set when mosaic title-bar drag parks this pane. Settle must destroy and
   * recreate the Ogre QWidget/GLX (rebind alone left a blank/transparent view).
   */
  bool needs_gl_recreate = false;

  /**
   * Snapshot taken when entering top-down ortho; restored on return to 3D.
   */
  std::optional<rendering::ViewState> saved_3d_view_state;

  /**
   * @brief Returns the ViewController from the active render backend.
   * @return Non-owning controller, or @c nullptr if no window exists yet.
   */
  rendering::ViewController* viewController() const;
};

}  // namespace autoviz
