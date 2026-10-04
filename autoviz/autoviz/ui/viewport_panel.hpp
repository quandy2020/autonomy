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
 * @see ViewportFloatingToolbar
 * @see ViewportHudOverlay
 * @see rendering::RenderWindow
 * @see rendering::OgreRenderWindow
 */

#pragma once

#include <optional>
#include <string>

#include <QString>

#include "autoviz/rendering/view_controller.hpp"

class QGridLayout;
class QToolButton;
class QWidget;

namespace autoviz {

class PanelDockWidget;
class ViewportFloatingToolbar;
class ViewportHudOverlay;

namespace rendering {
class OgreRenderWindow;
class RenderWindow;
}

/**
 * @struct ViewportPanelEntry
 * @brief One 3D/2D viewport dock and its render window + overlays.
 *
 * Exactly one of @c gl_viewport / @c ogre_viewport is non-null after
 * @ref FrameViewport::createRenderWindowInEntry() (depending on backend).
 * @ref viewController() returns the controller from the active backend.
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
  QWidget* host = nullptr;          /**< Content host inside the dock. */
  QGridLayout* layout = nullptr;    /**< Layout stacking GL + overlays. */
  QWidget* widget = nullptr;        /**< Render window's QWidget surface. */
  rendering::RenderWindow* gl_viewport = nullptr;       /**< OpenGL backend. */
  rendering::OgreRenderWindow* ogre_viewport = nullptr; /**< Ogre backend. */
  ViewportFloatingToolbar* floating_toolbar = nullptr;  /**< Right-edge tools. */
  ViewportHudOverlay* hud_overlay = nullptr;            /**< Top-left HUD. */
  QToolButton* expand_button = nullptr;   /**< Title-bar expand toggle. */
  QToolButton* settings_button = nullptr; /**< Title-bar settings toggle. */

  /**
   * Per-split tool (Select / Measure / Interact); not shared across 3D panels.
   */
  std::string local_tool_id = "Interact";

  /**
   * Snapshot taken when entering 2D via floating toolbar; restored on 3D.
   */
  std::optional<rendering::ViewState> saved_3d_view_state;

  /**
   * @brief Returns the ViewController from the active render backend.
   * @return Non-owning controller, or @c nullptr if no window exists yet.
   */
  rendering::ViewController* viewController() const;
};

}  // namespace autoviz
