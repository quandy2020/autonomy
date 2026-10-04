/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file tool.hpp
 * @brief RViz-style interactive tools and the per-viewport @ref ToolContext.
 *
 * Tools receive mouse/wheel events, optional draw overlays, and property maps.
 * @ref ToolContext aggregates viewport, picking, selection, and Autolink
 * pointers needed during interaction.
 *
 * @see ToolManager
 * @see ToolRegistry
 * @see ViewPicker
 */

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <QCursor>
#include <QImage>
#include <QString>
#include <QVector3D>

#include "autolink/node/node.hpp"
#include "autoviz/common/display_context.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/common/pick_handle.hpp"
#include "autoviz/common/selection_handler.hpp"
#include "autoviz/common/pick_registry.hpp"
#include "autoviz/common/selection.hpp"
#include "autoviz/common/selection_manager.hpp"

class QMouseEvent;
class QWheelEvent;

namespace autoviz {
namespace display {
class InteractiveMarkerRegistry;
}
namespace rendering {
class SceneOverlay;
class ViewController;
}

namespace common {

/**
 * @struct ToolContext
 * @brief Non-owning services bundle passed to the active @ref Tool.
 *
 * Constructed/updated by the viewport host each frame or on tool activation.
 * Pointers remain owned by VisualizationManager / FrameViewport.
 */
struct ToolContext {
  /** Live camera controller for the active 3D panel. */
  rendering::ViewController* view_controller = nullptr;

  /** Immediate-mode overlay for tool drawings (lines, markers). */
  rendering::SceneOverlay* scene_overlay = nullptr;

  /** Identifies the 3D panel (dock @c objectName) for per-split tool state. */
  std::string viewport_key;

  /** Viewport width in pixels (≥ 1). */
  int viewport_width = 1;

  /** Viewport height in pixels (≥ 1). */
  int viewport_height = 1;

  /** @c true when hardware GPU was detected; enables depth-buffer picking. */
  bool gpu_picking_enabled = false;

  /**
   * Optional GPU depth pick: screen pixel → fixed-frame world point.
   * Returns @c false on miss.
   */
  std::function<bool(int pixel_x, int pixel_y, QVector3D* world)>
      gpu_depth_pick;

  /**
   * Optional GPU color-id pick: screen pixel → @ref PickHandle.
   */
  std::function<common::PickHandle(int pixel_x, int pixel_y)> gpu_pick_id_read;

  /** Per-frame pick metadata registry. */
  common::PickRegistry* pick_registry = nullptr;

  /** Pick-handle → @ref SelectionHandler map. */
  common::HandlerManager* handler_manager = nullptr;

  /** Autolink node for tools that publish (e.g. Publish Point). */
  std::shared_ptr<::autolink::Node> autolink_node;

  /** Fixed frame id mirrored from VisualizationManager. */
  std::string fixed_frame = "map";

  /** Shared display context (TF, pick, view matrices). */
  DisplayContext* display_context = nullptr;

  /** Global selection set. */
  SelectionManager* selection_manager = nullptr;

  /** Interactive marker interaction registry. */
  autoviz::display::InteractiveMarkerRegistry* interactive_markers = nullptr;

  /** Request a viewport redraw. */
  std::function<void()> request_redraw;

  /** Refresh @c display_context->ogre_scene_host before Ogre tool drawing. */
  std::function<void()> sync_ogre_host;

  /** Update status-bar text for the active tool. */
  std::function<void(const QString&)> set_status;

  /** Notify UI that the selection list changed. */
  std::function<void(const std::vector<SelectionEntry>&)> selections_changed;

  /**
   * RViz @c Tool::Finished — switch back to Move Camera after one-shot tools.
   */
  std::function<void()> revert_to_default_tool;
};

/**
 * @class Tool
 * @brief Base class for interactive viewport tools (Move Camera, Focus, …).
 *
 * Subclasses override mouse handlers and optional @ref onDraw. Returning
 * @c true from a mouse handler consumes the event (skips default camera drag).
 */
class Tool {
 public:
  virtual ~Tool() = default;

  /**
   * @brief Stable tool id used in session config and toolbars.
   * @return Id string (e.g. @c "Interact").
   */
  virtual std::string id() const = 0;

  /**
   * @brief Human-readable label for UI.
   * @return Localized label.
   */
  virtual QString label() const = 0;

  /**
   * @brief Called when the tool becomes active.
   * @param context Non-owning services bundle (stored for later use).
   */
  virtual void activate(ToolContext* context) { context_ = context; }

  /**
   * @brief Refresh context pointers without resetting interaction state.
   *
   * Used when the viewport is resized or the view controller is replaced.
   *
   * @param context Updated services bundle.
   */
  virtual void updateContext(ToolContext* context) { context_ = context; }

  /**
   * @brief Called when another tool becomes active; clears @c context_.
   */
  virtual void deactivate() { context_ = nullptr; }

  /**
   * @brief Cursor shown while this tool is active.
   * @return Current cursor.
   */
  const QCursor& cursor() const { return cursor_; }

  /**
   * @brief Sets the cursor for this tool.
   * @param cursor Qt cursor to show while active.
   */
  void setCursor(const QCursor& cursor) { cursor_ = cursor; }

  /**
   * @brief Handles a mouse press.
   * @param event Qt mouse event.
   * @return @c true if consumed (skip default camera drag).
   */
  virtual bool mousePressEvent(QMouseEvent* event);

  /**
   * @brief Handles a mouse move.
   * @param event Qt mouse event.
   * @return @c true if consumed.
   */
  virtual bool mouseMoveEvent(QMouseEvent* event);

  /**
   * @brief Handles a mouse release.
   * @param event Qt mouse event.
   * @return @c true if consumed.
   */
  virtual bool mouseReleaseEvent(QMouseEvent* event);

  /**
   * @brief Handles a mouse wheel event.
   * @param event Qt wheel event.
   * @return @c true if consumed.
   */
  virtual bool wheelEvent(QWheelEvent* event);

  /**
   * @brief Optional per-frame overlay drawing into @p scene.
   * @param scene Shared scene overlay (may be unused by the tool).
   */
  virtual void onDraw(rendering::SceneOverlay& /*scene*/) {}

  /**
   * @brief Drops interaction state for one Split 3D panel.
   * @param viewport_key Dock / panel key matching @ref ToolContext::viewport_key.
   */
  virtual void clearViewportSession(const std::string& /*viewport_key*/) {}

  /**
   * @brief Short status-bar text while the tool is active.
   * @return Status string (empty by default).
   */
  virtual QString statusText() const { return {}; }

  /**
   * @brief RViz @c Tool::getShortcutKey — letter that activates this tool.
   * @return Shortcut character, or @c '\\0' if none.
   */
  virtual char shortcutKey() const { return '\0'; }

  /**
   * @brief Property schema for the Tools property panel.
   * @return Spec list (empty if the tool has no editable properties).
   */
  virtual std::vector<DisplayPropertySpec> propertySpecs() const { return {}; }

  /**
   * @brief Replaces the entire property map.
   * @param properties New key/value map.
   */
  void setProperties(const DisplayPropertyMap& properties);

  /**
   * @brief Returns the current property map.
   * @return Const reference to stored properties.
   */
  const DisplayPropertyMap& properties() const { return properties_; }

  /**
   * @brief Looks up a property with a default fallback.
   *
   * @param key Property key.
   * @param default_value Returned when @p key is absent.
   * @return Stored value or @p default_value.
   */
  std::string propertyValue(const std::string& key,
                            const std::string& default_value) const;

  /**
   * @brief Sets a single property value.
   *
   * @param key Property key.
   * @param value New string value.
   */
  void setPropertyValue(const std::string& key, const std::string& value);

 protected:
  /**
   * @brief Accesses the active tool context.
   * @return Context pointer, or @c nullptr when inactive.
   */
  ToolContext* context() const { return context_; }

 private:
  ToolContext* context_ = nullptr;                 /**< Active context. */
  DisplayPropertyMap properties_;                  /**< Tool settings. */
  QCursor cursor_ = Qt::ArrowCursor;               /**< Active cursor. */
};

}  // namespace common
}  // namespace autoviz
