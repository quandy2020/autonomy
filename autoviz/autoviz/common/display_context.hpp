/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file display_context.hpp
 * @brief Context passed to Display / Tool plugins (RViz DisplayContext subset).
 *
 * Aggregates Autolink, TF, picking, selection, tools, view matrices, and
 * optional Ogre scene host so plugins avoid reaching into VisualizationManager
 * directly.
 *
 * @see VisualizationManager
 * @see display::Display
 * @see ToolContext
 */

#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include <QImage>
#include <QMatrix4x4>

#include "autoviz/integration/autolink_context.hpp"

namespace autoviz {
namespace rendering {
class OgreSceneHost;
class ViewController;
}

namespace transform {
class Buffer;
}

namespace common {

class FrameManager;
class HandlerManager;
class PickRegistry;
class SelectionManager;
class ToolManager;
class ViewManager;

/**
 * @class DisplayContext
 * @brief Shared services and per-frame view state for displays and tools.
 *
 * ## Lifetime
 *
 * Owned by @ref VisualizationManager; pointers inside are non-owning and
 * updated via @c syncDisplayContext() each update tick.
 */
class DisplayContext {
 public:
  /** Autolink runtime (node, clocks). */
  integration::AutolinkContext* autolink = nullptr;

  /** Shared TF buffer. */
  transform::Buffer* tf_buffer = nullptr;

  /** Fixed-frame / TF lookup facade. */
  FrameManager* frame_manager = nullptr;

  /** Per-frame pick metadata. */
  PickRegistry* pick_registry = nullptr;

  /** Pick-handle → selection handler map. */
  HandlerManager* handler_manager = nullptr;

  /** Global selection set. */
  SelectionManager* selection_manager = nullptr;

  /** Interactive tool manager. */
  ToolManager* tool_manager = nullptr;

  /** Saved-view / current-view config manager. */
  ViewManager* view_manager = nullptr;

  /**
   * Current viewport camera (for screen-space overlays such as LabelBubble).
   */
  rendering::ViewController* view_controller = nullptr;

  /** Viewport width in pixels. */
  int viewport_width = 1;

  /** Viewport height in pixels. */
  int viewport_height = 1;

  /** View matrix for the active camera (when @c has_view_matrices). */
  QMatrix4x4 view_matrix;

  /** Projection matrix for the active camera. */
  QMatrix4x4 projection_matrix;

  /** Whether @c view_matrix / @c projection_matrix are valid this frame. */
  bool has_view_matrices = false;

  /** Fixed frame id mirrored from VisualizationManager. */
  std::string fixed_frame;

  /** Request a viewport redraw. */
  std::function<void()> request_redraw;

  /**
   * Notifies UI that an Image display produced a new frame.
   * Arguments: display name, image.
   */
  std::function<void(const std::string& display_name, const QImage& image)>
      image_updated;

  /** Optional Ogre scene host when the Ogre backend is enabled. */
  rendering::OgreSceneHost* ogre_scene_host = nullptr;

  /** Active display name for pick-source tagging during @c draw(). */
  const std::string* active_display_name = nullptr;

  /** Active display type for pick-source tagging during @c draw(). */
  const std::string* active_display_type = nullptr;

  /** Active display visibility bit mask during @c draw(). */
  const uint32_t* active_display_visibility_bits = nullptr;

  /** Default RViz-style visibility bit for newly created displays. */
  uint32_t default_visibility_bit = 0x00000001u;

  /**
   * @brief Convenience: invokes @c request_redraw if set.
   */
  void queueRender();

  /**
   * @brief Monotonic frame counter advanced by the manager each update.
   * @return Current frame count.
   */
  uint64_t frameCount() const { return frame_count_; }

  /**
   * @brief Increments the frame counter (called once per update tick).
   */
  void incrementFrameCount() { ++frame_count_; }

 private:
  uint64_t frame_count_ = 0; /**< Frames since initialize. */
};

}  // namespace common
}  // namespace autoviz
