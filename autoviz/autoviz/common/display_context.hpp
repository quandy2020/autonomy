/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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

/** rviz_common::DisplayContext — context passed to Display / Tool plugins. */
class DisplayContext {
 public:
  integration::AutolinkContext* autolink = nullptr;
  transform::Buffer* tf_buffer = nullptr;
  FrameManager* frame_manager = nullptr;
  PickRegistry* pick_registry = nullptr;
  HandlerManager* handler_manager = nullptr;
  SelectionManager* selection_manager = nullptr;
  ToolManager* tool_manager = nullptr;
  ViewManager* view_manager = nullptr;

  /** 当前视口与相机，供屏幕空间 overlay（LabelBubble 等）使用。 */
  rendering::ViewController* view_controller = nullptr;
  int viewport_width = 1;
  int viewport_height = 1;
  QMatrix4x4 view_matrix;
  QMatrix4x4 projection_matrix;
  bool has_view_matrices = false;

  std::string fixed_frame;
  std::function<void()> request_redraw;
  std::function<void(const std::string& display_name, const QImage& image)>
      image_updated;

#ifdef AUTOVIZ_USE_OGRE
  rendering::OgreSceneHost* ogre_scene_host = nullptr;
#endif

  /** Active display for pick-source tagging during draw(). */
  const std::string* active_display_name = nullptr;
  const std::string* active_display_type = nullptr;
  const uint32_t* active_display_visibility_bits = nullptr;

  /** Default rviz-style visibility bit for new displays. */
  uint32_t default_visibility_bit = 0x00000001u;

  void queueRender();
  uint64_t frameCount() const { return frame_count_; }
  void incrementFrameCount() { ++frame_count_; }

 private:
  uint64_t frame_count_ = 0;
};

}  // namespace common
}  // namespace autoviz
