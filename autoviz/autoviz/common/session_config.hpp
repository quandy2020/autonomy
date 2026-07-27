/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "autoviz/common/display_property.hpp"

namespace autoviz {
namespace display {
class Display;
}

namespace common {

struct DisplayConfig {
  std::string type;
  std::string name;
  std::string channel;
  bool enabled = true;
  DisplayPropertyMap properties;
  std::vector<DisplayConfig> children;
};

struct SavedViewConfig {
  std::string name;
  std::string type = "Orbit";
  float near_clip_distance = 0.01f;
  bool invert_z_axis = false;
  float yaw = 0.785398f;
  float pitch = 0.785398f;
  float distance = 10.f;
  float target_x = 0.f;
  float target_y = 0.f;
  float target_z = 0.f;
  float focal_shape_size = 0.05f;
  bool focal_shape_fixed_size = true;
  float fps_position_x = 0.f;
  float fps_position_y = 2.f;
  float fps_position_z = 8.f;
  float fps_yaw = 3.14f;
  float fps_pitch = 0.f;
  /** Empty means "<Fixed Frame>" in the Views panel. */
  std::string target_frame;
};

struct ToolConfig {
  std::string id;
  DisplayPropertyMap properties;
};

struct PanelLayoutConfig {
  std::string object_name;
  bool collapsed = false;
};

struct SessionConfig {
  std::string fixed_frame = "map";
  bool show_grid = true;
  int frame_rate = 30;
  std::string background_color = "48;48;48";
  std::string view_controller = "Orbit";
  std::string render_backend = "OpenGL";
  std::vector<std::string> toolbar_tools;
  std::string active_tool = "Interact";
  std::vector<DisplayConfig> displays;
  std::vector<SavedViewConfig> views;
  std::vector<ToolConfig> tools;
  std::string window_state_b64;
  std::string window_geometry_b64;
  bool hide_left_dock = false;
  bool hide_right_dock = false;
  std::vector<PanelLayoutConfig> panel_layouts;
  std::vector<std::string> visible_panels;
  std::string transformer_id = "autoviz/AutolinkTf";
  int window_x = -1;
  int window_y = -1;
  int window_width = -1;
  int window_height = -1;
  SavedViewConfig current_view;
  bool has_current_view = false;
};

class SessionConfigIO {
 public:
  static bool load(const std::string& path, SessionConfig* config);
  static bool save(const std::string& path, const SessionConfig& config);
  static SessionConfig defaultConfig();
};

}  // namespace common
}  // namespace autoviz
