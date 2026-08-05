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

struct PlotSeriesPersistConfig {
  std::string channel;
  std::string field_path;
  std::string x_field_path;
  std::string custom_timestamp_path;
  std::string label;
  std::string color = "#4e98e2";
  std::string line_size = "auto";
  bool show_line = true;
  int timestamp_mode = 0;
  bool enabled = true;
};

struct PlotPanelPersistConfig {
  std::string object_name;
  std::string title = "Plot";
  int x_axis_mode = 0;
  int message_path_mode = 1;
  bool sync_with_other_plots = false;
  bool show_legend_values = true;
  bool settings_visible = false;
  int settings_width = 300;
  bool lock_axis_scales = false;
  double x_window_sec = 30.0;
  std::vector<PlotSeriesPersistConfig> series;
};

struct ImageOverlayPersistConfig {
  std::string channel;
  double opacity = 0.5;
  int blend_mode = 0;
  int pixel_alpha = 0;
  bool enabled = true;
};

struct VariablePersistConfig {
  std::string name;
  std::string type = "string";
  std::string value;
};

struct ImagePanelPersistConfig {
  std::string object_name;
  std::string title = "Image";
  std::string image_channel;
  std::string calibration_channel;
  bool strict_time_sync = false;
  bool flip_horizontal = false;
  bool flip_vertical = false;
  int rotation = 0;
  int color_mode = 0;
  double color_min = 0.0;
  double color_max = 255.0;
  std::vector<ImageOverlayPersistConfig> overlays;
  std::vector<std::string> annotation_channels;
  std::vector<std::string> marker_channels;
  std::string background_color = "#000000";
  double label_scale = 1.0;
  std::string click_publish_channel;
  std::string hover_publish_channel;
  bool enable_undistort = false;
  bool settings_visible = false;
};

struct StateTransitionMappingPersistConfig {
  int kind = 0;
  std::string match_value;
  double range_min = 0.0;
  double range_max = 0.0;
  std::string label;
  std::string color = "#787882";
};

struct StateTransitionSeriesPersistConfig {
  std::string channel;
  std::string field_path;
  std::string custom_timestamp_path;
  std::string label;
  int timestamp_mode = 0;
  bool enabled = true;
  std::vector<StateTransitionMappingPersistConfig> mappings;
};

struct StateTransitionPanelPersistConfig {
  std::string object_name;
  std::string title = "State Transitions";
  int x_axis_mode = 0;
  double x_window_sec = 30.0;
  double fixed_min_time = 0.0;
  double fixed_max_time = 60.0;
  bool settings_visible = false;
  std::vector<StateTransitionSeriesPersistConfig> series;
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
  std::string main_panel_state_b64;
  std::string window_geometry_b64;
  bool hide_left_dock = false;
  bool hide_right_dock = false;
  std::vector<PanelLayoutConfig> panel_layouts;
  std::vector<std::string> visible_panels;
  std::vector<PlotPanelPersistConfig> plot_panels;
  std::vector<ImagePanelPersistConfig> image_panels;
  std::vector<StateTransitionPanelPersistConfig> state_transition_panels;
  std::vector<VariablePersistConfig> variables;
  bool plot_settings_visible = true;
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
