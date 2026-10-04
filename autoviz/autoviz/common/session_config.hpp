/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file session_config.hpp
 * @brief Strongly typed session / layout persistence structures for Autoviz.
 *
 * @ref SessionConfig is the in-memory representation of a @c .autoviz (or
 * imported @c .rviz) session: displays, views, tools, window layout, and
 * panel persist state. @ref SessionConfigIO loads and saves via YAML /
 * @ref Config bridges.
 *
 * @see config_session.hpp
 * @see VisualizationManager::loadSession
 * @see YamlConfigReader
 */

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

/**
 * @struct DisplayConfig
 * @brief Persistable description of one display (and optional Group children).
 */
struct DisplayConfig {
  /** Display type id (e.g. @c "PointCloud2", @c "Group"). */
  std::string type;

  /** Instance name shown in the Displays tree. */
  std::string name;

  /** Subscribed channel name (empty for non-subscribing displays). */
  std::string channel;

  /** Whether the display is enabled at load time. */
  bool enabled = true;

  /** Key/value property map. */
  DisplayPropertyMap properties;

  /** Nested child displays (used by Group). */
  std::vector<DisplayConfig> children;
};

/**
 * @struct SavedViewConfig
 * @brief Persistable viewpoint bookmark / current-view snapshot.
 *
 * Fields cover Orbit-family and FPS controllers; unused fields for a given
 * type are ignored when applying @ref ToViewState.
 *
 * @see view_state_io.hpp
 * @see ViewManager
 */
struct SavedViewConfig {
  /** Bookmark display name (e.g. @c "View 1"). */
  std::string name;

  /** View-controller type id (default @c "Orbit"). */
  std::string type = "Orbit";

  /** Near clip plane distance. */
  float near_clip_distance = 0.01f;

  /** Invert Z axis flag. */
  bool invert_z_axis = false;

  /** Orbit / FPS yaw (radians). */
  float yaw = 0.785398f;

  /** Orbit / FPS pitch (radians). */
  float pitch = 0.785398f;

  /** Orbit distance (or Ortho scale when remapped). */
  float distance = 10.f;

  /** Orbit focal point X. */
  float target_x = 0.f;

  /** Orbit focal point Y. */
  float target_y = 0.f;

  /** Orbit focal point Z. */
  float target_z = 0.f;

  /** Focal shape marker size. */
  float focal_shape_size = 0.05f;

  /** Whether focal shape ignores distance scaling. */
  bool focal_shape_fixed_size = true;

  /** FPS eye position X. */
  float fps_position_x = 0.f;

  /** FPS eye position Y. */
  float fps_position_y = 2.f;

  /** FPS eye position Z. */
  float fps_position_z = 8.f;

  /** FPS yaw (radians). */
  float fps_yaw = 3.14f;

  /** FPS pitch (radians). */
  float fps_pitch = 0.f;

  /**
   * Target / follow TF frame.
   * Empty means @c "<Fixed Frame>" in the Views panel.
   */
  std::string target_frame;
};

/**
 * @struct ToolConfig
 * @brief Persistable property map for one tool id.
 */
struct ToolConfig {
  /** Tool id (e.g. @c "Interact"). */
  std::string id;

  /** Tool property key/values. */
  DisplayPropertyMap properties;
};

/**
 * @struct PanelLayoutConfig
 * @brief Collapse state for a named dock / panel object.
 */
struct PanelLayoutConfig {
  /** Qt @c objectName of the panel. */
  std::string object_name;

  /** Whether the panel chrome is collapsed. */
  bool collapsed = false;
};

/**
 * @struct PlotSeriesPersistConfig
 * @brief One series inside a Plot panel.
 */
struct PlotSeriesPersistConfig {
  /** Source channel name. */
  std::string channel;

  /** Y (or primary) field path within the message. */
  std::string field_path;

  /** Optional X field path. */
  std::string x_field_path;

  /** Optional custom timestamp field path. */
  std::string custom_timestamp_path;

  /** Series legend label. */
  std::string label;

  /** Series color (CSS hex, default blue). */
  std::string color = "#4e98e2";

  /** Line width mode / size (@c "auto" or numeric string). */
  std::string line_size = "auto";

  /** Whether to draw the connecting line. */
  bool show_line = true;

  /** Plot against the right Y-axis when true. */
  bool use_right_y = false;

  /** Binary combine op with secondary operand (panel enum as int). */
  int binary_op = 0;

  /** Secondary channel; empty means same as primary. */
  std::string secondary_channel;

  /** Secondary Y field path. */
  std::string secondary_field_path;

  /** Max |Δt| seconds for cross-channel nearest-neighbor match. */
  double binary_max_dt_sec = 0.1;

  /** Timestamp interpretation mode (panel-specific enum as int). */
  int timestamp_mode = 0;

  /** Whether the series is enabled. */
  bool enabled = true;
};

/**
 * @struct PlotPanelPersistConfig
 * @brief Persist state for one Plot panel instance.
 */
struct PlotPanelPersistConfig {
  /** Qt @c objectName. */
  std::string object_name;

  /** Window / tab title. */
  std::string title = "Plot";

  /** X-axis mode (panel-specific enum as int). */
  int x_axis_mode = 0;

  /** Message-path mode (panel-specific enum as int). */
  int message_path_mode = 1;

  /** Sync X range with other plots. */
  bool sync_with_other_plots = false;

  /** Show numeric values in the legend. */
  bool show_legend_values = true;

  /** Whether the settings sidebar is visible. */
  bool settings_visible = false;

  /** Settings sidebar width in pixels. */
  int settings_width = 300;

  /** Lock axis scales against auto-fit. */
  bool lock_axis_scales = false;

  /** When false, use fixed @ref y_min / @ref y_max. */
  bool y_auto_scale = true;

  /** Fixed Y minimum (when @ref y_auto_scale is false). */
  double y_min = 0.0;

  /** Fixed Y maximum (when @ref y_auto_scale is false). */
  double y_max = 1.0;

  /** When false, use fixed @ref y_min_right / @ref y_max_right. */
  bool y_auto_scale_right = true;

  /** Fixed right Y minimum. */
  double y_min_right = 0.0;

  /** Fixed right Y maximum. */
  double y_max_right = 1.0;

  /** Draw chart grid lines. */
  bool show_grid = true;

  /** Draw a horizontal reference line. */
  bool show_reference_y = false;

  /** Y value for the reference line. */
  double reference_y = 0.0;

  /** Rolling X window length in seconds. */
  double x_window_sec = 30.0;

  /** Series list. */
  std::vector<PlotSeriesPersistConfig> series;
};

/**
 * @struct ImageOverlayPersistConfig
 * @brief One overlay layer on an Image panel.
 */
struct ImageOverlayPersistConfig {
  /** Overlay channel name. */
  std::string channel;

  /** Overlay opacity (0–1). */
  double opacity = 0.5;

  /** Blend mode (panel-specific enum as int). */
  int blend_mode = 0;

  /** Pixel alpha handling mode. */
  int pixel_alpha = 0;

  /** Whether the overlay is enabled. */
  bool enabled = true;
};

/**
 * @struct VariablePersistConfig
 * @brief Named session variable (string/typed value for scripting / panels).
 */
struct VariablePersistConfig {
  /** Variable name. */
  std::string name;

  /** Type tag (default @c "string"). */
  std::string type = "string";

  /** Serialized value. */
  std::string value;
};

/**
 * @struct ChannelsBrowserPersistConfig
 * @brief Persist state for the Channels sidebar (filter / chips / probe / expand).
 *
 * Single instance — there is only one Channel Browser dock.
 */
struct ChannelsBrowserPersistConfig {
  /** Substring filter text. */
  std::string filter_text;

  /** Type chip: 0=All, 1=Numeric, 2=Image, 3=Geo, 4=TF. */
  int type_filter = 0;

  /** Lightweight stats probe enabled. */
  bool probe_enabled = true;

  /** Fully-qualified channel names that should be expanded. */
  std::vector<std::string> expanded_channels;
};

/**
 * @struct RawMessagesPersistConfig
 * @brief Persist state for the Raw Messages (Messages) panel.
 *
 * Single instance — one ChannelsDock hosts RawMessagesPanel.
 */
struct RawMessagesPersistConfig {
  /** Selected Autolink channel (empty = none). */
  std::string channel;

  /** Optional message-path filter. */
  std::string message_path;

  /** Freeze live tree updates when true. */
  bool freeze = false;

  /** Highlight leaves that changed vs previous frame. */
  bool diff_highlight = true;
};

/**
 * @struct TablePanelPersistConfig
 * @brief Persist state for one Table panel instance (array-of-messages view).
 */
struct TablePanelPersistConfig {
  /** Qt @c objectName. */
  std::string object_name;

  /** Window / tab title. */
  std::string title = "Table";

  /** Source Autolink channel. */
  std::string channel;

  /** Path to a repeated protobuf field. */
  std::string field_path;

  /** Case-insensitive substring that hides non-matching rows. */
  std::string row_filter;
};

/**
 * @struct ChannelGraphPanelPersistConfig
 * @brief Persist state for one Channel Graph panel instance.
 */
struct ChannelGraphPanelPersistConfig {
  /** Qt @c objectName. */
  std::string object_name;

  bool show_services = true;
  bool show_channels = true;
  bool auto_refresh = true;
  bool neighborhood_mode = true;
  bool show_edge_labels = false;
  bool quiet_mode = false;
  bool hide_leaf_channels = false;
  bool hide_dead_end_channels = false;
  bool probe_enabled = false;

  /** @ref channel_graph::VertexArrangeMode as int. */
  int channel_arrange = 1;
  /** @ref channel_graph::VertexArrangeMode as int. */
  int service_arrange = 1;

  std::string filter;
  std::string prefix_filter;
};

/**
 * @struct TfTreePanelPersistConfig
 * @brief Persist state for one Transform Tree panel instance.
 */
struct TfTreePanelPersistConfig {
  /** Qt @c objectName. */
  std::string object_name;

  /** Substring filter for frame / parent ids. */
  std::string filter;

  /** Active tab: 0 = Tree, 1 = Graph. */
  int tab_index = 0;

  /** When true, only show frames older than @ref max_age_sec. */
  bool stale_only = false;

  /** Age threshold in seconds used by @ref stale_only (default 1.0). */
  double max_age_sec = 1.0;
};

/**
 * @struct ImagePanelPersistConfig
 * @brief Persist state for one Image panel instance.
 */
struct ImagePanelPersistConfig {
  /** Qt @c objectName. */
  std::string object_name;

  /** Window / tab title. */
  std::string title = "Image";

  /** Primary image channel. */
  std::string image_channel;

  /** Optional camera info / calibration channel. */
  std::string calibration_channel;

  /** Require exact time sync between image and overlays. */
  bool strict_time_sync = false;

  /** Flip image horizontally. */
  bool flip_horizontal = false;

  /** Flip image vertically. */
  bool flip_vertical = false;

  /** Rotation in degrees (0/90/180/270). */
  int rotation = 0;

  /** Color mapping mode. */
  int color_mode = 0;

  /** Color map minimum. */
  double color_min = 0.0;

  /** Color map maximum. */
  double color_max = 255.0;

  /** Overlay layers. */
  std::vector<ImageOverlayPersistConfig> overlays;

  /** Annotation channel names. */
  std::vector<std::string> annotation_channels;

  /** Marker channel names. */
  std::vector<std::string> marker_channels;

  /** PointCloud2 channel names projected into the image. */
  std::vector<std::string> point_cloud_channels;

  /** Viewer background color (CSS hex). */
  std::string background_color = "#000000";

  /** Annotation label scale factor. */
  double label_scale = 1.0;

  /** Channel for click-to-publish. */
  std::string click_publish_channel;

  /** Channel for hover-to-publish. */
  std::string hover_publish_channel;

  /** Enable undistortion when calibration is available. */
  bool enable_undistort = false;

  /** Whether the settings sidebar is visible. */
  bool settings_visible = false;
};

/**
 * @struct PublishPresetPersistConfig
 * @brief Saved preset for the Publish panel.
 */
struct PublishPresetPersistConfig {
  /** Preset name. */
  std::string name;

  /** Target channel. */
  std::string channel;

  /** Message type name. */
  std::string message_type;

  /** Message body as JSON. */
  std::string message_json;

  /** Loop publish while active. */
  bool loop_publish = false;

  /** Publish rate in Hz when looping. */
  double publish_rate_hz = 1.0;

  /** Custom send-button label. */
  std::string button_label;

  /** Custom send-button tooltip. */
  std::string button_tooltip;

  /** Custom send-button color. */
  std::string button_color;
};

/**
 * @struct PublishEntryPersistConfig
 * @brief One concurrent publisher entry inside a Publish panel.
 */
struct PublishEntryPersistConfig {
  /** Stable entry id. */
  std::string id;

  /** Target channel. */
  std::string channel;

  /** Message type name. */
  std::string message_type;

  /** Publish rate in Hz. */
  double publish_rate_hz = 1.0;

  /** Message body as JSON. */
  std::string message_json;

  /** Whether this entry is currently publishing. */
  bool publishing = true;
};

/**
 * @struct PublishPanelPersistConfig
 * @brief Persist state for one Publish panel instance.
 */
struct PublishPanelPersistConfig {
  /** Qt @c objectName. */
  std::string object_name;

  /** Window / tab title. */
  std::string title = "Publish";

  /** Default / primary channel. */
  std::string channel;

  /** Default message type. */
  std::string message_type;

  /** Default message JSON body. */
  std::string message_json;

  /** Whether the JSON editor is in editing mode. */
  bool editing_mode = false;

  /** Loop publish for the primary publisher. */
  bool loop_publish = false;

  /** Primary publish rate in Hz. */
  double publish_rate_hz = 1.0;

  /** Send-button label. */
  std::string button_label = "Send";

  /** Send-button tooltip. */
  std::string button_tooltip;

  /** Send-button color. */
  std::string button_color;

  /** Whether the settings sidebar is visible. */
  bool settings_visible = false;

  /** Name of the currently active preset (if any). */
  std::string active_preset_name;

  /** Saved presets. */
  std::vector<PublishPresetPersistConfig> saved_presets;

  /** User-defined channel names. */
  std::vector<std::string> custom_channels;

  /** Concurrent publisher entries. */
  std::vector<PublishEntryPersistConfig> publishers;

  /** Index of the selected publisher (−1 if none). */
  int selected_publisher_index = -1;
};

/**
 * @struct ServicePanelPersistConfig
 * @brief Persist state for one Service Call panel instance.
 *
 * Last response JSON is not stored; it is the result of a single call.
 */
struct ServicePanelPersistConfig {
  /** Qt @c objectName. */
  std::string object_name;

  /** Window / tab title. */
  std::string title = "Service Call";

  /** Fully-qualified service name. */
  std::string service_name;

  /** Protobuf request type. */
  std::string request_type;

  /** Protobuf response type. */
  std::string response_type;

  /** Request body as JSON. */
  std::string request_json = "{}";

  /** When true, show advanced type / JSON UI. */
  bool editing_mode = false;

  /** Request above response when true. */
  bool vertical_layout = true;

  /** RPC timeout in seconds. */
  int timeout_sec = 5;

  /** Call-button label. */
  std::string button_label = "Call";

  /** Call-button tooltip. */
  std::string button_tooltip;

  /** Call-button color (#rrggbb), empty = theme default. */
  std::string button_color;

  /** Whether the settings sidebar is visible. */
  bool settings_visible = false;
};

/**
 * @struct TeleopButtonPersistConfig
 * @brief One discrete teleop button stored in a session.
 */
struct TeleopButtonPersistConfig {
  /** @ref teleop::TeleopTwistField as int. */
  int field = 0;
  double value = 0.0;
};

/**
 * @struct TeleopPanelPersistConfig
 * @brief Persist state for one Teleop panel instance.
 */
struct TeleopPanelPersistConfig {
  /** Qt @c objectName. */
  std::string object_name;

  std::string title = "Teleop";
  std::string topic = "/cmd_vel";
  double publish_rate_hz = 1.0;
  bool stop_on_release = true;
  bool smart_teleop_enabled = false;
  /** @ref teleop::TeleopStickMode as int. */
  int stick_mode = 0;
  double max_linear_speed = 0.5;
  double max_angular_speed = 0.5;
  TeleopButtonPersistConfig up;
  TeleopButtonPersistConfig down;
  TeleopButtonPersistConfig left;
  TeleopButtonPersistConfig right;
  TeleopButtonPersistConfig stop;
  bool settings_visible = false;
};

/**
 * @struct MapOverlayPersistConfig
 * @brief One tile overlay stored with a Map panel.
 */
struct MapOverlayPersistConfig {
  std::string name;
  std::string tile_url_template;
  double opacity = 0.5;
  bool enabled = true;
};

/**
 * @struct MapGeoJsonPersistConfig
 * @brief One GeoJSON file stored with a Map panel.
 */
struct MapGeoJsonPersistConfig {
  std::string path;
  bool visible = true;
};

/**
 * @struct MapPlanPointPersistConfig
 * @brief One mission vertex stored with a Map panel.
 */
struct MapPlanPointPersistConfig {
  double latitude = 0;
  double longitude = 0;
};

/**
 * @struct MapTopicLayerPersistConfig
 * @brief One geo topic layer stored with a Map panel.
 */
struct MapTopicLayerPersistConfig {
  std::string channel;
  /** @ref map::MapPointStyle as int. */
  int point_style = 0;
  bool show_heading = true;
  bool show_velocity = false;
  double point_size = 8.0;
  /** @ref map::MapTimeRange as int. */
  int time_range = 0;
  double time_range_seconds = 30.0;
  double layer_opacity = 1.0;
  /** #rrggbb, empty = unset. */
  std::string color;
  bool enabled = true;
};

/**
 * @struct MapPanelPersistConfig
 * @brief Persist state for one Map panel instance.
 */
struct MapPanelPersistConfig {
  /** Qt @c objectName. */
  std::string object_name;

  std::string title = "Map";
  /** @ref map::MapBaseLayer as int. */
  int base_layer = 0;
  std::string custom_tile_url;
  std::string follow_channel;
  std::string gcs_channel;
  /** @ref map::MapDistanceUnit as int. */
  int distance_unit = 0;
  double center_latitude = 39.9042;
  double center_longitude = 116.4074;
  double zoom = 14.0;
  std::vector<MapOverlayPersistConfig> overlay_layers;
  std::vector<MapTopicLayerPersistConfig> topic_layers;
  std::vector<MapGeoJsonPersistConfig> geojson_sources;
  std::vector<MapPlanPointPersistConfig> waypoints;
  std::vector<MapPlanPointPersistConfig> geofence;
  std::vector<MapPlanPointPersistConfig> rally_points;
  /** @ref map::MapEditTool as int. */
  int edit_tool = 0;
  double survey_spacing_m = 40.0;
  double corridor_width_m = 50.0;
  double structure_radius_m = 80.0;
  bool settings_visible = false;
};

/**
 * @struct SessionConfig
 * @brief Full Autoviz session: global options, displays, views, tools, panels.
 *
 * Produced/consumed by @ref SessionConfigIO and applied by
 * @ref VisualizationManager::applySession.
 */
struct SessionConfig {
  /** Fixed TF frame (default @c "map"). */
  std::string fixed_frame = "map";

  /** Target render frame rate. */
  int frame_rate = 30;

  /** @ref TimeSyncMode as int (0 off, 1 exact, 2 approximate). */
  int time_sync_mode = 0;

  /** Channel or clock used when @ref time_sync_mode is not off. */
  std::string time_sync_source;

  /** Background color as @c "R;G;B". */
  std::string background_color = "48;48;48";

  /** Active view-controller type name. */
  std::string view_controller = "Orbit";

  /** Render backend name (@c "OpenGL", @c "Ogre", …). */
  std::string render_backend = "Ogre";

  /** Tool ids shown on the toolbar. */
  std::vector<std::string> toolbar_tools;

  /** Active tool id at load time. */
  std::string active_tool = "Interact";

  /** Display tree (top-level, may include Groups). */
  std::vector<DisplayConfig> displays;

  /** Saved viewpoint bookmarks. */
  std::vector<SavedViewConfig> views;

  /** Per-tool property maps. */
  std::vector<ToolConfig> tools;

  /** Main window Qt state (base64). */
  std::string window_state_b64;

  /** Central panel Qt state (base64). */
  std::string main_panel_state_b64;

  /** Main window geometry (base64). */
  std::string window_geometry_b64;

  /** Hide the left dock strip. */
  bool hide_left_dock = false;

  /** Hide the right dock strip. */
  bool hide_right_dock = false;

  /** Per-panel collapse state. */
  std::vector<PanelLayoutConfig> panel_layouts;

  /** Names of panels that should be visible. */
  std::vector<std::string> visible_panels;

  /** Plot panel persist records. */
  std::vector<PlotPanelPersistConfig> plot_panels;

  /** Image panel persist records. */
  std::vector<ImagePanelPersistConfig> image_panels;

  /** Publish panel persist records. */
  std::vector<PublishPanelPersistConfig> publish_panels;

  /** Service Call panel persist records. */
  std::vector<ServicePanelPersistConfig> service_panels;

  /** Teleop panel persist records. */
  std::vector<TeleopPanelPersistConfig> teleop_panels;

  /** Map panel persist records. */
  std::vector<MapPanelPersistConfig> map_panels;

  /** Channels sidebar persist record. */
  ChannelsBrowserPersistConfig channels_browser;

  /** Raw Messages panel persist record. */
  RawMessagesPersistConfig raw_messages;

  /** Table panel persist records. */
  std::vector<TablePanelPersistConfig> table_panels;

  /** Channel Graph panel persist records. */
  std::vector<ChannelGraphPanelPersistConfig> channel_graph_panels;

  /** Transform Tree panel persist records. */
  std::vector<TfTreePanelPersistConfig> tf_tree_panels;

  /** Session variables. */
  std::vector<VariablePersistConfig> variables;

  /** Global plot-settings visibility flag. */
  bool plot_settings_visible = true;

  /** Active FrameTransformer class id. */
  std::string transformer_id = "autoviz/AutolinkTf";

  /** Window X position (−1 = default). */
  int window_x = -1;

  /** Window Y position (−1 = default). */
  int window_y = -1;

  /** Window width (−1 = default). */
  int window_width = -1;

  /** Window height (−1 = default). */
  int window_height = -1;

  /** Snapshot of the live camera at save time. */
  SavedViewConfig current_view;

  /** Whether @c current_view is valid. */
  bool has_current_view = false;
};

/**
 * @class SessionConfigIO
 * @brief Static load / save / default helpers for @ref SessionConfig.
 */
class SessionConfigIO {
 public:
  /**
   * @brief Loads a session from a filesystem path.
   *
   * @param path Path to @c .autoviz / YAML session file.
   * @param[out] config Destination (non-null).
   * @return @c true on success.
   */
  static bool load(const std::string& path, SessionConfig* config);

  /**
   * @brief Saves a session to a filesystem path.
   *
   * @param path Destination path.
   * @param config Session to write.
   * @return @c true on success.
   */
  static bool save(const std::string& path, const SessionConfig& config);

  /**
   * @brief Returns a fresh default session (empty displays, default globals).
   * @return Default @ref SessionConfig.
   */
  static SessionConfig defaultConfig();
};

}  // namespace common
}  // namespace autoviz
