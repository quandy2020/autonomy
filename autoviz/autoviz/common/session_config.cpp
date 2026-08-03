/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/session_config.hpp"

#include <fstream>

#include <QString>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include "autoviz/common/display_property.hpp"
#include "autoviz/common/view_controller_registry.hpp"
#include "autoviz/common/config_session.hpp"
#include "autoviz/common/yaml_config_reader.hpp"
#include "autoviz/common/yaml_config_writer.hpp"
#include "autoviz/ui/panel_rviz_map.hpp"

namespace autoviz {
namespace common {
namespace {

std::string StripLeadingSlash(std::string frame) {
  if (!frame.empty() && frame.front() == '/') {
    frame.erase(frame.begin());
  }
  return frame;
}

std::string NormalizeRvizColor(const std::string& value) {
  std::string normalized;
  normalized.reserve(value.size());
  for (char ch : value) {
    if (ch != ' ') {
      normalized.push_back(ch);
    }
  }
  return normalized;
}

std::string RvizClassShortName(const std::string& rviz_class) {
  const auto pos = rviz_class.rfind('/');
  if (pos == std::string::npos) {
    return rviz_class;
  }
  return rviz_class.substr(pos + 1);
}

std::string MapRvizDisplayType(const std::string& rviz_class) {
  static const struct {
    const char* rviz_name;
    const char* autoviz_type;
  } kMap[] = {
      {"Grid", "Grid"},
      {"Axes", "Axes"},
      {"TF", "TF"},
      {"RobotModel", "RobotModel"},
      {"LaserScan", "LaserScan"},
      {"PointCloud2", "PointCloud2"},
      {"PointCloud", "PointCloud2"},
      {"Map", "Map"},
      {"Odometry", "Odometry"},
      {"Path", "Path"},
      {"Marker", "Marker"},
      {"MarkerArray", "MarkerArray"},
      {"Image", "Image"},
      {"Camera", "Camera"},
      {"Pose", "Pose"},
      {"PoseArray", "PoseArray"},
      {"PoseWithCovariance", "PoseWithCovariance"},
      {"GridCells", "GridCells"},
      {"Polygon", "Polygon"},
      {"PointStamped", "PointStamped"},
      {"Range", "Range"},
      {"TwistStamped", "TwistStamped"},
      {"AccelStamped", "AccelStamped"},
      {"CameraInfo", "CameraInfo"},
      {"DepthCloud", "DepthCloud"},
      {"Imu", "Imu"},
      {"Temperature", "Temperature"},
      {"Illuminance", "Illuminance"},
      {"FluidPressure", "FluidPressure"},
      {"RelativeHumidity", "RelativeHumidity"},
      {"Group", "Group"},
      {"WrenchStamped", "Wrench"},
      {"Wrench", "Wrench"},
      {"Effort", "Effort"},
      {"InteractiveMarkers", "InteractiveMarkers"},
  };
  const std::string short_name = RvizClassShortName(rviz_class);
  for (const auto& entry : kMap) {
    if (short_name == entry.rviz_name) {
      return entry.autoviz_type;
    }
  }
  return {};
}

std::string MapRvizViewType(const std::string& rviz_class) {
  return ViewControllerRegistry::instance().mapRvizClass(rviz_class);
}

std::string MapRvizPanelObjectName(const std::string& rviz_panel_title) {
  return MapRvizPanelToObjectName(rviz_panel_title);
}

std::string MapRvizPanelClass(const std::string& rviz_class) {
  return MapRvizPanelToObjectName(rviz_class);
}

void ImportRvizViewNode(const YAML::Node& view, SessionConfig* config,
                        bool as_current) {
  if (!view || !view.IsMap() || !view["Class"]) {
    return;
  }
  SavedViewConfig saved;
  saved.name = view["Name"].as<std::string>(as_current ? "Current View" : "View");
  saved.type = MapRvizViewType(view["Class"].as<std::string>());
  saved.yaw = view["Yaw"].as<float>(saved.yaw);
  saved.pitch = view["Pitch"].as<float>(saved.pitch);
  saved.distance = view["Distance"].as<float>(saved.distance);
  saved.near_clip_distance =
      view["Near Clip Distance"].as<float>(saved.near_clip_distance);
  saved.invert_z_axis = view["Invert Z Axis"].as<bool>(saved.invert_z_axis);
  saved.focal_shape_size =
      view["Focal Shape Size"].as<float>(saved.focal_shape_size);
  saved.focal_shape_fixed_size =
      view["Focal Shape Fixed Size"].as<bool>(saved.focal_shape_fixed_size);
  if (view["Focal Point"]) {
    saved.target_x = view["Focal Point"]["X"].as<float>(0.f);
    saved.target_y = view["Focal Point"]["Y"].as<float>(0.f);
    saved.target_z = view["Focal Point"]["Z"].as<float>(0.f);
  }
  if (view["Position"]) {
    saved.fps_position_x = view["Position"]["X"].as<float>(saved.fps_position_x);
    saved.fps_position_y = view["Position"]["Y"].as<float>(saved.fps_position_y);
    saved.fps_position_z = view["Position"]["Z"].as<float>(saved.fps_position_z);
  }
  if (as_current) {
    config->has_current_view = true;
    config->current_view = saved;
    config->view_controller = saved.type;
  }
  config->views.push_back(saved);
}

std::string MapRvizToolId(const std::string& rviz_class) {
  static const struct {
    const char* rviz_name;
    const char* autoviz_id;
  } kMap[] = {
      {"Interact", "Interact"},
      {"MoveCamera", "MoveCamera"},
      {"Select", "Select"},
      {"FocusCamera", "FocusCamera"},
      {"Measure", "Measure"},
      {"SetInitialPose", "PoseEstimate"},
      {"SetGoal", "NavGoal"},
      {"PublishPoint", "PublishPoint"},
  };
  const std::string short_name = RvizClassShortName(rviz_class);
  for (const auto& entry : kMap) {
    if (short_name == entry.rviz_name) {
      return entry.autoviz_id;
    }
  }
  return {};
}

std::string ReadRvizTopic(const YAML::Node& node) {
  if (node["Topic"]) {
    return node["Topic"].as<std::string>();
  }
  if (node["Image Topic"]) {
    return node["Image Topic"].as<std::string>();
  }
  if (node["Marker Topic"]) {
    return node["Marker Topic"].as<std::string>();
  }
  if (node["LaserScan Topic"]) {
    return node["LaserScan Topic"].as<std::string>();
  }
  if (node["Depth Map Topic"]) {
    return node["Depth Map Topic"].as<std::string>();
  }
  if (node["CameraInfo Topic"]) {
    return node["CameraInfo Topic"].as<std::string>();
  }
  return {};
}

void ImportRvizProperty(const YAML::Node& node, const char* rviz_key,
                        const char* autoviz_key, DisplayPropertyMap* props) {
  if (props == nullptr || !node[rviz_key]) {
    return;
  }
  const YAML::Node value = node[rviz_key];
  if (value.IsScalar()) {
    (*props)[autoviz_key] = value.as<std::string>();
  }
}

void ImportRvizColorProperty(const YAML::Node& node, const char* rviz_key,
                             const char* autoviz_key, DisplayPropertyMap* props) {
  if (props == nullptr || !node[rviz_key]) {
    return;
  }
  (*props)[autoviz_key] = NormalizeRvizColor(node[rviz_key].as<std::string>());
}

void ImportRvizDisplayProperties(const std::string& type,
                                 const YAML::Node& node,
                                 DisplayPropertyMap* props) {
  if (props == nullptr) {
    return;
  }

  ImportRvizColorProperty(node, "Color", "color", props);
  if (node["Alpha"]) {
    (*props)["alpha"] = std::to_string(node["Alpha"].as<float>());
  }
  ImportRvizProperty(node, "Line Style", "line_style", props);
  if (node["Line Width"]) {
    (*props)["line_width"] = std::to_string(node["Line Width"].as<float>());
  }
  ImportRvizProperty(node, "Style", "style", props);
  if (node["Size (Pixels)"]) {
    (*props)["point_size"] = std::to_string(node["Size (Pixels)"].as<float>());
  }
  if (node["Color Transformer"]) {
    std::string transform = node["Color Transformer"].as<std::string>();
    if (transform == "FlatColor") {
      transform = "Flat";
    } else if (transform == "AxisColor") {
      transform = "Axis";
    }
    (*props)["color_transform"] = transform;
  }
  ImportRvizProperty(node, "Pose Style", "pose_style", props);
  if (node["Buffer Length"]) {
    (*props)["buffer_length"] = std::to_string(node["Buffer Length"].as<int>());
  }
  if (node["Offset"]) {
    (*props)["offset"] = NormalizeRvizColor(node["Offset"].as<std::string>());
  }
  ImportRvizColorProperty(node, "Pose Color", "pose_color", props);
  if (node["Plane Cell Count"]) {
    (*props)["cell_count"] = std::to_string(node["Plane Cell Count"].as<int>());
  }
  if (node["Normal Cell Count"]) {
    (*props)["normal_cell_count"] =
        std::to_string(node["Normal Cell Count"].as<int>());
  }
  if (node["Cell Size"]) {
    (*props)["cell_size"] = std::to_string(node["Cell Size"].as<float>());
  }
  ImportRvizProperty(node, "Plane", "plane", props);
  ImportRvizProperty(node, "Reference Frame", "reference_frame", props);
  ImportRvizProperty(node, "Shape", "shape", props);
  ImportRvizProperty(node, "Show Names", "show_names", props);
  ImportRvizProperty(node, "Show Axes", "show_axes", props);
  ImportRvizProperty(node, "Show Arrows", "show_arrows", props);
  if (node["Marker Scale"]) {
    (*props)["marker_scale"] = std::to_string(node["Marker Scale"].as<float>());
  }
  if (node["Decay Time"]) {
    (*props)["decay_time"] = std::to_string(node["Decay Time"].as<float>());
  }
  if (node["Keep"]) {
    (*props)["keep"] = std::to_string(node["Keep"].as<int>());
  }

  if (type == "Path") {
    if (node["Length"]) {
      (*props)["pose_axes_length"] = std::to_string(node["Length"].as<float>());
    }
    if (node["Radius"]) {
      (*props)["pose_axes_radius"] = std::to_string(node["Radius"].as<float>());
    }
    if (node["Shaft Length"]) {
      (*props)["pose_arrow_shaft_length"] =
          std::to_string(node["Shaft Length"].as<float>());
    }
    if (node["Head Length"]) {
      (*props)["pose_arrow_head_length"] =
          std::to_string(node["Head Length"].as<float>());
    }
    if (node["Shaft Diameter"]) {
      (*props)["pose_arrow_shaft_diameter"] =
          std::to_string(node["Shaft Diameter"].as<float>());
    }
    if (node["Head Diameter"]) {
      (*props)["pose_arrow_head_diameter"] =
          std::to_string(node["Head Diameter"].as<float>());
    }
  }

  if (type == "RobotModel") {
    ImportRvizProperty(node, "Description Source", "description_source", props);
    ImportRvizProperty(node, "Description File", "urdf_path", props);
    ImportRvizProperty(node, "Description Topic", "description_channel", props);
    ImportRvizProperty(node, "TF Prefix", "tf_prefix", props);
    if (node["Update Interval"]) {
      (*props)["update_interval"] =
          std::to_string(node["Update Interval"].as<float>());
    }
  }
}

void ImportRvizDisplayNode(const YAML::Node& node,
                          std::vector<DisplayConfig>* out,
                          SessionConfig* config = nullptr) {
  if (!node || !node.IsMap() || out == nullptr) {
    return;
  }
  if (!node["Class"]) {
    return;
  }
  const std::string rviz_class = node["Class"].as<std::string>();
  const std::string short_name = RvizClassShortName(rviz_class);
  if (short_name == "Group") {
    DisplayConfig entry;
    entry.type = "Group";
    entry.name = node["Name"].as<std::string>("Group");
    entry.enabled = node["Enabled"].as<bool>(true);
    if (node["Displays"]) {
      for (const auto& child : node["Displays"]) {
        ImportRvizDisplayNode(child, &entry.children);
      }
    }
    out->push_back(std::move(entry));
    return;
  }

  const std::string type = MapRvizDisplayType(rviz_class);
  if (type.empty()) {
    LOG(WARNING) << "Skipping unsupported RViz display class: " << rviz_class;
    return;
  }

  DisplayConfig entry;
  entry.type = type;
  entry.name = node["Name"].as<std::string>(type);
  entry.enabled = node["Enabled"].as<bool>(true);
  entry.channel = ReadRvizTopic(node);

  ImportRvizDisplayProperties(type, node, &entry.properties);

  if (node["Color"]) {
    entry.properties["color"] =
        NormalizeRvizColor(node["Color"].as<std::string>());
  }
  if (node["Alpha"]) {
    entry.properties["alpha"] =
        std::to_string(node["Alpha"].as<float>());
  }
  if (node["Robot Description"]) {
    entry.properties["description_channel"] =
        node["Robot Description"].as<std::string>();
  }
  if (node["CameraInfo Topic"]) {
    entry.properties["camera_info_channel"] =
        node["CameraInfo Topic"].as<std::string>();
  }
  if (node["Color Image Topic"]) {
    entry.properties["color_channel"] =
        node["Color Image Topic"].as<std::string>();
  }
  if (type == "Grid" && entry.enabled && config != nullptr) {
    config->show_grid = true;
  }

  out->push_back(std::move(entry));
}

void ReadDisplayNode(const YAML::Node& node, DisplayConfig* entry) {
  if (entry == nullptr || !node || !node.IsMap()) {
    return;
  }
  entry->type = node["Type"].as<std::string>();
  entry->name = node["Name"].as<std::string>(entry->type);
  entry->channel = node["Channel"].as<std::string>();
  entry->enabled = node["Enabled"].as<bool>(true);
  entry->properties.clear();
  entry->children.clear();
  if (node["Properties"]) {
    for (const auto& prop : node["Properties"]) {
      entry->properties[prop.first.as<std::string>()] =
          prop.second.as<std::string>();
    }
  }
  if (node["Children"]) {
    for (const auto& child_node : node["Children"]) {
      DisplayConfig child;
      ReadDisplayNode(child_node, &child);
      entry->children.push_back(std::move(child));
    }
  }
}

void WriteDisplayNode(YAML::Emitter* out, const DisplayConfig& display) {
  *out << YAML::BeginMap;
  *out << YAML::Key << "Type" << YAML::Value << display.type;
  *out << YAML::Key << "Name" << YAML::Value << display.name();
  *out << YAML::Key << "Channel" << YAML::Value << display.channel;
  *out << YAML::Key << "Enabled" << YAML::Value << display.enabled;
  if (!display.properties.empty()) {
    *out << YAML::Key << "Properties";
    *out << YAML::Value << YAML::BeginMap;
    for (const auto& prop : display.properties) {
      *out << YAML::Key << prop.first << YAML::Value << prop.second;
    }
    *out << YAML::EndMap;
  }
  if (!display.children.empty()) {
    *out << YAML::Key << "Children";
    *out << YAML::Value << YAML::BeginSeq;
    for (const auto& child : display.children) {
      WriteDisplayNode(out, child);
    }
    *out << YAML::EndSeq;
  }
  *out << YAML::EndMap;
}

bool LoadFromRvizYaml(const YAML::Node& root, SessionConfig* config) {
  const YAML::Node manager = root["Visualization Manager"];
  if (!manager) {
    return false;
  }

  config->visible_panels.clear();
  if (root["Panels"]) {
    for (const auto& panel_node : root["Panels"]) {
      if (!panel_node.IsMap()) {
        continue;
      }
      std::string key;
      if (panel_node["Class"]) {
        key = MapRvizPanelClass(panel_node["Class"].as<std::string>());
      } else if (panel_node["Name"]) {
        key = MapRvizPanelObjectName(panel_node["Name"].as<std::string>());
      }
      if (!key.empty()) {
        config->visible_panels.push_back(key);
      }
    }
  }

  if (manager["Global Options"]) {
    const YAML::Node global = manager["Global Options"];
    if (global["Fixed Frame"]) {
      config->fixed_frame =
          StripLeadingSlash(global["Fixed Frame"].as<std::string>());
    }
    if (global["Background Color"]) {
      config->background_color =
          NormalizeRvizColor(global["Background Color"].as<std::string>());
    }
    if (global["Frame Rate"]) {
      config->frame_rate = global["Frame Rate"].as<int>(config->frame_rate);
    }
  }

  config->displays.clear();
  if (manager["Displays"]) {
    for (const auto& display_node : manager["Displays"]) {
      ImportRvizDisplayNode(display_node, &config->displays, config);
    }
  }

  config->views.clear();
  if (manager["Views"]) {
    if (manager["Views"]["Current"]) {
      ImportRvizViewNode(manager["Views"]["Current"], config, true);
    }
    if (manager["Views"]["Saved"] && manager["Views"]["Saved"].IsSequence()) {
      for (const auto& saved_view : manager["Views"]["Saved"]) {
        ImportRvizViewNode(saved_view, config, false);
      }
    }
  }

  config->tools.clear();
  if (manager["Tools"]) {
    for (const auto& tool_node : manager["Tools"]) {
      if (!tool_node["Class"]) {
        continue;
      }
      const std::string tool_id =
          MapRvizToolId(tool_node["Class"].as<std::string>());
      if (tool_id.empty()) {
        continue;
      }
      ToolConfig tool;
      tool.id = tool_id;
      config->toolbar_tools.push_back(tool_id);
      if (tool_node["Topic"]) {
        tool.properties["topic"] = tool_node["Topic"].as<std::string>();
      }
      config->tools.push_back(std::move(tool));
    }
  }

  if (manager["Transformation"]) {
    const YAML::Node transformation = manager["Transformation"];
    if (transformation["Current"] && transformation["Current"]["Class"]) {
      const std::string rviz_class =
          transformation["Current"]["Class"].as<std::string>();
      if (rviz_class.find("Identity") != std::string::npos) {
        config->transformer_id = "autoviz/Identity";
      } else {
        config->transformer_id = "autoviz/AutolinkTf";
      }
    }
  }

  if (root["Window Geometry"]) {
    const YAML::Node window = root["Window Geometry"];
    if (window["Hide Left Dock"]) {
      config->hide_left_dock = window["Hide Left Dock"].as<bool>();
    }
    if (window["Hide Right Dock"]) {
      config->hide_right_dock = window["Hide Right Dock"].as<bool>();
    }
    if (window["QMainWindow State"]) {
      config->window_state_b64 =
          window["QMainWindow State"].as<std::string>();
    }
    if (window["Width"]) {
      config->window_width = window["Width"].as<int>();
    }
    if (window["Height"]) {
      config->window_height = window["Height"].as<int>();
    }
    if (window["X"]) {
      config->window_x = window["X"].as<int>();
    }
    if (window["Y"]) {
      config->window_y = window["Y"].as<int>();
    }
    config->panel_layouts.clear();
    for (const auto& pair : window) {
      if (!pair.second.IsMap()) {
        continue;
      }
      if (!pair.second["collapsed"]) {
        continue;
      }
      PanelLayoutConfig panel;
      panel.object_name =
          MapRvizPanelObjectName(pair.first.as<std::string>());
      panel.collapsed = pair.second["collapsed"].as<bool>(false);
      config->panel_layouts.push_back(std::move(panel));
    }
  }

  return true;
}

void ReadViewFields(const YAML::Node& node, SavedViewConfig* view) {
  if (node["Name"]) {
    view->name = node["Name"].as<std::string>();
  }
  if (node["Type"]) {
    view->type = node["Type"].as<std::string>();
  }
  view->yaw = node["Yaw"].as<float>(view->yaw);
  view->pitch = node["Pitch"].as<float>(view->pitch);
  view->distance = node["Distance"].as<float>(view->distance);
  view->near_clip_distance =
      node["NearClipDistance"].as<float>(view->near_clip_distance);
  view->invert_z_axis = node["InvertZAxis"].as<bool>(view->invert_z_axis);
  view->focal_shape_size =
      node["FocalShapeSize"].as<float>(view->focal_shape_size);
  view->focal_shape_fixed_size =
      node["FocalShapeFixedSize"].as<bool>(view->focal_shape_fixed_size);
  if (node["Target"]) {
    view->target_x = node["Target"]["X"].as<float>(view->target_x);
    view->target_y = node["Target"]["Y"].as<float>(view->target_y);
    view->target_z = node["Target"]["Z"].as<float>(view->target_z);
  }
  if (node["FpsPosition"]) {
    view->fps_position_x = node["FpsPosition"]["X"].as<float>(view->fps_position_x);
    view->fps_position_y = node["FpsPosition"]["Y"].as<float>(view->fps_position_y);
    view->fps_position_z = node["FpsPosition"]["Z"].as<float>(view->fps_position_z);
  }
  view->fps_yaw = node["FpsYaw"].as<float>(view->fps_yaw);
  view->fps_pitch = node["FpsPitch"].as<float>(view->fps_pitch);
  if (node["TargetFrame"]) {
    view->target_frame = node["TargetFrame"].as<std::string>();
  }
}

void WriteViewFields(YAML::Emitter* out, const SavedViewConfig& view) {
  *out << YAML::Key << "Name" << YAML::Value << view.name();
  *out << YAML::Key << "Type" << YAML::Value << view.type;
  *out << YAML::Key << "NearClipDistance" << YAML::Value << view.near_clip_distance;
  *out << YAML::Key << "InvertZAxis" << YAML::Value << view.invert_z_axis;
  *out << YAML::Key << "Yaw" << YAML::Value << view.yaw;
  *out << YAML::Key << "Pitch" << YAML::Value << view.pitch;
  *out << YAML::Key << "Distance" << YAML::Value << view.distance;
  *out << YAML::Key << "FocalShapeSize" << YAML::Value << view.focal_shape_size;
  *out << YAML::Key << "FocalShapeFixedSize" << YAML::Value
       << view.focal_shape_fixed_size;
  *out << YAML::Key << "Target";
  *out << YAML::Value << YAML::BeginMap;
  *out << YAML::Key << "X" << YAML::Value << view.target_x;
  *out << YAML::Key << "Y" << YAML::Value << view.target_y;
  *out << YAML::Key << "Z" << YAML::Value << view.target_z;
  *out << YAML::EndMap;
  *out << YAML::Key << "FpsPosition";
  *out << YAML::Value << YAML::BeginMap;
  *out << YAML::Key << "X" << YAML::Value << view.fps_position_x;
  *out << YAML::Key << "Y" << YAML::Value << view.fps_position_y;
  *out << YAML::Key << "Z" << YAML::Value << view.fps_position_z;
  *out << YAML::EndMap;
  *out << YAML::Key << "FpsYaw" << YAML::Value << view.fps_yaw;
  *out << YAML::Key << "FpsPitch" << YAML::Value << view.fps_pitch;
  if (!view.target_frame.empty()) {
    *out << YAML::Key << "TargetFrame" << YAML::Value << view.target_frame;
  }
}

}  // namespace

SessionConfig SessionConfigIO::defaultConfig() {
  SessionConfig config;
  config.fixed_frame = "map";
  config.show_grid = true;
  config.view_controller = "Orbit";
  config.render_backend = "OpenGL";
  config.displays = {
      {"Grid", "Grid", "", true},
      {"Axes", "Axes", "", true},
      {"TF", "TF", "/tf", true},
      {"LaserScan", "Scan", "/fake/scan", true},
      {"Marker", "Marker", "/fake/marker", true},
      {"Path", "Path", "/fake/path", true},
      {"Map", "Map", "/fake/occupancy_grid", true},
      {"Odometry", "Odometry", "/fake/odom", true},
      {"PointCloud2", "PointCloud2", "/fake/point_cloud2", true},
      {"RobotModel", "Robot", "/joint_states", true},
  };
  return config;
}

bool SessionConfigIO::load(const std::string& path, SessionConfig* config) {
  if (config == nullptr) {
    return false;
  }
  YamlConfigReader reader;
  Config root;
  reader.readFile(root, QString::fromStdString(path));
  if (reader.error()) {
    LOG(ERROR) << "Failed to load config " << path << ": "
               << reader.errorMessage().toStdString();
    return false;
  }

  if (root.mapGetChild("Visualization Manager").isValid()) {
    *config = SessionConfig{};
    try {
      const YAML::Node yaml_root = YAML::LoadFile(path);
      if (!LoadFromRvizYaml(yaml_root, config)) {
        return false;
      }
    } catch (const std::exception& e) {
      LOG(ERROR) << "Failed to import RViz config " << path << ": " << e.what();
      return false;
    }
    SessionConfig overlay;
    Config manager = root.mapGetChild("Visualization Manager");
    Config transformation = manager.mapGetChild("Transformation");
    Config current = transformation.mapGetChild("Current");
    QString transformer_class;
    if (current.mapGetString("Class", &transformer_class)) {
      if (transformer_class.contains(QStringLiteral("Identity"))) {
        config->transformer_id = "autoviz/Identity";
      } else {
        config->transformer_id = "autoviz/AutolinkTf";
      }
    }
    LOG(INFO) << "Imported RViz config: " << path;
    return true;
  }

  *config = SessionConfig{};
  return SessionConfigFromConfig(root, config);
}

bool SessionConfigIO::save(const std::string& path,
                           const SessionConfig& config) {
  Config root;
  SessionConfigToConfig(config, &root);
  YamlConfigWriter writer;
  writer.writeFile(root, QString::fromStdString(path));
  if (writer.error()) {
    LOG(ERROR) << "Failed to save config " << path << ": "
               << writer.errorMessage().toStdString();
    return false;
  }
  return true;
}

}  // namespace common
}  // namespace autoviz
