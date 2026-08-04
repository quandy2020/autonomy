/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/display_registry.hpp"

#include <QDir>
#include <QFileInfo>

#include "autoviz/common/path_env_utils.hpp"
#include "autoviz/common/plugin_loader.hpp"

#include <automsgs/msgs/sensor_msgs/fluid_pressure.pb.h>
#include <automsgs/msgs/sensor_msgs/illuminance.pb.h>
#include <automsgs/msgs/sensor_msgs/relative_humidity.pb.h>
#include <automsgs/msgs/sensor_msgs/temperature.pb.h>
#include "autoviz/display/accel_stamped_display.hpp"
#include "autoviz/display/display_group.hpp"
#include "autoviz/display/imu_display.hpp"
#include "autoviz/display/scalar_sensor_display.hpp"
#include "autoviz/display/axes_display.hpp"
#include "autoviz/display/camera_display.hpp"
#include "autoviz/display/camera_info_display.hpp"
#include "autoviz/display/depth_cloud_display.hpp"
#include "autoviz/display/grid_display.hpp"
#include "autoviz/display/image_display.hpp"
#include "autoviz/display/interactive_marker_display.hpp"
#include "autoviz/display/laser_scan_display.hpp"
#include "autoviz/display/map_display.hpp"
#include "autoviz/display/marker_array_display.hpp"
#include "autoviz/display/marker_display.hpp"
#include "autoviz/display/odometry_display.hpp"
#include "autoviz/display/path_display.hpp"
#include "autoviz/display/point_cloud2_display.hpp"
#include "autoviz/display/pose_array_display.hpp"
#include "autoviz/display/pose_display.hpp"
#include "autoviz/display/robot_model_display.hpp"
#include "autoviz/display/strata_fov_display.hpp"
#include "autoviz/display/strata_building_display.hpp"
#include "autoviz/display/strata_canvas_label_display.hpp"
#include "autoviz/display/strata_iot_bubble_display.hpp"
#include "autoviz/display/strata_label_bubble_display.hpp"
#include "autoviz/display/strata_robot3d_display.hpp"
#include "autoviz/display/strata_poi_display.hpp"
#include "autoviz/display/strata_robot_display.hpp"
#include "autoviz/display/strata_road_graph_display.hpp"
#include "autoviz/display/strata_semantic_zone_display.hpp"
#include "autoviz/display/effort_display.hpp"
#include "autoviz/display/grid_cells_display.hpp"
#include "autoviz/display/point_stamped_display.hpp"
#include "autoviz/display/polygon_display.hpp"
#include "autoviz/display/pose_with_covariance_display.hpp"
#include "autoviz/display/range_display.hpp"
#include "autoviz/display/tf_display.hpp"
#include "autoviz/display/twist_stamped_display.hpp"
#include "autoviz/display/wrench_display.hpp"

namespace autoviz {
namespace common {
namespace {

DisplayConfig MakeDefault(const std::string& type, const std::string& name,
                          const std::string& channel) {
  DisplayConfig config;
  config.type = type;
  config.name = name;
  config.channel = channel;
  config.enabled = true;
  return config;
}

void ApplyDefaultProperties(display::Display* display) {
  DisplayPropertyMap defaults;
  for (const auto& spec : display->propertySpecs()) {
    defaults[spec.key] = spec.default_value;
  }
  display->setProperties(defaults);
}

std::unique_ptr<display::Display> FinalizeDisplay(
    std::unique_ptr<display::Display> display, const DisplayConfig& config) {
  if (display == nullptr) {
    return nullptr;
  }
  ApplyDefaultProperties(display.get());
  display->loadFromConfig(config);
  return display;
}

}  // namespace

DisplayRegistry& DisplayRegistry::instance() {
  static DisplayRegistry registry;
  static bool initialized = false;
  if (!initialized) {
    registry.registerBuiltinTypes();
    registry.loadPluginsFromEnv();
    initialized = true;
  }
  return registry;
}

void DisplayRegistry::registerType(const std::string& type,
                                   DisplayCreator creator,
                                   DisplayDefaultConfig default_config) {
  entries_[type] = Entry{std::move(creator), std::move(default_config)};
}

void DisplayRegistry::registerBuiltinTypes() {
  registerType(
      "Grid",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(std::make_unique<display::GridDisplay>(), config);
      },
      [] { return MakeDefault("Grid", "Grid", ""); });

  registerType(
      "Axes",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(std::make_unique<display::AxesDisplay>(), config);
      },
      [] { return MakeDefault("Axes", "Axes", ""); });

  registerType(
      "TF",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(std::make_unique<display::TfDisplay>(config.channel),
                               config);
      },
      [] { return MakeDefault("TF", "TF", "/tf"); });

  registerType(
      "LaserScan",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::LaserScanDisplay>(config.channel), config);
      },
      [] { return MakeDefault("LaserScan", "Scan", "/fake/scan"); });

  registerType(
      "Marker",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(std::make_unique<display::MarkerDisplay>(config.channel),
                               config);
      },
      [] { return MakeDefault("Marker", "Marker", "/fake/marker"); });

  registerType(
      "MarkerArray",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::MarkerArrayDisplay>(config.channel), config);
      },
      [] { return MakeDefault("MarkerArray", "MarkerArray", "/fake/marker_array"); });

  registerType(
      "StrataPoi",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::StrataPoiDisplay>(config.channel), config);
      },
      [] { return MakeDefault("StrataPoi", "Strata POI", "/strata/poi_markers"); });

  registerType(
      "StrataRobot",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::StrataRobotDisplay>(config.channel), config);
      },
      [] {
        return MakeDefault("StrataRobot", "Strata Robot", "/strata/robot_markers");
      });

  registerType(
      "StrataSemanticZone",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::StrataSemanticZoneDisplay>(config.channel),
            config);
      },
      [] {
        return MakeDefault("StrataSemanticZone", "Strata Semantic Zones",
                           "/strata/semantic_zones");
      });

  registerType(
      "StrataRoadGraph",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::StrataRoadGraphDisplay>(config.channel), config);
      },
      [] {
        return MakeDefault("StrataRoadGraph", "Strata Road Graph", "/strata/road_graph");
      });

  registerType(
      "StrataFov",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::StrataFovDisplay>(config.channel), config);
      },
      [] { return MakeDefault("StrataFov", "Strata Robot FOV", "/strata/markers"); });

  registerType(
      "StrataCanvasLabel",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::StrataCanvasLabelDisplay>(config.channel), config);
      },
      [] {
        return MakeDefault("StrataCanvasLabel", "Strata Canvas Labels",
                           "/strata/canvas_labels");
      });

  registerType(
      "StrataLabelBubble",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::StrataLabelBubbleDisplay>(config.channel), config);
      },
      [] {
        return MakeDefault("StrataLabelBubble", "Strata Label Bubbles",
                           "/strata/label_bubbles");
      });

  registerType(
      "StrataIotBubble",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::StrataIotBubbleDisplay>(config.channel), config);
      },
      [] {
        return MakeDefault("StrataIotBubble", "Strata IoT Bubbles", "/strata/iot_bubbles");
      });

  registerType(
      "StrataRobot3D",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::StrataRobot3DDisplay>(config.channel), config);
      },
      [] {
        return MakeDefault("StrataRobot3D", "Strata Robot 3D Layers",
                           "/strata/robot_3d_layers");
      });

  registerType(
      "StrataBuilding",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::StrataBuildingDisplay>(config.channel), config);
      },
      [] {
        return MakeDefault("StrataBuilding", "Strata Buildings", "/strata/markers");
      });

  registerType(
      "Path",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(std::make_unique<display::PathDisplay>(config.channel),
                               config);
      },
      [] { return MakeDefault("Path", "Path", "/fake/path"); });

  registerType(
      "Map",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(std::make_unique<display::MapDisplay>(config.channel),
                               config);
      },
      [] { return MakeDefault("Map", "Map", "/fake/occupancy_grid"); });

  registerType(
      "Odometry",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::OdometryDisplay>(config.channel), config);
      },
      [] { return MakeDefault("Odometry", "Odometry", "/fake/odom"); });

  registerType(
      "PointCloud2",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::PointCloud2Display>(config.channel), config);
      },
      [] { return MakeDefault("PointCloud2", "PointCloud2", "/fake/point_cloud2"); });

  registerType(
      "PointCloud",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::PointCloud2Display>(config.channel), config);
      },
      [] { return MakeDefault("PointCloud", "PointCloud", "/fake/point_cloud"); });

  registerType(
      "Pose",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(std::make_unique<display::PoseDisplay>(config.channel),
                               config);
      },
      [] { return MakeDefault("Pose", "Pose", "/fake/pose"); });

  registerType(
      "PoseArray",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::PoseArrayDisplay>(config.channel), config);
      },
      [] { return MakeDefault("PoseArray", "PoseArray", "/fake/pose_array"); });

  registerType(
      "Image",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(std::make_unique<display::ImageDisplay>(config.channel),
                               config);
      },
      [] { return MakeDefault("Image", "Camera", "/fake/image"); });

  registerType(
      "RobotModel",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::RobotModelDisplay>(config.channel), config);
      },
      []() {
        auto config = MakeDefault("RobotModel", "Robot", "/joint_states");
        config.properties["description_channel"] = "/robot_description";
        config.properties["root_link"] = "base_link";
        config.properties["urdf_path"] = "config/turtlebot3_burger.urdf";
        return config;
      });

  registerType(
      "Camera",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::CameraDisplay>(config.channel), config);
      },
      []() {
        auto config = MakeDefault("Camera", "Camera", "/fake/image");
        config.properties["camera_info_channel"] = "/fake/camera_info";
        return config;
      });

  registerType(
      "InteractiveMarkers",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::InteractiveMarkerDisplay>(config.channel),
            config);
      },
      []() {
        auto config =
            MakeDefault("InteractiveMarkers", "InteractiveMarkers", "/fake/update");
        config.properties["init_channel"] = "/fake/init";
        config.properties["feedback_channel"] = "/fake/feedback";
        return config;
      });

  registerType(
      "Wrench",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::WrenchDisplay>(config.channel), config);
      },
      [] { return MakeDefault("Wrench", "Wrench", "/fake/wrench"); });

  registerType(
      "Effort",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::EffortDisplay>(config.channel), config);
      },
      []() {
        auto config = MakeDefault("Effort", "Effort", "/joint_states");
        config.properties["description_channel"] = "/robot_description";
        config.properties["root_link"] = "base_link";
        config.properties["urdf_path"] = "config/turtlebot3_burger.urdf";
        return config;
      });

  registerType(
      "GridCells",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::GridCellsDisplay>(config.channel), config);
      },
      [] { return MakeDefault("GridCells", "GridCells", "/fake/grid_cells"); });

  registerType(
      "PointStamped",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::PointStampedDisplay>(config.channel),
            config);
      },
      [] { return MakeDefault("PointStamped", "PointStamped", "/fake/point"); });

  registerType(
      "Polygon",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::PolygonDisplay>(config.channel), config);
      },
      [] { return MakeDefault("Polygon", "Polygon", "/fake/polygon"); });

  registerType(
      "Range",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::RangeDisplay>(config.channel), config);
      },
      [] { return MakeDefault("Range", "Range", "/fake/range"); });

  registerType(
      "PoseWithCovariance",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::PoseWithCovarianceDisplay>(config.channel),
            config);
      },
      [] {
        return MakeDefault("PoseWithCovariance", "PoseWithCovariance",
                           "/fake/pose_with_covariance");
      });

  registerType(
      "TwistStamped",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::TwistStampedDisplay>(config.channel),
            config);
      },
      [] { return MakeDefault("TwistStamped", "TwistStamped", "/fake/twist"); });

  registerType(
      "CameraInfo",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::CameraInfoDisplay>(config.channel),
            config);
      },
      [] { return MakeDefault("CameraInfo", "CameraInfo", "/fake/camera_info"); });

  registerType(
      "DepthCloud",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::DepthCloudDisplay>(config.channel),
            config);
      },
      []() {
        auto config = MakeDefault("DepthCloud", "DepthCloud", "/fake/depth");
        config.properties["camera_info_channel"] = "/fake/camera_info";
        return config;
      });

  registerType(
      "AccelStamped",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::AccelStampedDisplay>(config.channel),
            config);
      },
      [] { return MakeDefault("AccelStamped", "AccelStamped", "/fake/accel"); });

  registerType(
      "Group",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(std::make_unique<display::DisplayGroup>(), config);
      },
      [] { return MakeDefault("Group", "Group", ""); });

  registerType(
      "Imu",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(std::make_unique<display::ImuDisplay>(config.channel),
                               config);
      },
      [] { return MakeDefault("Imu", "Imu", "/fake/imu"); });

  registerType(
      "Temperature",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::ScalarSensorDisplay<
                automsgs::msgs::sensor_msgs::Temperature>>(
                "Temperature", config.channel,
                "automsgs.msgs.sensor_msgs.Temperature",
                [](const automsgs::msgs::sensor_msgs::Temperature& msg) {
                  return msg.temperature();
                },
                "min_value", 0.0, "max_value", 100.0),
            config);
      },
      [] { return MakeDefault("Temperature", "Temperature", "/fake/temperature"); });

  registerType(
      "Illuminance",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::ScalarSensorDisplay<
                automsgs::msgs::sensor_msgs::Illuminance>>(
                "Illuminance", config.channel,
                "automsgs.msgs.sensor_msgs.Illuminance",
                [](const automsgs::msgs::sensor_msgs::Illuminance& msg) {
                  return msg.illuminance();
                },
                "min_value", 0.0, "max_value", 1000.0),
            config);
      },
      [] { return MakeDefault("Illuminance", "Illuminance", "/fake/illuminance"); });

  registerType(
      "FluidPressure",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::ScalarSensorDisplay<
                automsgs::msgs::sensor_msgs::FluidPressure>>(
                "FluidPressure", config.channel,
                "automsgs.msgs.sensor_msgs.FluidPressure",
                [](const automsgs::msgs::sensor_msgs::FluidPressure& msg) {
                  return msg.fluid_pressure();
                },
                "min_value", 90000.0, "max_value", 110000.0),
            config);
      },
      [] {
        return MakeDefault("FluidPressure", "FluidPressure", "/fake/fluid_pressure");
      });

  registerType(
      "RelativeHumidity",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::ScalarSensorDisplay<
                automsgs::msgs::sensor_msgs::RelativeHumidity>>(
                "RelativeHumidity", config.channel,
                "automsgs.msgs.sensor_msgs.RelativeHumidity",
                [](const automsgs::msgs::sensor_msgs::RelativeHumidity& msg) {
                  return msg.relative_humidity();
                },
                "min_value", 0.0, "max_value", 1.0),
            config);
      },
      [] {
        return MakeDefault("RelativeHumidity", "RelativeHumidity",
                           "/fake/relative_humidity");
      });
}

std::vector<std::string> DisplayRegistry::supportedTypes() const {
  std::vector<std::string> types;
  types.reserve(entries_.size());
  for (const auto& [type, entry] : entries_) {
    (void)entry;
    types.push_back(type);
  }
  return types;
}

DisplayConfig DisplayRegistry::defaultForType(const std::string& type) const {
  const auto it = entries_.find(type);
  if (it == entries_.end()) {
    return MakeDefault(type, type, "/");
  }
  return it->second.default_config();
}

std::unique_ptr<display::Display> DisplayRegistry::create(
    const DisplayConfig& config) const {
  if (config.type == "Group") {
    auto group = std::make_unique<display::DisplayGroup>();
    group->setDisplayName(config.name);
    for (const auto& child_config : config.children) {
      auto child = create(child_config);
      if (child != nullptr) {
        group->addChild(std::move(child));
      }
    }
    group->setEnabled(config.enabled);
    return group;
  }
  const auto it = entries_.find(config.type);
  if (it == entries_.end()) {
    return nullptr;
  }
  return it->second.creator(config);
}

void DisplayRegistry::loadPluginsFromEnv() {
  for (const QString& part : pluginSearchPaths()) {
    loadPluginsFromPath(part.toStdString());
  }
}

void DisplayRegistry::loadPluginsFromPath(const std::string& path) {
  QDir dir(QString::fromStdString(path));
  if (!dir.exists()) {
    return;
  }
  for (const QFileInfo& info : dir.entryInfoList(
           PluginLoader::libraryFilenameFilters(), QDir::Files | QDir::Readable)) {
    void* handle = PluginLoader::open(info.absoluteFilePath().toStdString());
    if (handle == nullptr) {
      continue;
    }
    using RegisterFn = void (*)(DisplayRegistry*);
    auto* register_fn = reinterpret_cast<RegisterFn>(
        PluginLoader::symbol(handle, "autoviz_register_displays"));
    if (register_fn == nullptr) {
      PluginLoader::close(handle);
      continue;
    }
    register_fn(this);
    plugin_handles_.push_back(handle);
  }
}

}  // namespace common
}  // namespace autoviz
