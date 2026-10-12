/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/display_registry.hpp"

#include <QDir>
#include <QFileInfo>

#include "autoviz/common/path_env_utils.hpp"
#include "autoviz/common/plugin_loader.hpp"

#include <automsgs/msgs/sensor_msgs/illuminance.pb.h>
#include "autoviz/plugins/displays/accel/accel_stamped_display.hpp"
#include "autoviz/plugins/displays/display_group.hpp"
#include "autoviz/plugins/displays/fluid_pressure/fluid_pressure_display.hpp"
#include "autoviz/plugins/displays/imu/imu_display.hpp"
#include "autoviz/plugins/displays/relative_humidity/relative_humidity_display.hpp"
#include "autoviz/plugins/displays/scalar_sensor/scalar_sensor_display.hpp"
#include "autoviz/plugins/displays/temperature/temperature_display.hpp"
#include "autoviz/plugins/displays/axes/axes_display.hpp"
#include "autoviz/plugins/displays/camera/camera_display.hpp"
#include "autoviz/plugins/displays/camera_info/camera_info_display.hpp"
#include "autoviz/plugins/displays/depth_cloud/depth_cloud_display.hpp"
#include "autoviz/plugins/displays/grid/grid_display.hpp"
#include "autoviz/plugins/displays/image/image_display.hpp"
#include "autoviz/plugins/displays/interactive_markers/interactive_marker_display.hpp"
#include "autoviz/plugins/displays/laser_scan/laser_scan_display.hpp"
#include "autoviz/plugins/displays/map/map_display.hpp"
#include "autoviz/plugins/displays/marker_array/marker_array_display.hpp"
#include "autoviz/plugins/displays/marker/marker_display.hpp"
#include "autoviz/plugins/displays/odometry/odometry_display.hpp"
#include "autoviz/plugins/displays/path/path_display.hpp"
#include "autoviz/plugins/displays/pointcloud/point_cloud2_display.hpp"
#include "autoviz/plugins/displays/pose_array/pose_array_display.hpp"
#include "autoviz/plugins/displays/pose/pose_display.hpp"
#include "autoviz/plugins/displays/robot_model/robot_model_display.hpp"
#include "autoviz/plugins/displays/effort/effort_display.hpp"
#include "autoviz/plugins/displays/grid_cells/grid_cells_display.hpp"
#include "autoviz/plugins/displays/grid_map/grid_map_display.hpp"
#include "autoviz/plugins/displays/point/point_stamped_display.hpp"
#include "autoviz/plugins/displays/polygon/polygon_display.hpp"
#include "autoviz/plugins/displays/pose_covariance/pose_with_covariance_display.hpp"
#include "autoviz/plugins/displays/range/range_display.hpp"
#include "autoviz/plugins/displays/tf/tf_display.hpp"
#include "autoviz/plugins/displays/twist/twist_stamped_display.hpp"
#include "autoviz/plugins/displays/wrench/wrench_display.hpp"

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
        // Always buffer-backed: Listener already covers /tf + /tf_static.
        // Clear legacy Channel bindings so old configs match RViz2.
        DisplayConfig normalized = config;
        normalized.channel.clear();
        return FinalizeDisplay(std::make_unique<display::TfDisplay>(""),
                               normalized);
      },
      [] { return MakeDefault("TF", "TF", ""); });

  registerType(
      "LaserScan",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::LaserScanDisplay>(config.channel), config);
      },
      [] { return MakeDefault("LaserScan", "Scan", ""); });

  registerType(
      "Marker",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(std::make_unique<display::MarkerDisplay>(config.channel),
                               config);
      },
      [] { return MakeDefault("Marker", "Marker", ""); });

  registerType(
      "MarkerArray",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::MarkerArrayDisplay>(config.channel), config);
      },
      [] { return MakeDefault("MarkerArray", "MarkerArray", ""); });

  registerType(
      "Path",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(std::make_unique<display::PathDisplay>(config.channel),
                               config);
      },
      [] { return MakeDefault("Path", "Path", ""); });

  registerType(
      "Map",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(std::make_unique<display::MapDisplay>(config.channel),
                               config);
      },
      [] { return MakeDefault("Map", "Map", "/map"); });

  registerType(
      "GridMap",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::GridMapDisplay>(config.channel), config);
      },
      [] { return MakeDefault("GridMap", "GridMap", "/grid_map"); });

  registerType(
      "Odometry",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::OdometryDisplay>(config.channel), config);
      },
      [] { return MakeDefault("Odometry", "Odometry", ""); });

  registerType(
      "PointCloud2",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::PointCloud2Display>(config.channel), config);
      },
      [] { return MakeDefault("PointCloud2", "PointCloud2", ""); });

  registerType(
      "PointCloud",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::PointCloud2Display>(config.channel), config);
      },
      [] { return MakeDefault("PointCloud", "PointCloud", ""); });

  registerType(
      "Pose",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(std::make_unique<display::PoseDisplay>(config.channel),
                               config);
      },
      [] { return MakeDefault("Pose", "Pose", ""); });

  registerType(
      "PoseArray",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::PoseArrayDisplay>(config.channel), config);
      },
      [] { return MakeDefault("PoseArray", "PoseArray", ""); });

  registerType(
      "Image",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(std::make_unique<display::ImageDisplay>(config.channel),
                               config);
      },
      [] { return MakeDefault("Image", "Camera", ""); });

  registerType(
      "RobotModel",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::RobotModelDisplay>(config.channel), config);
      },
      []() {
        auto config = MakeDefault("RobotModel", "Robot", "/joint_states");
        config.properties["description_channel"] = "/robot_description";
        config.properties["description_source"] = "Topic";
        config.properties["root_link"] = "";
        config.properties["urdf_path"] = "";
        config.properties["show_axes"] = "false";
        return config;
      });

  registerType(
      "Camera",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::CameraDisplay>(config.channel), config);
      },
      []() {
        auto config = MakeDefault("Camera", "Camera", "");
        config.properties["camera_info_channel"] = "";
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
            MakeDefault("InteractiveMarkers", "InteractiveMarkers", "");
        config.properties["init_channel"] = "";
        config.properties["feedback_channel"] = "";
        return config;
      });

  registerType(
      "Wrench",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::WrenchDisplay>(config.channel), config);
      },
      [] { return MakeDefault("Wrench", "Wrench", ""); });

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
        config.properties["urdf_path"] = "";
        return config;
      });

  registerType(
      "GridCells",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::GridCellsDisplay>(config.channel), config);
      },
      [] { return MakeDefault("GridCells", "GridCells", ""); });

  registerType(
      "PointStamped",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::PointStampedDisplay>(config.channel),
            config);
      },
      [] { return MakeDefault("PointStamped", "PointStamped", ""); });

  registerType(
      "Polygon",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::PolygonDisplay>(config.channel), config);
      },
      [] { return MakeDefault("Polygon", "Polygon", ""); });

  registerType(
      "Range",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::RangeDisplay>(config.channel), config);
      },
      [] { return MakeDefault("Range", "Range", ""); });

  registerType(
      "PoseWithCovariance",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::PoseWithCovarianceDisplay>(config.channel),
            config);
      },
      [] {
        return MakeDefault("PoseWithCovariance", "PoseWithCovariance",
                           "");
      });

  registerType(
      "TwistStamped",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::TwistStampedDisplay>(config.channel),
            config);
      },
      [] { return MakeDefault("TwistStamped", "TwistStamped", ""); });

  registerType(
      "CameraInfo",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::CameraInfoDisplay>(config.channel),
            config);
      },
      [] { return MakeDefault("CameraInfo", "CameraInfo", ""); });

  registerType(
      "DepthCloud",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::DepthCloudDisplay>(config.channel),
            config);
      },
      []() {
        auto config = MakeDefault("DepthCloud", "DepthCloud", "");
        config.properties["camera_info_channel"] = "";
        return config;
      });

  registerType(
      "AccelStamped",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::AccelStampedDisplay>(config.channel),
            config);
      },
      [] { return MakeDefault("AccelStamped", "AccelStamped", ""); });

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
      [] { return MakeDefault("Imu", "Imu", ""); });

  registerType(
      "Temperature",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::TemperatureDisplay>(config.channel),
            config);
      },
      [] { return MakeDefault("Temperature", "Temperature", ""); });

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
      [] { return MakeDefault("Illuminance", "Illuminance", ""); });

  registerType(
      "FluidPressure",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::FluidPressureDisplay>(config.channel),
            config);
      },
      [] { return MakeDefault("FluidPressure", "FluidPressure", ""); });

  registerType(
      "RelativeHumidity",
      [](const DisplayConfig& config) {
        return FinalizeDisplay(
            std::make_unique<display::RelativeHumidityDisplay>(config.channel),
            config);
      },
      [] {
        return MakeDefault("RelativeHumidity", "RelativeHumidity", "");
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
