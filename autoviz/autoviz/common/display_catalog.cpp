/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/display_catalog.hpp"

#include <algorithm>

#include "autoviz/common/display_factory.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"

namespace autoviz {
namespace common {
namespace {

DisplayTypeInfo MakeInfo(const char* type, const char* package,
                         const char* description,
                         std::initializer_list<const char*> message_types = {}) {
  DisplayTypeInfo info;
  info.type = type;
  info.package = package;
  info.description = description;
  for (const char* message_type : message_types) {
    info.message_types.emplace_back(message_type);
  }
  return info;
}

const std::vector<DisplayTypeInfo>& BuiltinCatalog() {
  static const std::vector<DisplayTypeInfo> kCatalog = {
      MakeInfo("Grid", "autoviz", "A planar grid in the XY plane."),
      MakeInfo("Axes", "autoviz", "Displays the origin of the target frame as axes."),
      MakeInfo("TF", "autoviz",
               "Shows the TF transform tree from the shared buffer "
               "(/tf and /tf_static).",
               {"automsgs.msgs.tf2_msgs.TFMessage", "tf2_msgs.TFMessage"}),
      MakeInfo("LaserScan", "autoviz", "Shows a sensor_msgs/LaserScan message.",
               {"automsgs.msgs.sensor_msgs.LaserScan", "sensor_msgs.LaserScan"}),
      MakeInfo("PointCloud2", "autoviz",
               "Shows a sensor_msgs/PointCloud2 message.",
               {"automsgs.msgs.sensor_msgs.PointCloud2",
                "sensor_msgs.PointCloud2"}),
      MakeInfo("Image", "autoviz", "Shows an image from sensor_msgs/Image.",
               {"automsgs.msgs.sensor_msgs.Image", "sensor_msgs.Image"}),
      MakeInfo("Camera", "autoviz", "Shows a camera image and frustum.",
               {"automsgs.msgs.sensor_msgs.Image", "sensor_msgs.Image"}),
      MakeInfo("Map", "autoviz", "Shows a map from map_msgs/OccupancyGrid.",
               {"automsgs.msgs.map_msgs.OccupancyGrid",
                "map_msgs.OccupancyGrid"}),
      MakeInfo("GridMap", "autoviz",
               "Shows map_msgs/GridMap (mesh / point_cloud / occupancy_grid / "
               "grid_cells / map_region / vectors — grid_map_visualization + "
               "grid_map_rviz_plugin).",
               {"automsgs.msgs.map_msgs.GridMap", "map_msgs.GridMap",
                "grid_map_msgs.GridMap"}),
      MakeInfo("Path", "autoviz", "Shows a nav_msgs/Path message.",
               {"automsgs.msgs.nav_msgs.Path", "nav_msgs.Path"}),
      MakeInfo("Odometry", "autoviz", "Shows nav_msgs/Odometry as axes and path.",
               {"automsgs.msgs.nav_msgs.Odometry", "nav_msgs.Odometry"}),
      MakeInfo("Marker", "autoviz", "Shows visualization_msgs/Marker.",
               {"automsgs.msgs.visualization_msgs.Marker",
                "visualization_msgs.Marker"}),
      MakeInfo("MarkerArray", "autoviz", "Shows visualization_msgs/MarkerArray.",
               {"automsgs.msgs.visualization_msgs.MarkerArray",
                "visualization_msgs.MarkerArray"}),
      MakeInfo("Pose", "autoviz", "Shows geometry_msgs/PoseStamped.",
               {"automsgs.msgs.geometry_msgs.PoseStamped",
                "geometry_msgs.PoseStamped"}),
      MakeInfo("PoseArray", "autoviz", "Shows geometry_msgs/PoseArray.",
               {"automsgs.msgs.geometry_msgs.PoseArray",
                "geometry_msgs.PoseArray"}),
      MakeInfo("RobotModel", "autoviz", "Shows a URDF robot model.",
               {"automsgs.msgs.sensor_msgs.JointState",
                "sensor_msgs.JointState"}),
      MakeInfo("Effort", "autoviz", "Shows joint efforts on a robot model.",
               {"automsgs.msgs.sensor_msgs.JointState",
                "sensor_msgs.JointState"}),
      MakeInfo("InteractiveMarkers", "autoviz",
               "Shows interactive markers and forwards feedback.",
               {"automsgs.msgs.visualization_msgs.InteractiveMarkerUpdate",
                "visualization_msgs.InteractiveMarkerUpdate"}),
      MakeInfo("Wrench", "autoviz", "Shows geometry_msgs/WrenchStamped.",
               {"automsgs.msgs.geometry_msgs.WrenchStamped",
                "geometry_msgs.WrenchStamped"}),
      MakeInfo("GridCells", "autoviz", "Shows map_msgs/GridCells.",
               {"automsgs.msgs.map_msgs.GridCells", "map_msgs.GridCells"}),
      MakeInfo("PointStamped", "autoviz", "Shows geometry_msgs/PointStamped.",
               {"automsgs.msgs.geometry_msgs.PointStamped",
                "geometry_msgs.PointStamped"}),
      MakeInfo("Polygon", "autoviz", "Shows geometry_msgs/PolygonStamped.",
               {"automsgs.msgs.geometry_msgs.PolygonStamped",
                "geometry_msgs.PolygonStamped"}),
      MakeInfo("Range", "autoviz", "Shows sensor_msgs/Range.",
               {"automsgs.msgs.sensor_msgs.Range", "sensor_msgs.Range"}),
      MakeInfo("PoseWithCovariance", "autoviz",
               "Shows geometry_msgs/PoseWithCovarianceStamped.",
               {"automsgs.msgs.geometry_msgs.PoseWithCovarianceStamped",
                "geometry_msgs.PoseWithCovarianceStamped"}),
      MakeInfo("TwistStamped", "autoviz", "Shows geometry_msgs/TwistStamped.",
               {"automsgs.msgs.geometry_msgs.TwistStamped",
                "geometry_msgs.TwistStamped"}),
      MakeInfo("CameraInfo", "autoviz", "Shows sensor_msgs/CameraInfo frustum.",
               {"automsgs.msgs.sensor_msgs.CameraInfo",
                "sensor_msgs.CameraInfo"}),
      MakeInfo("DepthCloud", "autoviz",
               "Projects depth image + CameraInfo into a point cloud.",
               {"automsgs.msgs.sensor_msgs.Image", "sensor_msgs.Image"}),
      MakeInfo("AccelStamped", "autoviz", "Shows geometry_msgs/AccelStamped.",
               {"automsgs.msgs.geometry_msgs.AccelStamped",
                "geometry_msgs.AccelStamped"}),
      MakeInfo("Imu", "autoviz", "Shows sensor_msgs/Imu.",
               {"automsgs.msgs.sensor_msgs.Imu", "sensor_msgs.Imu"}),
      MakeInfo("Temperature", "autoviz", "Shows sensor_msgs/Temperature.",
               {"automsgs.msgs.sensor_msgs.Temperature",
                "sensor_msgs.Temperature"}),
      MakeInfo("Illuminance", "autoviz", "Shows sensor_msgs/Illuminance.",
               {"automsgs.msgs.sensor_msgs.Illuminance",
                "sensor_msgs.Illuminance"}),
      MakeInfo("FluidPressure", "autoviz", "Shows sensor_msgs/FluidPressure.",
               {"automsgs.msgs.sensor_msgs.FluidPressure",
                "sensor_msgs.FluidPressure"}),
      MakeInfo("RelativeHumidity", "autoviz",
               "Shows sensor_msgs/RelativeHumidity.",
               {"automsgs.msgs.sensor_msgs.RelativeHumidity",
                "sensor_msgs.RelativeHumidity"}),
      MakeInfo("Group", "autoviz", "Container for nested displays.", {}),
  };
  return kCatalog;
}

bool MessageTypeMatches(const std::string& channel_type,
                        const std::vector<std::string>& display_types) {
  for (const auto& candidate : display_types) {
    if (commsgs::MessageTypesCompatible(channel_type, candidate)) {
      return true;
    }
  }
  return false;
}

}  // namespace

std::vector<DisplayTypeInfo> DisplayCatalog::allTypes() {
  std::vector<DisplayTypeInfo> types = BuiltinCatalog();
  const auto registered = DisplayFactory::supportedTypes();
  for (const std::string& type : registered) {
    if (std::any_of(types.begin(), types.end(),
                    [&type](const DisplayTypeInfo& info) {
                      return info.type == type;
                    })) {
      continue;
    }
    types.push_back(MakeInfo(type.c_str(), "autoviz", "Display plugin."));
  }
  std::sort(types.begin(), types.end(),
            [](const DisplayTypeInfo& a, const DisplayTypeInfo& b) {
              if (a.package != b.package) {
                return a.package < b.package;
              }
              return a.type < b.type;
            });
  return types;
}

DisplayTypeInfo DisplayCatalog::infoForType(const std::string& type) {
  for (const auto& info : BuiltinCatalog()) {
    if (info.type == type) {
      return info;
    }
  }
  DisplayTypeInfo fallback;
  fallback.type = type;
  fallback.package = "autoviz";
  fallback.description = "Display plugin.";
  return fallback;
}

std::vector<std::string> DisplayCatalog::typesForMessageType(
    const std::string& message_type) {
  std::vector<std::string> matches;
  for (const auto& info : allTypes()) {
    if (info.message_types.empty()) {
      continue;
    }
    for (const auto& candidate : info.message_types) {
      if (commsgs::MessageTypesCompatible(message_type, candidate)) {
        matches.push_back(info.type);
        break;
      }
    }
  }
  std::sort(matches.begin(), matches.end());
  matches.erase(std::unique(matches.begin(), matches.end()), matches.end());
  return matches;
}

bool DisplayCatalog::isStaticTfChannel(const std::string& channel) {
  std::string name = channel;
  while (!name.empty() && name.front() == '/') {
    name.erase(name.begin());
  }
  // ROS uses /tf_static; some stacks publish /static_tf.
  return name == "tf_static" || name == "static_tf";
}

std::vector<std::string> DisplayCatalog::typesForChannel(
    const std::string& channel_name, const std::string& message_type) {
  std::vector<std::string> matches = typesForMessageType(message_type);
  if (isStaticTfChannel(channel_name)) {
    matches.erase(std::remove(matches.begin(), matches.end(), std::string("TF")),
                  matches.end());
  }
  // URDF is a std_msgs/String. Bind it to RobotModel only when the channel
  // name is robot_description, the same topic RViz2 RobotModel subscribes to.
  const auto slash = channel_name.find_last_of('/');
  const std::string base = slash == std::string::npos
                               ? channel_name
                               : channel_name.substr(slash + 1);
  const bool string_payload = message_type.empty() ||
                              message_type.find("String") != std::string::npos;
  if (base == "robot_description" && string_payload &&
      std::find(matches.begin(), matches.end(), std::string("RobotModel")) ==
          matches.end()) {
    matches.push_back("RobotModel");
  }
  return matches;
}

}  // namespace common
}  // namespace autoviz
