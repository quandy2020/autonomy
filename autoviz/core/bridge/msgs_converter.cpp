/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autoviz/core/bridge/msgs_converter.hpp"

#include <algorithm>
#include <cctype>
#include <unordered_map>

namespace autoviz {

namespace {

std::string ToLowerCopy(std::string s) {
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

std::string ExtractShortType(const std::string& msg_type) {
  if (msg_type.empty()) {
    return "";
  }
  const auto pos = msg_type.find_last_of('.');
  if (pos == std::string::npos || pos + 1 >= msg_type.size()) {
    return msg_type;
  }
  return msg_type.substr(pos + 1);
}

}  // namespace

std::string MapSchemaNameToRos2Style(const std::string& msg_type) {
  if (msg_type.empty()) {
    return "unknown";
  }

  const std::string lower = ToLowerCopy(msg_type);
  const std::string short_type = ExtractShortType(msg_type);

  static const std::unordered_map<std::string, std::string> kSensorMsgsMap = {
      {"Image", "sensor_msgs/msg/Image"},
      {"CompressedImage", "sensor_msgs/msg/CompressedImage"},
      {"PointCloud2", "sensor_msgs/msg/PointCloud2"},
      {"PointCloud", "sensor_msgs/msg/PointCloud"},
      {"Imu", "sensor_msgs/msg/Imu"},
      {"LaserScan", "sensor_msgs/msg/LaserScan"},
      {"Range", "sensor_msgs/msg/Range"},
      {"CameraInfo", "sensor_msgs/msg/CameraInfo"},
      {"NavSatFix", "sensor_msgs/msg/NavSatFix"},
      {"MagneticField", "sensor_msgs/msg/MagneticField"},
      {"JointState", "sensor_msgs/msg/JointState"},
      {"Temperature", "sensor_msgs/msg/Temperature"},
      {"RelativeHumidity", "sensor_msgs/msg/RelativeHumidity"},
      {"FluidPressure", "sensor_msgs/msg/FluidPressure"},
      {"BatteryState", "sensor_msgs/msg/BatteryState"},
      {"TimeReference", "sensor_msgs/msg/TimeReference"},
      {"MultiEchoLaserScan", "sensor_msgs/msg/MultiEchoLaserScan"},
      {"MultiDOFJointState", "sensor_msgs/msg/MultiDOFJointState"},
      {"ChannelFloat32", "sensor_msgs/msg/ChannelFloat32"},
      {"RegionOfInterest", "sensor_msgs/msg/RegionOfInterest"},
      {"Illuminance", "sensor_msgs/msg/Illuminance"},
      {"Joy", "sensor_msgs/msg/Joy"},
      {"JoyFeedback", "sensor_msgs/msg/JoyFeedback"},
      {"JoyFeedbackArray", "sensor_msgs/msg/JoyFeedbackArray"},
      {"NavSatStatus", "sensor_msgs/msg/NavSatStatus"},
      {"PointField", "sensor_msgs/msg/PointField"},
      {"LaserEcho", "sensor_msgs/msg/LaserEcho"},
  };
  if (lower.find("sensor_msgs") != std::string::npos) {
    const auto it = kSensorMsgsMap.find(short_type);
    return it == kSensorMsgsMap.end() ? msg_type : it->second;
  }

  static const std::unordered_map<std::string, std::string> kGeometryMsgsMap = {
      {"Point", "geometry_msgs/msg/Point"},
      {"Quaternion", "geometry_msgs/msg/Quaternion"},
      {"Vector3", "geometry_msgs/msg/Vector3"},
      {"Pose", "geometry_msgs/msg/Pose"},
      {"TwistWithCovariance", "geometry_msgs/msg/TwistWithCovariance"},
      {"PoseWithCovariance", "geometry_msgs/msg/PoseWithCovariance"},
      {"PoseStamped", "geometry_msgs/msg/PoseStamped"},
      {"PoseArray", "geometry_msgs/msg/PoseArray"},
      {"PoseWithCovarianceStamped", "geometry_msgs/msg/PoseWithCovarianceStamped"},
      {"TransformStamped", "geometry_msgs/msg/TransformStamped"},
      {"Twist", "geometry_msgs/msg/Twist"},
      {"TwistStamped", "geometry_msgs/msg/TwistStamped"},
      {"TwistWithCovarianceStamped", "geometry_msgs/msg/TwistWithCovarianceStamped"},
      {"PointStamped", "geometry_msgs/msg/PointStamped"},
      {"Vector3Stamped", "geometry_msgs/msg/Vector3Stamped"},
      {"QuaternionStamped", "geometry_msgs/msg/QuaternionStamped"},
      {"Accel", "geometry_msgs/msg/Accel"},
      {"AccelStamped", "geometry_msgs/msg/AccelStamped"},
      {"Wrench", "geometry_msgs/msg/Wrench"},
      {"WrenchStamped", "geometry_msgs/msg/WrenchStamped"},
      {"PolygonStamped", "geometry_msgs/msg/PolygonStamped"},
  };
  if (lower.find("geometry_msgs") != std::string::npos) {
    const auto it = kGeometryMsgsMap.find(short_type);
    return it == kGeometryMsgsMap.end() ? msg_type : it->second;
  }

  if (lower.find("map_msgs") != std::string::npos || lower.find("nav_msgs") != std::string::npos) {
    if (short_type == "OccupancyGrid") return "nav_msgs/msg/OccupancyGrid";
    if (short_type == "Path") return "nav_msgs/msg/Path";
    if (short_type == "Odometry") return "nav_msgs/msg/Odometry";
    return msg_type;
  }

  if (lower.find("planning_msgs") != std::string::npos) {
    if (short_type == "Path") return "nav_msgs/msg/Path";
    if (short_type == "Odometry") return "nav_msgs/msg/Odometry";
    return msg_type;
  }

  if (lower.find("visualization_msgs") != std::string::npos) {
    if (short_type == "Marker") return "visualization_msgs/msg/Marker";
    if (short_type == "MarkerArray") return "visualization_msgs/msg/MarkerArray";
    return msg_type;
  }

  if (lower.find("diagnostic_msgs") != std::string::npos) {
    if (short_type == "DiagnosticArray") return "diagnostic_msgs/msg/DiagnosticArray";
    if (short_type == "DiagnosticStatus") return "diagnostic_msgs/msg/DiagnosticStatus";
    return msg_type;
  }

  if (lower.find("std_msgs") != std::string::npos) {
    if (short_type == "String") return "std_msgs/msg/String";
    if (short_type == "Header") return "std_msgs/msg/Header";
    if (short_type == "Bool") return "std_msgs/msg/Bool";
    if (short_type == "Int32") return "std_msgs/msg/Int32";
    if (short_type == "Int64") return "std_msgs/msg/Int64";
    if (short_type == "UInt32") return "std_msgs/msg/UInt32";
    if (short_type == "UInt64") return "std_msgs/msg/UInt64";
    if (short_type == "Float32") return "std_msgs/msg/Float32";
    if (short_type == "Float64") return "std_msgs/msg/Float64";
    return msg_type;
  }

  return msg_type;
}

}  // namespace autoviz