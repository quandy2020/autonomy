/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autolink/time/rate.hpp"
#include "autonomy/commsgs/proto/diagnostic_msgs.pb.h"
#include "autonomy/commsgs/proto/geometry_msgs.pb.h"
#include "autonomy/commsgs/proto/map_msgs.pb.h"
#include "autonomy/commsgs/proto/planning_msgs.pb.h"
#include "autonomy/commsgs/proto/sensor_msgs.pb.h"
#include "autonomy/commsgs/proto/tf2_msgs.pb.h"
#include "autonomy/commsgs/proto/vehicle_msgs.pb.h"
#include "autonomy/commsgs/proto/vision_msgs.pb.h"
#include "autonomy/commsgs/proto/visualization_msgs.pb.h"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/utils/occ_grid_values.hpp"

DEFINE_double(rate_hz, 50.0,
              "Publish rate (Hz) for dynamic topics only (scan/image/imu). "
              "Static TF/maps/scene layers are published once at startup.");
DEFINE_double(
    point_cloud2_sync_hz, 10.0,
    "Republish /fake/point_cloud2 at this rate with a fresh timestamp so "
    "Foxglove live Time sync keeps the cloud visible (geometry unchanged).");
DEFINE_string(channel_prefix, "/fake",
              "Prefix for all fake data channels, e.g. /fake/scan.");
DEFINE_string(tf_channel, "/tf",
              "TF topic (tf2_msgs.TFMessage). Foxglove 3D expects /tf.");

namespace {

namespace map_utils = autonomy::map::costmap_2d::utils;
namespace costmap = autonomy::map::costmap_2d;

namespace builtin_interfaces = autonomy::commsgs::proto::builtin_interfaces;
namespace diagnostic_msgs = autonomy::commsgs::proto::diagnostic_msgs;
namespace geometry_msgs = autonomy::commsgs::proto::geometry_msgs;
namespace map_msgs = autonomy::commsgs::proto::map_msgs;
namespace planning_msgs = autonomy::commsgs::proto::planning_msgs;
namespace sensor_msgs = autonomy::commsgs::proto::sensor_msgs;
namespace std_msgs = autonomy::commsgs::proto::std_msgs;
namespace tf2_msgs = autonomy::commsgs::proto::tf2_msgs;
namespace vehicle_msgs = autonomy::commsgs::proto::vehicle_msgs;
namespace vision_msgs = autonomy::commsgs::proto::vision_msgs;
namespace visualization_msgs = autonomy::commsgs::proto::visualization_msgs;

constexpr int32_t kMarkerAdd = 0;
constexpr int32_t kMarkerSphere = 2;
constexpr int32_t kMarkerLineStrip = 4;
constexpr uint32_t kPointFieldFloat32 = 7;

struct MotionState {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double phase = 0.0;
};

std::string Channel(const std::string& prefix, const std::string& name) {
  if (prefix.empty()) {
    return name;
  }
  if (prefix.back() == '/') {
    return prefix + name;
  }
  return prefix + "/" + name;
}

void FillHeader(std_msgs::Header* header, const std::string& frame_id,
                uint64_t stamp_ns) {
  const uint64_t sec = stamp_ns / 1000000000ULL;
  const uint32_t nsec = static_cast<uint32_t>(stamp_ns % 1000000000ULL);
  header->mutable_stamp()->set_sec(static_cast<int32_t>(sec));
  header->mutable_stamp()->set_nanosec(nsec);
  header->set_frame_id(frame_id);
}

void FillTime(builtin_interfaces::Time* stamp, uint64_t stamp_ns) {
  const uint64_t sec = stamp_ns / 1000000000ULL;
  const uint32_t nsec = static_cast<uint32_t>(stamp_ns % 1000000000ULL);
  stamp->set_sec(static_cast<int32_t>(sec));
  stamp->set_nanosec(nsec);
}

geometry_msgs::Quaternion YawQuaternion(double yaw) {
  geometry_msgs::Quaternion q;
  q.set_x(0.0);
  q.set_y(0.0);
  q.set_z(std::sin(yaw * 0.5));
  q.set_w(std::cos(yaw * 0.5));
  return q;
}

MotionState ComputeMotion(uint64_t seq) {
  const double phase = 0.2 * static_cast<double>(seq);
  constexpr double kRadius = 2.0;
  return {kRadius * std::cos(phase), kRadius * std::sin(phase),
          phase + M_PI_2, phase};
}

void AppendFloat(google::protobuf::RepeatedField<uint32_t>* data, float value) {
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(float));
  data->Add(bits);
}

geometry_msgs::TransformStamped MakeTransform(const std::string& parent,
                                              const std::string& child,
                                              uint64_t stamp_ns, double x,
                                              double y, double z = 0.0,
                                              double yaw = 0.0) {
  geometry_msgs::TransformStamped tf;
  FillHeader(tf.mutable_header(), parent, stamp_ns);
  tf.set_child_frame_id(child);
  tf.mutable_transform()->mutable_translation()->set_x(x);
  tf.mutable_transform()->mutable_translation()->set_y(y);
  tf.mutable_transform()->mutable_translation()->set_z(z);
  *tf.mutable_transform()->mutable_rotation() = YawQuaternion(yaw);
  return tf;
}

visualization_msgs::Marker MakeRobotMarker(uint64_t stamp_ns, double x,
                                           double y) {
  visualization_msgs::Marker marker;
  FillHeader(marker.mutable_header(), "map", stamp_ns);
  marker.set_ns("fake");
  marker.set_id(0);
  marker.set_type(kMarkerSphere);
  marker.set_action(kMarkerAdd);
  marker.mutable_pose()->mutable_position()->set_x(x);
  marker.mutable_pose()->mutable_position()->set_y(y);
  marker.mutable_pose()->mutable_position()->set_z(0.3);
  marker.mutable_pose()->mutable_orientation()->set_w(1.0);
  marker.mutable_scale()->set_x(0.4);
  marker.mutable_scale()->set_y(0.4);
  marker.mutable_scale()->set_z(0.4);
  marker.mutable_color()->set_r(0.2f);
  marker.mutable_color()->set_g(0.6f);
  marker.mutable_color()->set_b(1.0f);
  marker.mutable_color()->set_a(1.0f);
  return marker;
}

visualization_msgs::Marker MakePathMarker(uint64_t stamp_ns) {
  visualization_msgs::Marker marker;
  FillHeader(marker.mutable_header(), "map", stamp_ns);
  marker.set_ns("fake");
  marker.set_id(1);
  marker.set_type(kMarkerLineStrip);
  marker.set_action(kMarkerAdd);
  marker.mutable_scale()->set_x(0.05);
  marker.mutable_color()->set_r(1.0f);
  marker.mutable_color()->set_g(0.4f);
  marker.mutable_color()->set_b(0.1f);
  marker.mutable_color()->set_a(1.0f);

  constexpr int kSamples = 64;
  constexpr double kRadius = 2.0;
  for (int i = 0; i <= kSamples; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / kSamples;
    auto* point = marker.add_points();
    point->set_x(kRadius * std::cos(angle));
    point->set_y(kRadius * std::sin(angle));
    point->set_z(0.05);
  }
  return marker;
}

tf2_msgs::TFMessage MakeTfTree(uint64_t stamp_ns) {
  tf2_msgs::TFMessage msg;
  // Static TF tree for stable 3D visualization.
  *msg.add_transforms() =
      MakeTransform("map", "odom", stamp_ns, 0.0, 0.0);
  *msg.add_transforms() =
      MakeTransform("odom", "base_link", stamp_ns, 0.0, 0.0);
  *msg.add_transforms() =
      MakeTransform("base_link", "laser", stamp_ns, 0.2, 0.0);
  *msg.add_transforms() =
      MakeTransform("base_link", "camera", stamp_ns, 0.15, 0.0, 0.4);
  return msg;
}

geometry_msgs::TransformStamped MakeSingleTransform(uint64_t stamp_ns,
                                                    const MotionState& /*motion*/) {
  // Keep in sync with MakeTfTree(); do not animate TF here.
  return MakeTransform("odom", "base_link", stamp_ns, 0.0, 0.0);
}

geometry_msgs::PoseStamped MakeStaticPoseStamped(uint64_t stamp_ns) {
  geometry_msgs::PoseStamped pose;
  FillHeader(pose.mutable_header(), "map", stamp_ns);
  pose.mutable_pose()->mutable_orientation()->set_w(1.0);
  return pose;
}

geometry_msgs::PointStamped MakeStaticPointStamped(uint64_t stamp_ns) {
  geometry_msgs::PointStamped point;
  FillHeader(point.mutable_header(), "map", stamp_ns);
  point.mutable_point()->set_z(0.1);
  return point;
}

geometry_msgs::PoseArray MakePoseArray(uint64_t stamp_ns, double phase) {
  geometry_msgs::PoseArray array;
  FillHeader(array.mutable_header(), "map", stamp_ns);
  for (int i = 0; i < 8; ++i) {
    const double angle = phase + 2.0 * M_PI * static_cast<double>(i) / 8.0;
    auto* pose = array.add_poses();
    pose->mutable_position()->set_x(std::cos(angle));
    pose->mutable_position()->set_y(std::sin(angle));
    *pose->mutable_orientation() = YawQuaternion(angle);
  }
  return array;
}

geometry_msgs::PolygonStamped MakePolygonStamped(uint64_t stamp_ns) {
  geometry_msgs::PolygonStamped polygon;
  FillHeader(polygon.mutable_header(), "map", stamp_ns);
  constexpr int kCorners = 4;
  constexpr double kSize = 0.8;
  for (int i = 0; i < kCorners; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / kCorners;
    auto* point = polygon.mutable_polygon()->add_points();
    point->set_x(kSize * std::cos(angle));
    point->set_y(kSize * std::sin(angle));
    point->set_z(0.0f);
  }
  return polygon;
}

visualization_msgs::MarkerArray MakeMarkerArray(uint64_t stamp_ns,
                                                const MotionState& /*motion*/) {
  visualization_msgs::MarkerArray array;
  *array.add_markers() = MakeRobotMarker(stamp_ns, 0.0, 0.0);
  *array.add_markers() = MakePathMarker(stamp_ns);
  return array;
}

planning_msgs::Path MakePath(uint64_t stamp_ns) {
  planning_msgs::Path path;
  FillHeader(path.mutable_header(), "map", stamp_ns);
  constexpr int kSamples = 32;
  constexpr double kRadius = 2.0;
  for (int i = 0; i <= kSamples; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / kSamples;
    auto* pose = path.add_poses();
    FillHeader(pose->mutable_header(), "map", stamp_ns);
    pose->mutable_pose()->mutable_position()->set_x(kRadius * std::cos(angle));
    pose->mutable_pose()->mutable_position()->set_y(kRadius * std::sin(angle));
    *pose->mutable_pose()->mutable_orientation() = YawQuaternion(angle + M_PI_2);
  }
  return path;
}

planning_msgs::Odometry MakeOdometry(uint64_t stamp_ns,
                                     const MotionState& motion) {
  planning_msgs::Odometry odom;
  FillHeader(odom.mutable_header(), "odom", stamp_ns);
  odom.set_child_frame_id("base_link");
  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(1.0);
  odom.mutable_twist()->mutable_twist()->mutable_linear()->set_x(
      -2.0 * std::sin(motion.phase) * 0.2);
  odom.mutable_twist()->mutable_twist()->mutable_linear()->set_y(
      2.0 * std::cos(motion.phase) * 0.2);
  odom.mutable_twist()->mutable_twist()->mutable_angular()->set_z(0.2);
  return odom;
}

sensor_msgs::LaserScan MakeLaserScan(uint64_t stamp_ns, double phase) {
  sensor_msgs::LaserScan scan;
  FillHeader(scan.mutable_header(), "laser", stamp_ns);

  constexpr int kBeams = 360;
  constexpr float kAngleMin = -static_cast<float>(M_PI);
  constexpr float kAngleMax = static_cast<float>(M_PI);
  const float angle_increment =
      (kAngleMax - kAngleMin) / static_cast<float>(kBeams - 1);

  scan.set_angle_min(kAngleMin);
  scan.set_angle_max(kAngleMax);
  scan.set_angle_increment(angle_increment);
  scan.set_scan_time(0.1f);
  scan.set_range_min(0.1f);
  scan.set_range_max(10.0f);

  for (int i = 0; i < kBeams; ++i) {
    const float angle = kAngleMin + angle_increment * static_cast<float>(i);
    scan.add_ranges(3.0f + 0.8f * std::sin(3.0f * angle + static_cast<float>(phase)));
    scan.add_intensities(100.0f + 50.0f * std::cos(angle));
  }
  return scan;
}

sensor_msgs::MultiEchoLaserScan MakeMultiEchoLaserScan(uint64_t stamp_ns,
                                                       double phase) {
  sensor_msgs::MultiEchoLaserScan scan;
  FillHeader(scan.mutable_header(), "laser", stamp_ns);
  scan.set_angle_min(-static_cast<float>(M_PI));
  scan.set_angle_max(static_cast<float>(M_PI));
  scan.set_angle_increment(static_cast<float>(M_PI) / 180.0f);
  scan.set_scan_time(0.1f);
  scan.set_range_min(0.1f);
  scan.set_range_max(10.0f);

  for (int i = 0; i < 180; ++i) {
    const float angle = scan.angle_min() + scan.angle_increment() * i;
    const float range =
        3.0f + 0.5f * std::sin(static_cast<float>(phase) + angle);
    auto* range_echo = scan.add_ranges();
    range_echo->add_echoes(range);
    range_echo->add_echoes(range + 0.1f);
    scan.add_intensities()->add_echoes(80.0f);
  }
  return scan;
}

void AppendPointCloudPoint(google::protobuf::RepeatedField<uint32_t>* data,
                           float x, float y, float z, float intensity) {
  AppendFloat(data, x);
  AppendFloat(data, y);
  AppendFloat(data, z);
  AppendFloat(data, intensity);
}

void AppendCubeVolumePoints(google::protobuf::RepeatedField<uint32_t>* data,
                            float center_x, float center_y, float center_z,
                            float half_size, int samples_per_axis,
                            float intensity) {
  if (samples_per_axis < 2) {
    return;
  }
  const float step =
      (2.0f * half_size) / static_cast<float>(samples_per_axis - 1);
  for (int iz = 0; iz < samples_per_axis; ++iz) {
    for (int iy = 0; iy < samples_per_axis; ++iy) {
      for (int ix = 0; ix < samples_per_axis; ++ix) {
        const float x =
            center_x - half_size + static_cast<float>(ix) * step;
        const float y =
            center_y - half_size + static_cast<float>(iy) * step;
        const float z =
            center_z - half_size + static_cast<float>(iz) * step;
        AppendPointCloudPoint(data, x, y, z, intensity);
      }
    }
  }
}

sensor_msgs::PointCloud2 MakePointCloud2(uint64_t stamp_ns) {
  sensor_msgs::PointCloud2 cloud;
  FillHeader(cloud.mutable_header(), "map", stamp_ns);

  // 18 solid cubes; 8^3 points/cube ~= 9k points (stable at ~10 Hz resync).
  constexpr int kGrid = 3;
  constexpr int kMiddleLayer = 1;
  constexpr float kCubeHalfSize = 0.22f;
  constexpr float kCellPitch = 0.55f;
  constexpr int kSamplesPerAxis = 8;
  constexpr int kFilledCubes = kGrid * kGrid * kGrid - kGrid * kGrid;
  constexpr int kPointsPerCube = kSamplesPerAxis * kSamplesPerAxis * kSamplesPerAxis;
  constexpr uint32_t kPointCount =
      static_cast<uint32_t>(kFilledCubes * kPointsPerCube);

  cloud.set_height(1);
  cloud.set_width(kPointCount);
  cloud.set_point_step(16);
  cloud.set_row_step(cloud.point_step() * cloud.width());
  cloud.set_is_dense(true);

  auto add_field = [&cloud](const std::string& name, uint32_t offset) {
    auto* field = cloud.add_fields();
    field->set_name(name);
    field->set_offset(offset);
    field->set_datatype(kPointFieldFloat32);
    field->set_count(1);
  };
  add_field("x", 0);
  add_field("y", 4);
  add_field("z", 8);
  add_field("intensity", 12);

  cloud.mutable_data()->Reserve(kPointCount * 4);

  for (int iz = 0; iz < kGrid; ++iz) {
    if (iz == kMiddleLayer) {
      continue;
    }
    for (int iy = 0; iy < kGrid; ++iy) {
      for (int ix = 0; ix < kGrid; ++ix) {
        const float center_x = (static_cast<float>(ix) - 1.0f) * kCellPitch;
        const float center_y = (static_cast<float>(iy) - 1.0f) * kCellPitch;
        const float center_z = (static_cast<float>(iz) - 1.0f) * kCellPitch;
        const float intensity =
            80.0f + 20.0f * static_cast<float>(ix + iy + iz);
        AppendCubeVolumePoints(cloud.mutable_data(), center_x, center_y,
                               center_z, kCubeHalfSize, kSamplesPerAxis,
                               intensity);
      }
    }
  }
  return cloud;
}

sensor_msgs::PointCloud MakePointCloud(uint64_t stamp_ns, double phase) {
  sensor_msgs::PointCloud cloud;
  FillHeader(cloud.mutable_header(), "base_link", stamp_ns);
  for (int i = 0; i < 24; ++i) {
    const float t = static_cast<float>(i) / 24.0f;
    auto* point = cloud.add_points();
    point->set_x(1.5f * std::cos(t + static_cast<float>(phase)));
    point->set_y(1.5f * std::sin(t + static_cast<float>(phase)));
    point->set_z(0.05f * static_cast<float>(i));
    point->set_intensity(static_cast<uint32_t>(50 + i * 5));
    FillTime(point->mutable_timestamp(), stamp_ns);
  }
  return cloud;
}

sensor_msgs::Range MakeRange(uint64_t stamp_ns, double /*phase*/) {
  sensor_msgs::Range range;
  FillHeader(range.mutable_header(), "base_link", stamp_ns);
  range.set_radiation_type(0);
  range.set_field_of_view(0.5f);
  range.set_min_range(0.1f);
  range.set_max_range(5.0f);
  range.set_range(2.0f);
  return range;
}

sensor_msgs::Image MakeImage(uint64_t stamp_ns, uint64_t seq) {
  sensor_msgs::Image image;
  FillHeader(image.mutable_header(), "camera", stamp_ns);
  image.set_height(64);
  image.set_width(64);
  image.set_encoding("rgb8");
  image.set_step(image.width() * 3);
  std::string pixels(image.step() * image.height(), '\0');
  for (uint32_t y = 0; y < image.height(); ++y) {
    for (uint32_t x = 0; x < image.width(); ++x) {
      const size_t idx = y * image.step() + x * 3;
      pixels[idx] = static_cast<char>((x + seq) % 256);
      pixels[idx + 1] = static_cast<char>((y + seq) % 256);
      pixels[idx + 2] = static_cast<char>(128);
    }
  }
  image.set_data(pixels);
  return image;
}

sensor_msgs::CompressedImage MakeCompressedImage(uint64_t stamp_ns,
                                                 uint64_t seq) {
  sensor_msgs::CompressedImage image;
  FillHeader(image.mutable_header(), "camera", stamp_ns);
  image.set_format("rgb8");
  constexpr uint32_t kWidth = 64;
  constexpr uint32_t kHeight = 64;
  std::string pixels(kWidth * kHeight * 3, '\0');
  for (uint32_t y = 0; y < kHeight; ++y) {
    for (uint32_t x = 0; x < kWidth; ++x) {
      const size_t idx = (static_cast<size_t>(y) * kWidth + x) * 3;
      pixels[idx] = static_cast<char>((x + seq) % 256);
      pixels[idx + 1] = static_cast<char>((y + seq) % 256);
      pixels[idx + 2] = static_cast<char>((x + y + seq) % 256);
    }
  }
  image.set_data(pixels);
  return image;
}

sensor_msgs::CameraInfo MakeCameraInfo(uint64_t stamp_ns) {
  sensor_msgs::CameraInfo info;
  FillHeader(info.mutable_header(), "camera", stamp_ns);
  info.set_height(64);
  info.set_width(64);
  info.set_distortion_model("plumb_bob");
  info.add_d(0.0);
  info.add_d(0.0);
  info.add_d(0.0);
  info.add_d(0.0);
  info.add_d(0.0);
  info.add_k(320.0);
  info.add_k(0.0);
  info.add_k(32.0);
  info.add_k(0.0);
  info.add_k(320.0);
  info.add_k(32.0);
  info.add_k(0.0);
  info.add_k(0.0);
  info.add_k(1.0);
  info.add_p(320.0);
  info.add_p(0.0);
  info.add_p(32.0);
  info.add_p(0.0);
  info.add_p(0.0);
  info.add_p(320.0);
  info.add_p(32.0);
  info.add_p(0.0);
  info.add_p(0.0);
  info.add_p(1.0);
  return info;
}

sensor_msgs::NavSatFix MakeNavSatFix(uint64_t stamp_ns, double phase) {
  sensor_msgs::NavSatFix fix;
  FillHeader(fix.mutable_header(), "gps", stamp_ns);
  fix.mutable_status()->set_status(0);
  fix.mutable_status()->set_service(1);
  fix.set_latitude(48.137 + 0.0001 * std::sin(phase));
  fix.set_longitude(11.575 + 0.0001 * std::cos(phase));
  fix.set_altitude(520.0);
  fix.set_position_covariance_type(1);
  return fix;
}

sensor_msgs::Imu MakeImu(uint64_t stamp_ns, double phase) {
  sensor_msgs::Imu imu;
  FillHeader(imu.mutable_header(), "base_link", stamp_ns);
  imu.mutable_orientation()->set_w(1.0);
  imu.mutable_angular_velocity()->set_z(0.2 * std::sin(phase));
  imu.mutable_linear_acceleration()->set_x(0.1 * std::cos(phase));
  imu.mutable_linear_acceleration()->set_y(0.1 * std::sin(phase));
  imu.mutable_linear_acceleration()->set_z(9.81);
  return imu;
}

sensor_msgs::BatteryState MakeBatteryState(uint64_t stamp_ns, double phase) {
  sensor_msgs::BatteryState battery;
  FillHeader(battery.mutable_header(), "base_link", stamp_ns);
  battery.set_voltage(24.0f);
  battery.set_current(-1.2f);
  battery.set_percentage(0.6f + 0.05f * static_cast<float>(std::sin(phase)));
  battery.set_power_supply_status(
      sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING);
  battery.set_power_supply_health(
      sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD);
  battery.set_present(true);
  return battery;
}

map_msgs::OccupancyGrid MakeOccupancyGrid(uint64_t stamp_ns) {
  map_msgs::OccupancyGrid grid;
  FillHeader(grid.mutable_header(), "map", stamp_ns);

  constexpr float kResolution = 0.05f;
  constexpr uint32_t kWidth = 80;
  constexpr uint32_t kHeight = 80;
  grid.mutable_info()->set_resolution(kResolution);
  grid.mutable_info()->set_width(kWidth);
  grid.mutable_info()->set_height(kHeight);
  FillTime(grid.mutable_info()->mutable_map_load_time(), stamp_ns);

  auto* origin = grid.mutable_info()->mutable_origin();
  origin->mutable_position()->set_x(-kWidth * kResolution * 0.5f);
  origin->mutable_position()->set_y(-kHeight * kResolution * 0.5f);
  origin->mutable_position()->set_z(0.0);
  origin->mutable_orientation()->set_w(1.0);

  for (uint32_t y = 0; y < kHeight; ++y) {
    for (uint32_t x = 0; x < kWidth; ++x) {
      const float wx =
          origin->position().x() + (static_cast<float>(x) + 0.5f) * kResolution;
      const float wy =
          origin->position().y() + (static_cast<float>(y) + 0.5f) * kResolution;
      const float dist = std::hypot(wx, wy);

      int32_t value = map_utils::OCC_GRID_UNKNOWN;
      if (dist > 2.8f) {
        value = map_utils::OCC_GRID_UNKNOWN;
      } else if (std::abs(wx) > 2.4f || std::abs(wy) > 2.4f) {
        value = map_utils::OCC_GRID_OCCUPIED;
      } else if (std::abs(wx - 1.0f) < 0.12f && wy > -0.5f) {
        value = map_utils::OCC_GRID_OCCUPIED;
      } else if (std::abs(std::abs(wx) - 2.4f) < 0.12f ||
                 std::abs(std::abs(wy) - 2.4f) < 0.12f) {
        value = 70;
      } else {
        value = map_utils::OCC_GRID_FREE;
      }
      grid.add_data(value);
    }
  }
  return grid;
}

map_msgs::OccupancyGridUpdate MakeOccupancyGridUpdate(uint64_t stamp_ns) {
  map_msgs::OccupancyGridUpdate update;
  FillHeader(update.mutable_header(), "map", stamp_ns);
  update.set_x(0);
  update.set_y(0);
  update.set_width(10);
  update.set_height(10);
  for (uint32_t i = 0; i < 100; ++i) {
    update.add_data(static_cast<int32_t>((i % 3) * 50));
  }
  return update;
}

map_msgs::Costmap MakeCostmap(uint64_t stamp_ns) {
  map_msgs::Costmap costmap;
  FillHeader(costmap.mutable_header(), "map", stamp_ns);
  constexpr float kResolution = 0.05f;
  constexpr uint32_t kSize = 80;
  costmap.mutable_metadata()->set_resolution(kResolution);
  costmap.mutable_metadata()->set_size_x(kSize);
  costmap.mutable_metadata()->set_size_y(kSize);
  FillTime(costmap.mutable_metadata()->mutable_map_load_time(), stamp_ns);
  auto* origin = costmap.mutable_metadata()->mutable_origin();
  origin->mutable_position()->set_x(-kSize * kResolution * 0.5f);
  origin->mutable_position()->set_y(-kSize * kResolution * 0.5f);
  origin->mutable_position()->set_z(0.0);
  origin->mutable_orientation()->set_w(1.0);

  std::string bytes(kSize * kSize, static_cast<char>(costmap::NO_INFORMATION));
  for (uint32_t y = 0; y < kSize; ++y) {
    for (uint32_t x = 0; x < kSize; ++x) {
      const float wx =
          origin->position().x() + (static_cast<float>(x) + 0.5f) * kResolution;
      const float wy =
          origin->position().y() + (static_cast<float>(y) + 0.5f) * kResolution;
      const float dist = std::hypot(wx, wy);
      const size_t idx = static_cast<size_t>(y) * kSize + x;

      if (dist > 2.8f) {
        bytes[idx] = static_cast<char>(costmap::NO_INFORMATION);
      } else if (std::abs(wx) > 2.4f || std::abs(wy) > 2.4f ||
                 (std::abs(wx - 1.0f) < 0.12f && wy > -0.5f)) {
        bytes[idx] = static_cast<char>(costmap::LETHAL_OBSTACLE);
      } else {
        const float wall_dist =
            std::min({std::abs(std::abs(wx) - 2.4f), std::abs(std::abs(wy) - 2.4f),
                      std::max(0.0f, std::abs(wx - 1.0f) - 0.12f)});
        if (wall_dist < 0.35f) {
          const uint8_t cost = static_cast<uint8_t>(std::clamp(
              static_cast<double>(wall_dist / 0.35f * 200.0f + 20.0f), 1.0,
              252.0));
          bytes[idx] = static_cast<char>(cost);
        } else {
          bytes[idx] = static_cast<char>(costmap::FREE_SPACE);
        }
      }
    }
  }
  costmap.set_data(bytes);
  return costmap;
}

map_msgs::CostmapUpdate MakeCostmapUpdate(uint64_t stamp_ns) {
  map_msgs::CostmapUpdate update;
  FillHeader(update.mutable_header(), "map", stamp_ns);
  update.set_x(0);
  update.set_y(0);
  update.set_size_x(10);
  update.set_size_y(10);
  update.set_data(std::string(100, static_cast<char>(128)));
  return update;
}

map_msgs::GridMap MakeGridMap(uint64_t stamp_ns, double /*phase*/) {
  map_msgs::GridMap grid;
  FillHeader(grid.mutable_info()->mutable_header(), "map", stamp_ns);
  grid.mutable_info()->set_resolution(0.2f);
  grid.mutable_info()->set_length_x(4.0f);
  grid.mutable_info()->set_length_y(4.0f);
  grid.mutable_info()->mutable_pose()->mutable_orientation()->set_w(1.0);
  grid.add_layers("elevation");
  grid.add_basic_layers("elevation");

  auto* layer = grid.add_data();
  auto* dim0 = layer->mutable_layout()->add_dim();
  dim0->set_label("column_index");
  dim0->set_size(20);
  dim0->set_stride(20);
  auto* dim1 = layer->mutable_layout()->add_dim();
  dim1->set_label("row_index");
  dim1->set_size(20);
  dim1->set_stride(1);
  for (int y = 0; y < 20; ++y) {
    for (int x = 0; x < 20; ++x) {
      const float dx = static_cast<float>(x - 10) * 0.2f;
      const float dy = static_cast<float>(y - 10) * 0.2f;
      layer->add_data(0.5f * std::exp(-(dx * dx + dy * dy)));
    }
  }
  return grid;
}

map_msgs::GridCells MakeGridCells(uint64_t stamp_ns, const MotionState& /*motion*/) {
  map_msgs::GridCells cells;
  FillHeader(cells.mutable_header(), "map", stamp_ns);
  cells.set_cell_width(0.2f);
  cells.set_cell_height(0.2f);
  for (int i = 0; i < 12; ++i) {
    auto* cell = cells.add_cells();
    cell->set_x(-1.0 + 0.2 * i);
    cell->set_y(-1.5);
    cell->set_z(0.0);
  }
  return cells;
}

map_msgs::VoxelGrid MakeVoxelGrid(uint64_t stamp_ns) {
  map_msgs::VoxelGrid grid;
  FillHeader(grid.mutable_header(), "map", stamp_ns);
  grid.set_size_x(8);
  grid.set_size_y(8);
  grid.set_size_z(4);
  grid.mutable_resolutions()->set_x(0.1f);
  grid.mutable_resolutions()->set_y(0.1f);
  grid.mutable_resolutions()->set_z(0.1f);
  for (uint32_t i = 0; i < grid.size_x() * grid.size_y() * grid.size_z(); ++i) {
    grid.add_data((i % 3 == 0) ? 1U : 0U);
  }
  return grid;
}

vision_msgs::BoundingBox2D MakeBoundingBox2D(double phase) {
  vision_msgs::BoundingBox2D box;
  box.mutable_center()->mutable_position()->set_x(32.0 + 4.0 * std::sin(phase));
  box.mutable_center()->mutable_position()->set_y(32.0);
  box.mutable_center()->set_theta(0.1 * std::sin(phase));
  box.set_size_x(24.0);
  box.set_size_y(16.0);
  return box;
}

vision_msgs::BoundingBox2DArray MakeBoundingBox2DArray(uint64_t stamp_ns,
                                                       double phase) {
  vision_msgs::BoundingBox2DArray array;
  FillHeader(array.mutable_header(), "camera", stamp_ns);
  *array.add_boxes() = MakeBoundingBox2D(phase);
  *array.add_boxes() = MakeBoundingBox2D(phase + 1.0);
  return array;
}

vision_msgs::Detection2D MakeDetection2D(uint64_t stamp_ns, double phase) {
  vision_msgs::Detection2D detection;
  FillHeader(detection.mutable_header(), "camera", stamp_ns);
  auto* result = detection.add_results();
  result->mutable_hypothesis()->set_class_id("person");
  result->mutable_hypothesis()->set_score(0.9);
  *detection.mutable_bbox() = MakeBoundingBox2D(phase);
  detection.set_id("det-2d-0");
  return detection;
}

vision_msgs::Detection2DArray MakeDetection2DArray(uint64_t stamp_ns,
                                                   double phase) {
  vision_msgs::Detection2DArray array;
  FillHeader(array.mutable_header(), "camera", stamp_ns);
  *array.add_detections() = MakeDetection2D(stamp_ns, phase);
  return array;
}

vision_msgs::BoundingBox3D MakeBoundingBox3D(const MotionState& /*motion*/) {
  vision_msgs::BoundingBox3D box;
  box.mutable_center()->mutable_position()->set_x(1.5);
  box.mutable_center()->mutable_position()->set_y(0.0);
  box.mutable_center()->mutable_position()->set_z(0.6);
  box.mutable_center()->mutable_orientation()->set_w(1.0);
  box.mutable_size()->set_x(0.8);
  box.mutable_size()->set_y(0.5);
  box.mutable_size()->set_z(1.2);
  return box;
}

vision_msgs::BoundingBox3DArray MakeBoundingBox3DArray(
    uint64_t stamp_ns, const MotionState& motion) {
  vision_msgs::BoundingBox3DArray array;
  FillHeader(array.mutable_header(), "map", stamp_ns);
  *array.add_boxes() = MakeBoundingBox3D(motion);
  return array;
}

vision_msgs::Detection3D MakeDetection3D(uint64_t stamp_ns,
                                          const MotionState& motion) {
  vision_msgs::Detection3D detection;
  FillHeader(detection.mutable_header(), "map", stamp_ns);
  auto* result = detection.add_results();
  result->mutable_hypothesis()->set_class_id("box");
  result->mutable_hypothesis()->set_score(0.85);
  *detection.mutable_bbox() = MakeBoundingBox3D(motion);
  detection.set_id("det-3d-0");
  return detection;
}

vision_msgs::Detection3DArray MakeDetection3DArray(uint64_t stamp_ns,
                                                   const MotionState& motion) {
  vision_msgs::Detection3DArray array;
  FillHeader(array.mutable_header(), "map", stamp_ns);
  *array.add_detections() = MakeDetection3D(stamp_ns, motion);
  return array;
}

diagnostic_msgs::DiagnosticArray MakeDiagnosticArray(uint64_t stamp_ns,
                                                     uint64_t seq) {
  diagnostic_msgs::DiagnosticArray array;
  FillHeader(array.mutable_header(), "base_link", stamp_ns);
  auto* status = array.add_status();
  status->set_level(diagnostic_msgs::DiagnosticStatus::OK);
  status->set_name("fake_node");
  status->set_message("running");
  status->set_hardware_id("fake-001");
  auto* value = status->add_values();
  value->set_key("frame");
  value->set_value(std::to_string(seq));
  return array;
}

vehicle_msgs::RobotState MakeRobotState(uint64_t stamp_ns,
                                        const MotionState& motion) {
  vehicle_msgs::RobotState state;
  FillTime(state.mutable_timestamp(), stamp_ns);
  *state.mutable_pose() = MakeStaticPoseStamped(stamp_ns);
  FillHeader(state.mutable_twist()->mutable_header(), "base_link", stamp_ns);
  state.mutable_twist()->mutable_twist()->mutable_linear()->set_x(0.2);
  state.mutable_twist()->mutable_twist()->mutable_angular()->set_z(0.1);
  state.set_battery_percent(72.0f);
  state.set_map_name("fake_map");
  state.set_global_frame("map");
  state.set_active_task_type(vehicle_msgs::ROBOT_TASK_NAVIGATION);
  state.set_active_task_status(vehicle_msgs::ROBOT_TASK_STATUS_RUNNING);
  state.set_motion_enabled(true);
  state.set_localization_quality(0.95f);
  return state;
}

template <typename MessageT>
using WriterPtr = std::shared_ptr<autolink::Writer<MessageT>>;

struct FakePublishers {
  WriterPtr<tf2_msgs::TFMessage> tf;
  WriterPtr<visualization_msgs::Marker> marker;
  WriterPtr<visualization_msgs::MarkerArray> marker_array;
  WriterPtr<geometry_msgs::PoseStamped> pose_stamped;
  WriterPtr<geometry_msgs::PoseArray> pose_array;
  WriterPtr<geometry_msgs::PointStamped> point_stamped;
  WriterPtr<geometry_msgs::PolygonStamped> polygon_stamped;
  WriterPtr<geometry_msgs::TransformStamped> transform_stamped;
  WriterPtr<planning_msgs::Path> path;
  WriterPtr<planning_msgs::Odometry> odom;
  WriterPtr<sensor_msgs::LaserScan> scan;
  WriterPtr<sensor_msgs::MultiEchoLaserScan> multi_echo_scan;
  WriterPtr<sensor_msgs::PointCloud2> point_cloud2;
  WriterPtr<sensor_msgs::PointCloud> point_cloud;
  WriterPtr<sensor_msgs::Range> range;
  WriterPtr<sensor_msgs::Image> image;
  WriterPtr<sensor_msgs::CompressedImage> compressed_image;
  WriterPtr<sensor_msgs::CameraInfo> camera_info;
  WriterPtr<sensor_msgs::NavSatFix> nav_sat_fix;
  WriterPtr<sensor_msgs::Imu> imu;
  WriterPtr<sensor_msgs::BatteryState> battery_state;
  WriterPtr<map_msgs::OccupancyGrid> occupancy_grid;
  WriterPtr<map_msgs::OccupancyGridUpdate> occupancy_grid_update;
  WriterPtr<map_msgs::Costmap> costmap;
  WriterPtr<map_msgs::CostmapUpdate> costmap_update;
  WriterPtr<map_msgs::GridMap> grid_map;
  WriterPtr<map_msgs::GridCells> grid_cells;
  WriterPtr<map_msgs::VoxelGrid> voxel_grid;
  WriterPtr<vision_msgs::BoundingBox2D> bbox2d;
  WriterPtr<vision_msgs::BoundingBox2DArray> bbox2d_array;
  WriterPtr<vision_msgs::Detection2D> detection2d;
  WriterPtr<vision_msgs::Detection2DArray> detection2d_array;
  WriterPtr<vision_msgs::BoundingBox3D> bbox3d;
  WriterPtr<vision_msgs::BoundingBox3DArray> bbox3d_array;
  WriterPtr<vision_msgs::Detection3D> detection3d;
  WriterPtr<vision_msgs::Detection3DArray> detection3d_array;
  WriterPtr<diagnostic_msgs::DiagnosticArray> diagnostic_array;
  WriterPtr<vehicle_msgs::RobotState> robot_state;
};

#define CREATE_WRITER(field, suffix, type) \
  publishers.field = node->CreateWriter<type>(Channel(prefix, suffix))

FakePublishers CreatePublishers(const std::shared_ptr<autolink::Node>& node,
                                const std::string& prefix,
                                const std::string& tf_channel) {
  FakePublishers publishers;
  publishers.tf = node->CreateWriter<tf2_msgs::TFMessage>(tf_channel);
  CREATE_WRITER(marker, "marker", visualization_msgs::Marker);
  CREATE_WRITER(marker_array, "marker_array", visualization_msgs::MarkerArray);
  CREATE_WRITER(pose_stamped, "pose_stamped", geometry_msgs::PoseStamped);
  CREATE_WRITER(pose_array, "pose_array", geometry_msgs::PoseArray);
  CREATE_WRITER(point_stamped, "point_stamped", geometry_msgs::PointStamped);
  CREATE_WRITER(polygon_stamped, "polygon_stamped",
                geometry_msgs::PolygonStamped);
  CREATE_WRITER(transform_stamped, "transform_stamped",
                geometry_msgs::TransformStamped);
  CREATE_WRITER(path, "path", planning_msgs::Path);
  CREATE_WRITER(odom, "odom", planning_msgs::Odometry);
  CREATE_WRITER(scan, "scan", sensor_msgs::LaserScan);
  CREATE_WRITER(multi_echo_scan, "multi_echo_scan",
                sensor_msgs::MultiEchoLaserScan);
  CREATE_WRITER(point_cloud2, "point_cloud2", sensor_msgs::PointCloud2);
  CREATE_WRITER(point_cloud, "point_cloud", sensor_msgs::PointCloud);
  CREATE_WRITER(range, "range", sensor_msgs::Range);
  CREATE_WRITER(image, "image", sensor_msgs::Image);
  CREATE_WRITER(compressed_image, "compressed_image",
                sensor_msgs::CompressedImage);
  CREATE_WRITER(camera_info, "camera_info", sensor_msgs::CameraInfo);
  CREATE_WRITER(nav_sat_fix, "nav_sat_fix", sensor_msgs::NavSatFix);
  CREATE_WRITER(imu, "imu", sensor_msgs::Imu);
  CREATE_WRITER(battery_state, "battery_state", sensor_msgs::BatteryState);
  CREATE_WRITER(occupancy_grid, "occupancy_grid", map_msgs::OccupancyGrid);
  CREATE_WRITER(occupancy_grid_update, "occupancy_grid_update",
                map_msgs::OccupancyGridUpdate);
  CREATE_WRITER(costmap, "costmap", map_msgs::Costmap);
  CREATE_WRITER(costmap_update, "costmap_update", map_msgs::CostmapUpdate);
  CREATE_WRITER(grid_map, "grid_map", map_msgs::GridMap);
  CREATE_WRITER(grid_cells, "grid_cells", map_msgs::GridCells);
  CREATE_WRITER(voxel_grid, "voxel_grid", map_msgs::VoxelGrid);
  CREATE_WRITER(bbox2d, "bbox2d", vision_msgs::BoundingBox2D);
  CREATE_WRITER(bbox2d_array, "bbox2d_array", vision_msgs::BoundingBox2DArray);
  CREATE_WRITER(detection2d, "detection2d", vision_msgs::Detection2D);
  CREATE_WRITER(detection2d_array, "detection2d_array",
                vision_msgs::Detection2DArray);
  CREATE_WRITER(bbox3d, "bbox3d", vision_msgs::BoundingBox3D);
  CREATE_WRITER(bbox3d_array, "bbox3d_array", vision_msgs::BoundingBox3DArray);
  CREATE_WRITER(detection3d, "detection3d", vision_msgs::Detection3D);
  CREATE_WRITER(detection3d_array, "detection3d_array",
                vision_msgs::Detection3DArray);
  CREATE_WRITER(diagnostic_array, "diagnostic_array",
                diagnostic_msgs::DiagnosticArray);
  CREATE_WRITER(robot_state, "robot_state", vehicle_msgs::RobotState);
  return publishers;
}

#undef CREATE_WRITER

template <typename MessageT>
void Publish(const WriterPtr<MessageT>& writer,
             const MessageT& message) {
  writer->Write(std::make_shared<MessageT>(message));
}

const sensor_msgs::PointCloud2& StaticPointCloud2Template() {
  static const sensor_msgs::PointCloud2 kTemplate = MakePointCloud2(0);
  return kTemplate;
}

void PublishPointCloud2Synced(const WriterPtr<sensor_msgs::PointCloud2>& writer,
                              uint64_t stamp_ns) {
  sensor_msgs::PointCloud2 cloud = StaticPointCloud2Template();
  FillHeader(cloud.mutable_header(), "map", stamp_ns);
  Publish(writer, cloud);
}

bool ValidatePublishers(const FakePublishers& publishers) {
  const bool ok =
      publishers.tf && publishers.marker && publishers.marker_array &&
      publishers.pose_stamped && publishers.pose_array &&
      publishers.point_stamped && publishers.polygon_stamped &&
      publishers.transform_stamped && publishers.path && publishers.odom &&
      publishers.scan && publishers.multi_echo_scan && publishers.point_cloud2 &&
      publishers.point_cloud && publishers.range && publishers.image &&
      publishers.compressed_image && publishers.camera_info &&
      publishers.nav_sat_fix && publishers.imu && publishers.battery_state &&
      publishers.occupancy_grid && publishers.occupancy_grid_update &&
      publishers.costmap && publishers.costmap_update && publishers.grid_map &&
      publishers.grid_cells && publishers.voxel_grid && publishers.bbox2d &&
      publishers.bbox2d_array && publishers.detection2d &&
      publishers.detection2d_array && publishers.bbox3d &&
      publishers.bbox3d_array && publishers.detection3d &&
      publishers.detection3d_array && publishers.diagnostic_array &&
      publishers.robot_state;
  return ok;
}

void PublishStaticAssets(const FakePublishers& publishers,
                         uint64_t static_stamp_ns) {
  Publish(publishers.occupancy_grid, MakeOccupancyGrid(static_stamp_ns));
  Publish(publishers.occupancy_grid_update,
          MakeOccupancyGridUpdate(static_stamp_ns));
  Publish(publishers.costmap, MakeCostmap(static_stamp_ns));
  Publish(publishers.grid_map, MakeGridMap(static_stamp_ns, 0.0));
  Publish(publishers.costmap_update, MakeCostmapUpdate(static_stamp_ns));

  Publish(publishers.tf, MakeTfTree(static_stamp_ns));
  Publish(publishers.transform_stamped, MakeSingleTransform(static_stamp_ns, {}));
  Publish(publishers.marker, MakeRobotMarker(static_stamp_ns, 0.0, 0.0));
  Publish(publishers.marker_array, MakeMarkerArray(static_stamp_ns, {}));
  Publish(publishers.pose_stamped, MakeStaticPoseStamped(static_stamp_ns));
  Publish(publishers.point_stamped, MakeStaticPointStamped(static_stamp_ns));
  Publish(publishers.polygon_stamped, MakePolygonStamped(static_stamp_ns));
  Publish(publishers.path, MakePath(static_stamp_ns));
  Publish(publishers.voxel_grid, MakeVoxelGrid(static_stamp_ns));
  Publish(publishers.grid_cells, MakeGridCells(static_stamp_ns, {}));
  Publish(publishers.range, MakeRange(static_stamp_ns, 0.0));
  Publish(publishers.bbox3d, MakeBoundingBox3D({}));
  Publish(publishers.bbox3d_array, MakeBoundingBox3DArray(static_stamp_ns, {}));
  Publish(publishers.detection3d, MakeDetection3D(static_stamp_ns, {}));
  Publish(publishers.detection3d_array,
          MakeDetection3DArray(static_stamp_ns, {}));
  Publish(publishers.camera_info, MakeCameraInfo(static_stamp_ns));
}

void PublishDynamicFrame(const FakePublishers& publishers, uint64_t stamp_ns,
                         uint64_t seq, const MotionState& motion) {
  const uint64_t sync_interval = std::max<uint64_t>(
      1, static_cast<uint64_t>(FLAGS_rate_hz / FLAGS_point_cloud2_sync_hz));
  if (seq % sync_interval == 0) {
    PublishPointCloud2Synced(publishers.point_cloud2, stamp_ns);
  }

  Publish(publishers.pose_array, MakePoseArray(stamp_ns, motion.phase));
  Publish(publishers.odom, MakeOdometry(stamp_ns, motion));
  Publish(publishers.scan, MakeLaserScan(stamp_ns, motion.phase));
  Publish(publishers.multi_echo_scan,
          MakeMultiEchoLaserScan(stamp_ns, motion.phase));
  Publish(publishers.point_cloud, MakePointCloud(stamp_ns, motion.phase));
  Publish(publishers.image, MakeImage(stamp_ns, seq));
  Publish(publishers.compressed_image, MakeCompressedImage(stamp_ns, seq));
  Publish(publishers.nav_sat_fix, MakeNavSatFix(stamp_ns, motion.phase));
  Publish(publishers.imu, MakeImu(stamp_ns, motion.phase));
  Publish(publishers.battery_state, MakeBatteryState(stamp_ns, motion.phase));
  Publish(publishers.bbox2d, MakeBoundingBox2D(motion.phase));
  Publish(publishers.bbox2d_array,
          MakeBoundingBox2DArray(stamp_ns, motion.phase));
  Publish(publishers.detection2d, MakeDetection2D(stamp_ns, motion.phase));
  Publish(publishers.detection2d_array,
          MakeDetection2DArray(stamp_ns, motion.phase));
  Publish(publishers.diagnostic_array, MakeDiagnosticArray(stamp_ns, seq));
  Publish(publishers.robot_state, MakeRobotState(stamp_ns, motion));
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (!autolink::Init(argv[0])) {
    LOG(ERROR) << "Failed to initialize autolink runtime.";
    return 1;
  }

  auto node = autolink::CreateNode("foxglove_fakedata");
  const FakePublishers publishers = CreatePublishers(
      node, FLAGS_channel_prefix, FLAGS_tf_channel);
  if (!ValidatePublishers(publishers)) {
    LOG(ERROR) << "Failed to create one or more writers.";
    return 1;
  }

  const uint64_t static_stamp_ns = autolink::Time::Now().ToNanosecond();
  PublishStaticAssets(publishers, static_stamp_ns);
  PublishPointCloud2Synced(publishers.point_cloud2, static_stamp_ns);
  LOG(INFO) << "Published static TF/maps/scene once; point_cloud2 resyncs at "
            << FLAGS_point_cloud2_sync_hz << " Hz with live timestamp.";

  LOG(INFO) << "Publishing fake data under prefix " << FLAGS_channel_prefix
            << " (TF tree on " << FLAGS_tf_channel
            << ": map->odom->base_link->{laser,camera})";
  LOG(INFO) << "Foxglove 3D: Fixed frame=map; enable " << FLAGS_tf_channel
            << ", " << Channel(FLAGS_channel_prefix, "scan") << ", "
            << Channel(FLAGS_channel_prefix, "occupancy_grid");
  LOG(INFO) << "Connect Foxglove to ws://127.0.0.1:8765 after starting bridge.";

  autolink::Rate rate(FLAGS_rate_hz);
  uint64_t seq = 0;
  while (autolink::OK()) {
    const uint64_t stamp_ns = autolink::Time::Now().ToNanosecond();
    const MotionState motion = ComputeMotion(seq);
    PublishDynamicFrame(publishers, stamp_ns, seq, motion);

    if (seq % static_cast<uint64_t>(FLAGS_rate_hz) == 0) {
      Publish(publishers.tf, MakeTfTree(stamp_ns));
    }

    if (seq % static_cast<uint64_t>(FLAGS_rate_hz) == 0) {
      LOG(INFO) << "Published dynamic frame #" << seq;
    }
    ++seq;
    rate.Sleep();
  }

  autolink::Clear();
  return 0;
}
