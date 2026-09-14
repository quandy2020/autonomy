/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/adapters/automsgs/automsgs_converter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

#include <automsgs/msgs/geometry_msgs/pose2d.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/polygon.pb.h>
#include <automsgs/msgs/geometry_msgs/polygon_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/strata_msgs/floor_info.pb.h>
#include <automsgs/msgs/strata_msgs/semantic_zone.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>
#include <automsgs/msgs/vehicle_msgs/robot_state.pb.h>

#include "autonomy/orbisview/backend/common/render_schemas.hpp"

namespace autonomy {
namespace orbisview {
namespace adapters {
namespace {

namespace geometry_msgs = automsgs::msgs::geometry_msgs;
namespace map_msgs = automsgs::msgs::map_msgs;
namespace nav_msgs = automsgs::msgs::nav_msgs;
namespace sensor_msgs = automsgs::msgs::sensor_msgs;
namespace strata_msgs = automsgs::msgs::strata_msgs;
namespace tf2_msgs = automsgs::msgs::tf2_msgs;
namespace vehicle_msgs = automsgs::msgs::vehicle_msgs;

bool EndsWith(const std::string& s, const std::string& suffix) {
  return s.size() >= suffix.size() &&
         s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool TypeIs(const std::string& msg_type, const char* full_name) {
  if (msg_type == full_name) return true;
  const char* short_name = std::strrchr(full_name, '.');
  short_name = short_name ? short_name + 1 : full_name;
  if (msg_type == short_name) return true;
  const std::string dotted = std::string(".") + short_name;
  return EndsWith(msg_type, dotted);
}

double YawFromQuat(double x, double y, double z, double w) {
  // yaw (Z) from quaternion
  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double YawFromOrientation(const geometry_msgs::Quaternion& q) {
  return YawFromQuat(q.x(), q.y(), q.z(), q.w());
}

void SetJsonPayload(core::StreamEnvelope* out, const std::string& json) {
  out->encoding = "json";
  out->payload.assign(json.begin(), json.end());
  out->unsupported = false;
}

bool ConvertPose2D(const std::string& bytes, core::StreamEnvelope* out) {
  geometry_msgs::Pose2D msg;
  if (!msg.ParseFromString(bytes)) return false;
  std::ostringstream oss;
  oss << "{\"x\":" << msg.x() << ",\"y\":" << msg.y()
      << ",\"yaw\":" << msg.theta() << '}';
  out->schema = rendering::kSchemaPose;
  SetJsonPayload(out, oss.str());
  return true;
}

bool ConvertPose2DStamped(const std::string& bytes, core::StreamEnvelope* out) {
  geometry_msgs::Pose2DStamped msg;
  if (!msg.ParseFromString(bytes)) return false;
  out->frame_id = msg.header().frame_id();
  std::ostringstream oss;
  oss << "{\"x\":" << msg.pose().x() << ",\"y\":" << msg.pose().y()
      << ",\"yaw\":" << msg.pose().theta() << '}';
  out->schema = rendering::kSchemaPose;
  SetJsonPayload(out, oss.str());
  return true;
}

bool ConvertPoseStamped(const std::string& bytes, core::StreamEnvelope* out) {
  geometry_msgs::PoseStamped msg;
  if (!msg.ParseFromString(bytes)) return false;
  out->frame_id = msg.header().frame_id();
  const auto& p = msg.pose().position();
  const double yaw = YawFromOrientation(msg.pose().orientation());
  std::ostringstream oss;
  oss << "{\"x\":" << p.x() << ",\"y\":" << p.y() << ",\"yaw\":" << yaw << '}';
  out->schema = rendering::kSchemaPose;
  SetJsonPayload(out, oss.str());
  return true;
}

bool ConvertOdometry(const std::string& bytes, core::StreamEnvelope* out) {
  nav_msgs::Odometry msg;
  if (!msg.ParseFromString(bytes)) return false;
  out->frame_id = msg.header().frame_id();
  // Odometry.pose = PoseWithCovariance { PoseStamped pose }
  const auto& pose = msg.pose().pose().pose();
  const auto& p = pose.position();
  const double yaw = YawFromOrientation(pose.orientation());
  std::ostringstream oss;
  oss << "{\"x\":" << p.x() << ",\"y\":" << p.y() << ",\"yaw\":" << yaw << '}';
  out->schema = rendering::kSchemaPose;
  SetJsonPayload(out, oss.str());
  return true;
}

bool ConvertPath(const std::string& bytes, core::StreamEnvelope* out) {
  nav_msgs::Path msg;
  if (!msg.ParseFromString(bytes)) return false;
  out->frame_id = msg.header().frame_id();
  std::ostringstream oss;
  oss << "{\"poses\":[";
  for (int i = 0; i < msg.poses_size(); ++i) {
    if (i) oss << ',';
    const auto& ps = msg.poses(i);
    const auto& p = ps.pose().position();
    const double yaw = YawFromOrientation(ps.pose().orientation());
    oss << "{\"x\":" << p.x() << ",\"y\":" << p.y() << ",\"yaw\":" << yaw << '}';
  }
  oss << "]}";
  out->schema = rendering::kSchemaPath;
  SetJsonPayload(out, oss.str());
  return true;
}

bool ConvertLaserScan(const std::string& bytes, core::StreamEnvelope* out) {
  sensor_msgs::LaserScan msg;
  if (!msg.ParseFromString(bytes)) return false;
  out->frame_id = msg.header().frame_id();
  std::ostringstream oss;
  oss << "{\"angle_min\":" << msg.angle_min()
      << ",\"angle_increment\":" << msg.angle_increment()
      << ",\"range_max\":" << msg.range_max() << ",\"ranges\":[";
  constexpr int kMax = 360;
  const int n = std::min(msg.ranges_size(), kMax);
  for (int i = 0; i < n; ++i) {
    if (i) oss << ',';
    oss << msg.ranges(i);
  }
  oss << "]}";
  out->schema = rendering::kSchemaLaserScan;
  SetJsonPayload(out, oss.str());
  return true;
}

bool ConvertOccupancyGrid(const std::string& bytes, core::StreamEnvelope* out) {
  map_msgs::OccupancyGrid msg;
  if (!msg.ParseFromString(bytes)) return false;
  out->frame_id = msg.header().frame_id();
  const auto& info = msg.info();
  const auto& origin = info.origin().position();
  const double yaw = YawFromOrientation(info.origin().orientation());
  std::ostringstream oss;
  oss << "{\"resolution\":" << info.resolution() << ",\"width\":" << info.width()
      << ",\"height\":" << info.height() << ",\"origin\":{\"x\":" << origin.x()
      << ",\"y\":" << origin.y() << ",\"yaw\":" << yaw << "},\"data\":[";
  for (int i = 0; i < msg.data_size(); ++i) {
    if (i) oss << ',';
    oss << msg.data(i);
  }
  oss << "]}";
  out->schema = rendering::kSchemaOccupancyGrid;
  SetJsonPayload(out, oss.str());
  return true;
}

bool ConvertPolygonPoints(const geometry_msgs::Polygon& poly,
                          core::StreamEnvelope* out) {
  if (poly.points_size() < 3) return false;
  std::ostringstream oss;
  oss << "{\"shape\":\"POLYGON\",\"padding\":0,\"points\":[";
  for (int i = 0; i < poly.points_size(); ++i) {
    if (i) oss << ',';
    const auto& p = poly.points(i);
    oss << "{\"x\":" << p.x() << ",\"y\":" << p.y() << ",\"yaw\":0}";
  }
  oss << "]}";
  out->schema = rendering::kSchemaFootprint;
  SetJsonPayload(out, oss.str());
  return true;
}

bool ConvertPolygon(const std::string& bytes, core::StreamEnvelope* out) {
  geometry_msgs::Polygon msg;
  if (!msg.ParseFromString(bytes)) return false;
  return ConvertPolygonPoints(msg, out);
}

bool ConvertPolygonStamped(const std::string& bytes,
                           core::StreamEnvelope* out) {
  geometry_msgs::PolygonStamped msg;
  if (!msg.ParseFromString(bytes)) return false;
  out->frame_id = msg.header().frame_id();
  return ConvertPolygonPoints(msg.polygon(), out);
}

void AppendTfJson(std::ostringstream& oss, bool* first,
                  const geometry_msgs::TransformStamped& tf) {
  if (!*first) oss << ',';
  *first = false;
  const auto& t = tf.transform().translation();
  const double yaw = YawFromOrientation(tf.transform().rotation());
  oss << "{\"parent\":" << core::JsonEscape(tf.header().frame_id())
      << ",\"child\":" << core::JsonEscape(tf.child_frame_id())
      << ",\"x\":" << t.x() << ",\"y\":" << t.y() << ",\"yaw\":" << yaw << '}';
}

bool ConvertTransformStampeds(const std::string& bytes,
                              core::StreamEnvelope* out) {
  geometry_msgs::TransformStampeds msg;
  if (!msg.ParseFromString(bytes)) return false;
  out->frame_id = msg.header().frame_id();
  std::ostringstream oss;
  oss << "{\"transforms\":[";
  bool first = true;
  for (const auto& tf : msg.transforms()) AppendTfJson(oss, &first, tf);
  oss << "]}";
  out->schema = rendering::kSchemaTfTree;
  SetJsonPayload(out, oss.str());
  return true;
}

bool ConvertTfMessage(const std::string& bytes, core::StreamEnvelope* out) {
  tf2_msgs::TFMessage msg;
  if (!msg.ParseFromString(bytes)) return false;
  std::ostringstream oss;
  oss << "{\"transforms\":[";
  bool first = true;
  for (const auto& tf : msg.transforms()) AppendTfJson(oss, &first, tf);
  oss << "]}";
  out->schema = rendering::kSchemaTfTree;
  SetJsonPayload(out, oss.str());
  return true;
}

int FindFieldOffset(const sensor_msgs::PointCloud2& cloud, const char* name) {
  for (const auto& f : cloud.fields()) {
    if (f.name() == name) return static_cast<int>(f.offset());
  }
  return -1;
}

float ReadFloatLE(const std::string& data, size_t off) {
  if (off + 4 > data.size()) return 0.f;
  float v = 0.f;
  std::memcpy(&v, data.data() + off, sizeof(float));
  return v;
}

bool ConvertPointCloud2(const std::string& bytes, core::StreamEnvelope* out) {
  sensor_msgs::PointCloud2 msg;
  if (!msg.ParseFromString(bytes)) return false;
  out->frame_id = msg.header().frame_id();
  const int ox = FindFieldOffset(msg, "x");
  const int oy = FindFieldOffset(msg, "y");
  const int oz = FindFieldOffset(msg, "z");
  if (ox < 0 || oy < 0 || oz < 0) return false;
  const uint32_t point_step = msg.point_step();
  const uint32_t n_points = msg.height() * msg.width();
  if (point_step == 0 || n_points == 0) return false;
  constexpr uint32_t kMaxPoints = 500;
  const uint32_t stride =
      n_points > kMaxPoints ? (n_points + kMaxPoints - 1) / kMaxPoints : 1;
  std::ostringstream oss;
  oss << "{\"points\":[";
  bool first = true;
  const std::string& data = msg.data();
  for (uint32_t i = 0; i < n_points; i += stride) {
    const size_t base = static_cast<size_t>(i) * point_step;
    if (base + point_step > data.size()) break;
    if (!first) oss << ',';
    first = false;
    const float x = ReadFloatLE(data, base + static_cast<size_t>(ox));
    const float y = ReadFloatLE(data, base + static_cast<size_t>(oy));
    const float z = ReadFloatLE(data, base + static_cast<size_t>(oz));
    oss << "{\"x\":" << x << ",\"y\":" << y << ",\"z\":" << z << '}';
  }
  oss << "]}";
  out->schema = rendering::kSchemaPointCloud2;
  SetJsonPayload(out, oss.str());
  return true;
}

bool ConvertImageLike(const std::string& bytes, core::StreamEnvelope* out,
                      bool prefer_depth) {
  sensor_msgs::Image msg;
  if (!msg.ParseFromString(bytes)) return false;
  out->frame_id = msg.header().frame_id();
  const std::string enc = msg.encoding();
  const bool is_depth =
      prefer_depth || enc.find("16UC") != std::string::npos ||
      enc.find("32FC") != std::string::npos || enc == "mono16" ||
      enc.find("depth") != std::string::npos;

  constexpr uint32_t kMaxW = 64;
  constexpr uint32_t kMaxH = 48;
  const uint32_t w = msg.width();
  const uint32_t h = msg.height();
  if (w == 0 || h == 0) return false;
  const uint32_t step_x = std::max(1u, (w + kMaxW - 1) / kMaxW);
  const uint32_t step_y = std::max(1u, (h + kMaxH - 1) / kMaxH);
  const uint32_t out_w = (w + step_x - 1) / step_x;
  const uint32_t out_h = (h + step_y - 1) / step_y;

  std::ostringstream oss;
  oss << "{\"width\":" << out_w << ",\"height\":" << out_h
      << ",\"encoding\":\"mono8\",\"data\":[";
  bool first = true;
  const std::string& data = msg.data();
  const uint32_t row_step = msg.step() ? msg.step() : w;
  for (uint32_t y = 0; y < h; y += step_y) {
    for (uint32_t x = 0; x < w; x += step_x) {
      if (!first) oss << ',';
      first = false;
      int v = 0;
      if (enc == "mono8" || enc == "8UC1") {
        const size_t idx = static_cast<size_t>(y) * row_step + x;
        if (idx < data.size()) v = static_cast<uint8_t>(data[idx]);
      } else if (enc == "rgb8" || enc == "bgr8") {
        const size_t idx = static_cast<size_t>(y) * row_step + x * 3;
        if (idx + 2 < data.size()) {
          v = (static_cast<uint8_t>(data[idx]) +
               static_cast<uint8_t>(data[idx + 1]) +
               static_cast<uint8_t>(data[idx + 2])) /
              3;
        }
      } else if (enc == "16UC1" || enc == "mono16") {
        const size_t idx = static_cast<size_t>(y) * row_step + x * 2;
        if (idx + 1 < data.size()) {
          const uint16_t d = static_cast<uint8_t>(data[idx]) |
                             (static_cast<uint16_t>(
                                  static_cast<uint8_t>(data[idx + 1]))
                              << 8);
          v = static_cast<int>(std::min<uint16_t>(d / 32, 255));
        }
      } else {
        const size_t idx = static_cast<size_t>(y) * row_step + x;
        if (idx < data.size()) v = static_cast<uint8_t>(data[idx]);
      }
      oss << v;
    }
  }
  oss << "]}";
  out->schema =
      is_depth ? rendering::kSchemaDepthImage : rendering::kSchemaImage;
  SetJsonPayload(out, oss.str());
  return true;
}

bool ConvertTwist(const std::string& bytes, core::StreamEnvelope* out) {
  geometry_msgs::Twist msg;
  if (!msg.ParseFromString(bytes)) return false;
  std::ostringstream oss;
  oss << "{\"vx\":" << msg.linear().x() << ",\"wz\":" << msg.angular().z()
      << '}';
  out->schema = rendering::kSchemaTwist2D;
  out->frame_id = "base_link";
  SetJsonPayload(out, oss.str());
  return true;
}

bool ConvertTwist2DMsg(const std::string& bytes, core::StreamEnvelope* out) {
  geometry_msgs::Twist2D msg;
  if (!msg.ParseFromString(bytes)) return false;
  std::ostringstream oss;
  oss << "{\"vx\":" << msg.x() << ",\"wz\":" << msg.theta() << '}';
  out->schema = rendering::kSchemaTwist2D;
  out->frame_id = "base_link";
  SetJsonPayload(out, oss.str());
  return true;
}

bool ConvertTwistStamped(const std::string& bytes, core::StreamEnvelope* out) {
  geometry_msgs::TwistStamped msg;
  if (!msg.ParseFromString(bytes)) return false;
  std::ostringstream oss;
  oss << "{\"vx\":" << msg.twist().linear().x()
      << ",\"wz\":" << msg.twist().angular().z() << '}';
  out->schema = rendering::kSchemaTwist2D;
  out->frame_id = "base_link";
  SetJsonPayload(out, oss.str());
  return true;
}

bool ConvertRobotState(const std::string& bytes, core::StreamEnvelope* out) {
  vehicle_msgs::RobotState msg;
  if (!msg.ParseFromString(bytes)) return false;
  const double vx = msg.twist().twist().linear().x();
  const double wz = msg.twist().twist().angular().z();
  std::ostringstream oss;
  oss << "{\"vx\":" << vx << ",\"wz\":" << wz << ",\"gear\":\"D\",\"throttle\":"
      << std::min(1.0, std::abs(vx)) << ",\"brake\":0,\"steering\":"
      << std::max(-1.0, std::min(1.0, wz)) << ",\"driving_mode\":\""
      << (msg.motion_enabled() ? "AUTO" : "MANUAL") << "\"}";
  out->schema = rendering::kSchemaChassis;
  out->frame_id =
      msg.global_frame().empty() ? "map" : msg.global_frame();
  SetJsonPayload(out, oss.str());
  return true;
}

void AppendColorJson(std::ostringstream& oss, const automsgs::msgs::std_msgs::ColorRGBA& c) {
  oss << "{\"r\":" << c.r() << ",\"g\":" << c.g() << ",\"b\":" << c.b()
      << ",\"a\":" << c.a() << '}';
}

bool ConvertSemanticZoneArray(const std::string& bytes, core::StreamEnvelope* out) {
  strata_msgs::SemanticZoneArray msg;
  if (!msg.ParseFromString(bytes)) return false;
  out->frame_id = msg.header().frame_id().empty() ? "map" : msg.header().frame_id();
  std::ostringstream oss;
  oss << "{\"zones\":[";
  for (int i = 0; i < msg.zones_size(); ++i) {
    if (i) oss << ',';
    const auto& z = msg.zones(i);
    oss << "{\"id\":" << core::JsonEscape(z.id())
        << ",\"zone_type\":" << core::JsonEscape(z.zone_type())
        << ",\"label\":" << core::JsonEscape(z.label())
        << ",\"fill_opacity\":" << z.fill_opacity()
        << ",\"outline_width\":" << z.outline_width()
        << ",\"fill_color\":";
    AppendColorJson(oss, z.fill_color());
    oss << ",\"outline_color\":";
    AppendColorJson(oss, z.outline_color());
    oss << ",\"polygon\":[";
    for (int j = 0; j < z.polygon_size(); ++j) {
      if (j) oss << ',';
      const auto& p = z.polygon(j);
      oss << "{\"x\":" << p.x() << ",\"y\":" << p.y() << '}';
    }
    oss << "]}";
  }
  oss << "]}";
  out->schema = rendering::kSchemaSemanticZones;
  SetJsonPayload(out, oss.str());
  return true;
}

bool ConvertFloorInfoArray(const std::string& bytes, core::StreamEnvelope* out) {
  strata_msgs::FloorInfoArray msg;
  if (!msg.ParseFromString(bytes)) return false;
  out->frame_id = msg.header().frame_id().empty() ? "map" : msg.header().frame_id();
  std::ostringstream oss;
  oss << "{\"active_floor_id\":" << core::JsonEscape(msg.active_floor_id())
      << ",\"floors\":[";
  for (int i = 0; i < msg.floors_size(); ++i) {
    if (i) oss << ',';
    const auto& f = msg.floors(i);
    oss << "{\"id\":" << core::JsonEscape(f.id())
        << ",\"name\":" << core::JsonEscape(f.name())
        << ",\"level\":" << f.level()
        << ",\"slam_image_path\":" << core::JsonEscape(f.slam_image_path())
        << ",\"start_x\":" << f.start_x() << ",\"start_y\":" << f.start_y()
        << ",\"x_grid_count\":" << f.x_grid_count()
        << ",\"y_grid_count\":" << f.y_grid_count()
        << ",\"resolution\":" << f.resolution() << '}';
  }
  oss << "]}";
  out->schema = rendering::kSchemaFloors;
  SetJsonPayload(out, oss.str());
  return true;
}

}  // namespace
  if (TypeIs(msg_type, "automsgs.msgs.geometry_msgs.Pose2D") ||
      TypeIs(msg_type, "automsgs.msgs.geometry_msgs.Pose2DStamped") ||
      TypeIs(msg_type, "automsgs.msgs.geometry_msgs.PoseStamped") ||
      TypeIs(msg_type, "automsgs.msgs.nav_msgs.Odometry")) {
    return rendering::kSchemaPose;
  }
  if (TypeIs(msg_type, "automsgs.msgs.nav_msgs.Path")) {
    return rendering::kSchemaPath;
  }
  if (TypeIs(msg_type, "automsgs.msgs.sensor_msgs.LaserScan")) {
    return rendering::kSchemaLaserScan;
  }
  if (TypeIs(msg_type, "automsgs.msgs.map_msgs.OccupancyGrid")) {
    return rendering::kSchemaOccupancyGrid;
  }
  if (TypeIs(msg_type, "automsgs.msgs.geometry_msgs.Polygon") ||
      TypeIs(msg_type, "automsgs.msgs.geometry_msgs.PolygonStamped")) {
    return rendering::kSchemaFootprint;
  }
  if (TypeIs(msg_type, "automsgs.msgs.geometry_msgs.TransformStampeds") ||
      TypeIs(msg_type, "automsgs.msgs.tf2_msgs.TFMessage")) {
    return rendering::kSchemaTfTree;
  }
  if (TypeIs(msg_type, "automsgs.msgs.sensor_msgs.PointCloud2")) {
    return rendering::kSchemaPointCloud2;
  }
  if (TypeIs(msg_type, "automsgs.msgs.sensor_msgs.Image")) {
    return rendering::kSchemaImage;
  }
  if (TypeIs(msg_type, "automsgs.msgs.geometry_msgs.Twist") ||
      TypeIs(msg_type, "automsgs.msgs.geometry_msgs.Twist2D") ||
      TypeIs(msg_type, "automsgs.msgs.geometry_msgs.TwistStamped")) {
    return rendering::kSchemaTwist2D;
  }
  if (TypeIs(msg_type, "automsgs.msgs.vehicle_msgs.RobotState")) {
    return rendering::kSchemaChassis;
  }
  if (TypeIs(msg_type, "automsgs.msgs.strata_msgs.SemanticZoneArray")) {
    return rendering::kSchemaSemanticZones;
  }
  if (TypeIs(msg_type, "automsgs.msgs.strata_msgs.FloorInfoArray")) {
    return rendering::kSchemaFloors;
  }
  // Obstacle / Prediction / Planning: no stable automsgs yet — documented gap.
  return {};
}

bool ConvertAutomsgsRaw(const std::string& channel, const std::string& msg_type,
                        const std::string& bytes, int64_t timestamp_ns,
                        core::StreamEnvelope* out) {
  if (!out) return false;
  out->channel = channel;
  out->timestamp_ns = timestamp_ns;
  out->encoding = "json";

  if (TypeIs(msg_type, "automsgs.msgs.geometry_msgs.Pose2D")) {
    return ConvertPose2D(bytes, out);
  }
  if (TypeIs(msg_type, "automsgs.msgs.geometry_msgs.Pose2DStamped")) {
    return ConvertPose2DStamped(bytes, out);
  }
  if (TypeIs(msg_type, "automsgs.msgs.geometry_msgs.PoseStamped")) {
    return ConvertPoseStamped(bytes, out);
  }
  if (TypeIs(msg_type, "automsgs.msgs.nav_msgs.Odometry")) {
    return ConvertOdometry(bytes, out);
  }
  if (TypeIs(msg_type, "automsgs.msgs.nav_msgs.Path")) {
    return ConvertPath(bytes, out);
  }
  if (TypeIs(msg_type, "automsgs.msgs.sensor_msgs.LaserScan")) {
    return ConvertLaserScan(bytes, out);
  }
  if (TypeIs(msg_type, "automsgs.msgs.map_msgs.OccupancyGrid")) {
    return ConvertOccupancyGrid(bytes, out);
  }
  if (TypeIs(msg_type, "automsgs.msgs.geometry_msgs.Polygon")) {
    return ConvertPolygon(bytes, out);
  }
  if (TypeIs(msg_type, "automsgs.msgs.geometry_msgs.PolygonStamped")) {
    return ConvertPolygonStamped(bytes, out);
  }
  if (TypeIs(msg_type, "automsgs.msgs.geometry_msgs.TransformStampeds")) {
    return ConvertTransformStampeds(bytes, out);
  }
  if (TypeIs(msg_type, "automsgs.msgs.tf2_msgs.TFMessage")) {
    return ConvertTfMessage(bytes, out);
  }
  if (TypeIs(msg_type, "automsgs.msgs.sensor_msgs.PointCloud2")) {
    return ConvertPointCloud2(bytes, out);
  }
  if (TypeIs(msg_type, "automsgs.msgs.sensor_msgs.Image")) {
    const bool depth_hint =
        channel.find("depth") != std::string::npos ||
        channel.find("Depth") != std::string::npos;
    return ConvertImageLike(bytes, out, depth_hint);
  }
  if (TypeIs(msg_type, "automsgs.msgs.geometry_msgs.Twist")) {
    return ConvertTwist(bytes, out);
  }
  if (TypeIs(msg_type, "automsgs.msgs.geometry_msgs.Twist2D")) {
    return ConvertTwist2DMsg(bytes, out);
  }
  if (TypeIs(msg_type, "automsgs.msgs.geometry_msgs.TwistStamped")) {
    return ConvertTwistStamped(bytes, out);
  }
  if (TypeIs(msg_type, "automsgs.msgs.vehicle_msgs.RobotState")) {
    return ConvertRobotState(bytes, out);
  }
  if (TypeIs(msg_type, "automsgs.msgs.strata_msgs.SemanticZoneArray")) {
    return ConvertSemanticZoneArray(bytes, out);
  }
  if (TypeIs(msg_type, "automsgs.msgs.strata_msgs.FloorInfoArray")) {
    return ConvertFloorInfoArray(bytes, out);
  }
  return false;
}

}  // namespace adapters
}  // namespace orbisview
}  // namespace autonomy
