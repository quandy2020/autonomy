#include "autonomy/autoviz/core/convert/message/convert_automsgs_body_3d.hpp"

#if defined(AUTOVIZ_HAS_FOXGLOVE) && AUTOVIZ_HAS_FOXGLOVE && \
    defined(AUTOVIZ_FOXGLOVE_HAS_ENCODE) && AUTOVIZ_FOXGLOVE_HAS_ENCODE

#include "autonomy/autoviz/core/convert/message/convert_nav_path_to_scene.hpp"
#include "autonomy/autoviz/core/convert/encode/foxglove_protobuf_encode.hpp"

#include "automsgs/msgs/geometry_msgs/pose_array.pb.h"
#include "automsgs/msgs/geometry_msgs/pose_stamped.pb.h"
#include "automsgs/msgs/geometry_msgs/transform_stamped.pb.h"
#include "automsgs/msgs/nav_msgs/grid_cells.pb.h"
#include "automsgs/msgs/nav_msgs/occupancy_grid.pb.h"
#include "automsgs/msgs/nav_msgs/odometry.pb.h"
#include "automsgs/msgs/nav_msgs/path.pb.h"
#include "automsgs/msgs/sensor_msgs/laser_scan.pb.h"
#include "automsgs/msgs/sensor_msgs/multi_echo_laser_scan.pb.h"
#include "automsgs/msgs/sensor_msgs/nav_sat_fix.pb.h"
#include "automsgs/msgs/sensor_msgs/point_cloud.pb.h"
#include "automsgs/msgs/sensor_msgs/point_cloud2.pb.h"
#include "automsgs/msgs/sensor_msgs/point_field.pb.h"
#include "automsgs/msgs/vision_msgs/bounding_box3d.pb.h"
#include "automsgs/msgs/vision_msgs/bounding_box3d_array.pb.h"
#include "automsgs/msgs/vision_msgs/detection3d.pb.h"
#include "automsgs/msgs/vision_msgs/detection3d_array.pb.h"
#include "automsgs/msgs/visualization_msgs/marker.pb.h"
#include "automsgs/msgs/visualization_msgs/marker_array.pb.h"

#include <foxglove/schemas.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>

namespace autoviz {
namespace converter {
namespace {

using NT = foxglove::schemas::PackedElementField::NumericType;

void CopySchemaToOut(const foxglove::Schema& fs, FoxgloveConvertedMessage* out) {
  out->schema_name = fs.name;
  out->schema_encoding = fs.encoding;
  if (fs.data != nullptr && fs.data_len > 0) {
    out->schema_data.assign(reinterpret_cast<const char*>(fs.data), fs.data_len);
  } else {
    out->schema_data.clear();
  }
}

foxglove::schemas::Timestamp StampFromHeader(
    const automsgs::msgs::std_msgs::Header& header) {
  foxglove::schemas::Timestamp ts;
  if (!header.has_stamp()) {
    return ts;
  }
  const auto& st = header.stamp();
  ts.sec = st.sec() < 0 ? 0u : static_cast<uint32_t>(st.sec());
  ts.nsec = st.nanosec();
  return ts;
}

foxglove::schemas::Color ColorFromRgba(const automsgs::msgs::std_msgs::ColorRGBA& c) {
  foxglove::schemas::Color out;
  out.r = c.r();
  out.g = c.g();
  out.b = c.b();
  out.a = c.a() > 0.0 ? c.a() : 1.0;
  return out;
}

std::optional<foxglove::schemas::Quaternion> QuatFromGeom(
    const automsgs::msgs::geometry_msgs::Quaternion& q) {
  foxglove::schemas::Quaternion oq;
  oq.x = q.x();
  oq.y = q.y();
  oq.z = q.z();
  oq.w = q.w();
  return oq;
}

std::optional<foxglove::schemas::Vector3> Vec3FromPoint(
    const automsgs::msgs::geometry_msgs::Point& p) {
  foxglove::schemas::Vector3 v;
  v.x = p.x();
  v.y = p.y();
  v.z = p.z();
  return v;
}

std::optional<foxglove::schemas::Pose> FoxPoseFromGeom(
    const automsgs::msgs::geometry_msgs::Pose& p) {
  foxglove::schemas::Pose fp;
  if (p.has_position()) {
    fp.position = Vec3FromPoint(p.position());
  }
  if (p.has_orientation()) {
    fp.orientation = QuatFromGeom(p.orientation());
  }
  return fp;
}

std::optional<foxglove::schemas::Duration> DurationFromRos(
    const automsgs::msgs::builtin_interfaces::Duration& d) {
  foxglove::schemas::Duration fd;
  fd.sec = d.sec();
  fd.nsec = d.nanosec() >= 0 ? static_cast<uint32_t>(d.nanosec()) : 0u;
  return fd;
}

std::string MarkerEntityId(const automsgs::msgs::visualization_msgs::Marker& m) {
  if (m.ns().empty()) {
    return std::to_string(m.id());
  }
  return m.ns() + "_" + std::to_string(m.id());
}

foxglove::schemas::Color ColorFromIndex(size_t idx) {
  static constexpr double kPal[][4] = {
      {0.9, 0.2, 0.2, 0.9},  {0.2, 0.85, 0.25, 0.9}, {0.25, 0.35, 0.95, 0.9},
      {0.9, 0.85, 0.2, 0.9}, {0.9, 0.35, 0.75, 0.9}, {0.35, 0.9, 0.9, 0.9},
      {0.55, 0.35, 0.2, 0.9}, {0.5, 0.5, 0.55, 0.9}, {0.85, 0.5, 0.2, 0.9},
      {0.4, 0.2, 0.65, 0.9}, {0.65, 0.9, 0.4, 0.9}, {0.3, 0.55, 0.35, 0.9},
  };
  const double* c = kPal[idx % 12];
  foxglove::schemas::Color o;
  o.r = c[0];
  o.g = c[1];
  o.b = c[2];
  o.a = c[3];
  return o;
}

void AddBboxCubeEntity(const automsgs::msgs::vision_msgs::BoundingBox3D& bbox,
                       const std::string& entity_id,
                       const automsgs::msgs::std_msgs::Header* hdr,
                       const foxglove::schemas::Color& color,
                       foxglove::schemas::SceneUpdate* scene) {
  if (!bbox.has_center()) {
    return;
  }
  foxglove::schemas::SceneEntity ent;
  if (hdr != nullptr) {
    ent.timestamp = StampFromHeader(*hdr);
    ent.frame_id = hdr->frame_id();
  }
  ent.id = entity_id;
  ent.frame_locked = false;
  foxglove::schemas::CubePrimitive cube;
  cube.pose = FoxPoseFromGeom(bbox.center());
  foxglove::schemas::Vector3 sz;
  const double sx = bbox.has_size() ? static_cast<double>(bbox.size().x()) : 0.1;
  const double sy = bbox.has_size() ? static_cast<double>(bbox.size().y()) : 0.1;
  const double szv = bbox.has_size() ? static_cast<double>(bbox.size().z()) : 0.1;
  sz.x = std::max(1e-4, sx);
  sz.y = std::max(1e-4, sy);
  sz.z = std::max(1e-4, szv);
  cube.size = sz;
  cube.color = color;
  ent.cubes.push_back(std::move(cube));
  scene->entities.push_back(std::move(ent));
}

void AppendDetection3DToScene(const automsgs::msgs::vision_msgs::Detection3D& det,
                              size_t color_index,
                              foxglove::schemas::SceneUpdate* scene) {
  const foxglove::schemas::Color color = ColorFromIndex(color_index);
  const std::string eid =
      det.id().empty() ? ("detection3d_" + std::to_string(color_index)) : det.id();
  if (det.has_bbox()) {
    AddBboxCubeEntity(det.bbox(), eid, &det.header(), color, scene);
  }

  std::ostringstream oss;
  if (!det.id().empty()) {
    oss << det.id() << "\n";
  }
  for (int i = 0; i < det.results_size(); ++i) {
    const auto& r = det.results(i);
    if (r.has_hypothesis()) {
      oss << r.hypothesis().class_id() << ":" << r.hypothesis().score() << " ";
    }
  }
  const std::string label = oss.str();
  if (label.empty()) {
    return;
  }

  foxglove::schemas::SceneEntity text_ent;
  text_ent.timestamp = StampFromHeader(det.header());
  text_ent.frame_id = det.header().frame_id();
  text_ent.id = eid + "_label";
  text_ent.frame_locked = false;
  foxglove::schemas::TextPrimitive tx;
  tx.billboard = true;
  tx.font_size = 0.14;
  tx.scale_invariant = false;
  tx.color = color;
  tx.text = label;
  if (det.has_bbox() && det.bbox().has_center()) {
    automsgs::msgs::geometry_msgs::Pose pose_label = det.bbox().center();
    if (pose_label.has_position()) {
      const float dz =
          det.bbox().has_size() ? det.bbox().size().z() * 0.5f + 0.08f : 0.12f;
      pose_label.mutable_position()->set_z(pose_label.position().z() + dz);
    }
    tx.pose = FoxPoseFromGeom(pose_label);
  } else {
    foxglove::schemas::Pose zp;
    foxglove::schemas::Vector3 o;
    o.x = 0;
    o.y = 0;
    o.z = 0;
    zp.position = o;
    tx.pose = zp;
  }
  text_ent.texts.push_back(std::move(tx));
  scene->entities.push_back(std::move(text_ent));
}

NT MapRosPointFieldType(automsgs::msgs::sensor_msgs::PointField::DataType dt) {
  using PF = automsgs::msgs::sensor_msgs::PointField;
  switch (dt) {
    case PF::INT8:
      return NT::INT8;
    case PF::UINT8:
      return NT::UINT8;
    case PF::INT16:
      return NT::INT16;
    case PF::UINT16:
      return NT::UINT16;
    case PF::INT32:
      return NT::INT32;
    case PF::UINT32:
      return NT::UINT32;
    case PF::FLOAT32:
      return NT::FLOAT32;
    case PF::FLOAT64:
      return NT::FLOAT64;
    default:
      return NT::UNKNOWN;
  }
}

bool FillLaserScan(const void* data, size_t len, FoxgloveConvertedMessage* out) {
  automsgs::msgs::sensor_msgs::LaserScan scan;
  if (!scan.ParseFromArray(data, static_cast<int>(len))) {
    return false;
  }
  const int n = scan.ranges_size();
  if (n <= 0) {
    return false;
  }
  foxglove::schemas::LaserScan fg;
  fg.timestamp = StampFromHeader(scan.header());
  fg.frame_id = scan.header().frame_id();
  fg.pose = std::nullopt;
  fg.start_angle = static_cast<double>(scan.angle_min());
  if (n == 1) {
    fg.end_angle = fg.start_angle;
  } else {
    fg.end_angle = static_cast<double>(scan.angle_min() +
                                        static_cast<float>(n - 1) * scan.angle_increment());
  }
  fg.ranges.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    fg.ranges.push_back(static_cast<double>(scan.ranges(i)));
  }
  if (scan.intensities_size() == n) {
    fg.intensities.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
      fg.intensities.push_back(static_cast<double>(scan.intensities(i)));
    }
  }
  std::string payload = SerializeFoxglove(std::move(fg));
  if (payload.empty()) {
    return false;
  }
  CopySchemaToOut(foxglove::schemas::LaserScan::schema(), out);
  out->payload = std::move(payload);
  out->ok = true;
  return true;
}

bool FillPointCloud2(const void* data, size_t len, FoxgloveConvertedMessage* out) {
  automsgs::msgs::sensor_msgs::PointCloud2 pc;
  if (!pc.ParseFromArray(data, static_cast<int>(len))) {
    return false;
  }
  if (pc.is_bigendian()) {
    return false;
  }
  const uint32_t w = pc.width();
  const uint32_t h = pc.height();
  if (w == 0 || h == 0 || pc.point_step() == 0) {
    return false;
  }
  if (pc.data().size() < static_cast<size_t>(pc.row_step()) * static_cast<size_t>(h)) {
    return false;
  }
  foxglove::schemas::PointCloud fg;
  fg.timestamp = StampFromHeader(pc.header());
  fg.frame_id = pc.header().frame_id();
  fg.pose = std::nullopt;
  fg.point_stride = pc.point_step();
  fg.fields.reserve(static_cast<size_t>(pc.fields_size()));
  for (int i = 0; i < pc.fields_size(); ++i) {
    const auto& f = pc.fields(i);
    foxglove::schemas::PackedElementField pf;
    pf.name = f.name();
    pf.offset = f.offset();
    pf.type = MapRosPointFieldType(f.datatype());
    if (pf.type == NT::UNKNOWN) {
      return false;
    }
    fg.fields.push_back(std::move(pf));
  }
  fg.data.resize(pc.data().size());
  if (!fg.data.empty()) {
    std::memcpy(fg.data.data(), pc.data().data(), fg.data.size());
  }
  std::string payload = SerializeFoxglove(std::move(fg));
  if (payload.empty()) {
    return false;
  }
  CopySchemaToOut(foxglove::schemas::PointCloud::schema(), out);
  out->payload = std::move(payload);
  out->ok = true;
  return true;
}

bool FillPointCloudLegacy(const void* data, size_t len, FoxgloveConvertedMessage* out) {
  automsgs::msgs::sensor_msgs::PointCloud pc;
  if (!pc.ParseFromArray(data, static_cast<int>(len))) {
    return false;
  }
  const int n = pc.points_size();
  if (n <= 0) {
    return false;
  }
  foxglove::schemas::PointCloud fg;
  fg.timestamp = StampFromHeader(pc.header());
  fg.frame_id = pc.header().frame_id();
  fg.pose = std::nullopt;
  fg.point_stride = 12;
  {
    foxglove::schemas::PackedElementField fx;
    fx.name = "x";
    fx.offset = 0;
    fx.type = NT::FLOAT32;
    fg.fields.push_back(fx);
    foxglove::schemas::PackedElementField fy;
    fy.name = "y";
    fy.offset = 4;
    fy.type = NT::FLOAT32;
    fg.fields.push_back(fy);
    foxglove::schemas::PackedElementField fz;
    fz.name = "z";
    fz.offset = 8;
    fz.type = NT::FLOAT32;
    fg.fields.push_back(fz);
  }
  fg.data.resize(static_cast<size_t>(n) * 12);
  uint8_t* dst = reinterpret_cast<uint8_t*>(fg.data.data());
  for (int i = 0; i < n; ++i) {
    const auto& pt = pc.points(i);
    float x = pt.x();
    float y = pt.y();
    float z = pt.z();
    std::memcpy(dst + i * 12, &x, 4);
    std::memcpy(dst + i * 12 + 4, &y, 4);
    std::memcpy(dst + i * 12 + 8, &z, 4);
  }
  std::string payload = SerializeFoxglove(std::move(fg));
  if (payload.empty()) {
    return false;
  }
  CopySchemaToOut(foxglove::schemas::PointCloud::schema(), out);
  out->payload = std::move(payload);
  out->ok = true;
  return true;
}

bool FillOccupancyGrid(const void* data, size_t len, FoxgloveConvertedMessage* out) {
  automsgs::msgs::nav_msgs::OccupancyGrid grid;
  if (!grid.ParseFromArray(data, static_cast<int>(len)) || !grid.has_info()) {
    return false;
  }
  const auto& info = grid.info();
  const uint32_t width = info.width();
  const uint32_t height = info.height();
  if (width == 0 || height == 0) {
    return false;
  }
  const size_t expected = static_cast<size_t>(width) * static_cast<size_t>(height);
  if (static_cast<size_t>(grid.data_size()) < expected) {
    return false;
  }
  foxglove::schemas::Grid fg;
  fg.timestamp = StampFromHeader(grid.header());
  fg.frame_id = grid.header().frame_id();
  if (info.has_origin()) {
    fg.pose = FoxPoseFromGeom(info.origin());
  }
  fg.column_count = width;
  foxglove::schemas::Vector2 cs;
  cs.x = info.resolution();
  cs.y = info.resolution();
  fg.cell_size = cs;
  fg.row_stride = width;
  fg.cell_stride = 1;
  foxglove::schemas::PackedElementField occ;
  occ.name = "occupancy";
  occ.offset = 0;
  occ.type = NT::INT8;
  fg.fields.push_back(occ);
  fg.data.resize(expected);
  for (size_t i = 0; i < expected; ++i) {
    int32_t v = grid.data(static_cast<int>(i));
    int8_t b = 0;
    if (v > 127) {
      b = 127;
    } else if (v < -128) {
      b = -128;
    } else {
      b = static_cast<int8_t>(v);
    }
    fg.data[i] = static_cast<std::byte>(b);
  }
  std::string payload = SerializeFoxglove(std::move(fg));
  if (payload.empty()) {
    return false;
  }
  CopySchemaToOut(foxglove::schemas::Grid::schema(), out);
  out->payload = std::move(payload);
  out->ok = true;
  return true;
}

bool FillNavSatFix(const void* data, size_t len, FoxgloveConvertedMessage* out) {
  automsgs::msgs::sensor_msgs::NavSatFix fix;
  if (!fix.ParseFromArray(data, static_cast<int>(len))) {
    return false;
  }
  foxglove::schemas::LocationFix fg;
  fg.timestamp = StampFromHeader(fix.header());
  fg.frame_id = fix.header().frame_id();
  fg.latitude = fix.latitude();
  fg.longitude = fix.longitude();
  fg.altitude = std::isfinite(fix.altitude()) ? fix.altitude() : 0.0;
  fg.position_covariance.fill(0.0);
  for (int i = 0; i < std::min(9, fix.position_covariance_size()); ++i) {
    fg.position_covariance[static_cast<size_t>(i)] = fix.position_covariance(i);
  }
  using NS = automsgs::msgs::sensor_msgs::NavSatFix;
  using LF = foxglove::schemas::LocationFix;
  switch (fix.position_covariance_type()) {
    case NS::COVARIANCE_TYPE_UNKNOWN:
      fg.position_covariance_type = LF::PositionCovarianceType::UNKNOWN;
      break;
    case NS::COVARIANCE_TYPE_APPROXIMATED:
      fg.position_covariance_type = LF::PositionCovarianceType::APPROXIMATED;
      break;
    case NS::COVARIANCE_TYPE_DIAGONAL_KNOWN:
      fg.position_covariance_type = LF::PositionCovarianceType::DIAGONAL_KNOWN;
      break;
    case NS::COVARIANCE_TYPE_KNOWN:
      fg.position_covariance_type = LF::PositionCovarianceType::KNOWN;
      break;
    default:
      fg.position_covariance_type = LF::PositionCovarianceType::UNKNOWN;
      break;
  }
  foxglove::schemas::Color pin;
  pin.r = 1.0;
  pin.g = 0.2;
  pin.b = 0.2;
  pin.a = 1.0;
  fg.color = pin;
  std::string payload = SerializeFoxglove(std::move(fg));
  if (payload.empty()) {
    return false;
  }
  CopySchemaToOut(foxglove::schemas::LocationFix::schema(), out);
  out->payload = std::move(payload);
  out->ok = true;
  return true;
}

bool FillPoseStampedLike(const automsgs::msgs::geometry_msgs::Pose& pose,
                         const automsgs::msgs::std_msgs::Header& header,
                         FoxgloveConvertedMessage* out) {
  foxglove::schemas::PoseInFrame pf;
  pf.timestamp = StampFromHeader(header);
  pf.frame_id = header.frame_id();
  pf.pose = FoxPoseFromGeom(pose);
  std::string payload = SerializeFoxglove(std::move(pf));
  if (payload.empty()) {
    return false;
  }
  CopySchemaToOut(foxglove::schemas::PoseInFrame::schema(), out);
  out->payload = std::move(payload);
  out->ok = true;
  return true;
}

bool FillPoseStamped(const void* data, size_t len, FoxgloveConvertedMessage* out) {
  automsgs::msgs::geometry_msgs::PoseStamped m;
  if (!m.ParseFromArray(data, static_cast<int>(len)) || !m.has_pose()) {
    return false;
  }
  return FillPoseStampedLike(m.pose(), m.header(), out);
}

bool FillOdometry(const void* data, size_t len, FoxgloveConvertedMessage* out) {
  automsgs::msgs::nav_msgs::Odometry odom;
  if (!odom.ParseFromArray(data, static_cast<int>(len)) || !odom.has_pose()) {
    return false;
  }
  const auto& pwc = odom.pose();
  if (!pwc.has_pose() || !pwc.pose().has_pose()) {
    return false;
  }
  return FillPoseStampedLike(pwc.pose().pose(), odom.header(), out);
}

bool FillTransformStamped(const void* data, size_t len, FoxgloveConvertedMessage* out) {
  automsgs::msgs::geometry_msgs::TransformStamped m;
  if (!m.ParseFromArray(data, static_cast<int>(len)) || !m.has_transform()) {
    return false;
  }
  foxglove::schemas::FrameTransform ft;
  ft.timestamp = StampFromHeader(m.header());
  ft.parent_frame_id = m.header().frame_id();
  ft.child_frame_id = m.child_frame_id();
  const auto& tr = m.transform();
  if (tr.has_translation()) {
    foxglove::schemas::Vector3 v;
    v.x = tr.translation().x();
    v.y = tr.translation().y();
    v.z = tr.translation().z();
    ft.translation = v;
  }
  if (tr.has_rotation()) {
    ft.rotation = QuatFromGeom(tr.rotation());
  }
  std::string payload = SerializeFoxglove(std::move(ft));
  if (payload.empty()) {
    return false;
  }
  CopySchemaToOut(foxglove::schemas::FrameTransform::schema(), out);
  out->payload = std::move(payload);
  out->ok = true;
  return true;
}

bool FillPoseArray(const void* data, size_t len, FoxgloveConvertedMessage* out) {
  automsgs::msgs::geometry_msgs::PoseArray arr;
  if (!arr.ParseFromArray(data, static_cast<int>(len))) {
    return false;
  }
  foxglove::schemas::PosesInFrame pf;
  pf.timestamp = StampFromHeader(arr.header());
  pf.frame_id = arr.header().frame_id();
  for (int i = 0; i < arr.poses_size(); ++i) {
    auto fp = FoxPoseFromGeom(arr.poses(i));
    if (fp.has_value()) {
      pf.poses.push_back(*fp);
    }
  }
  std::string payload = SerializeFoxglove(std::move(pf));
  if (payload.empty()) {
    return false;
  }
  CopySchemaToOut(foxglove::schemas::PosesInFrame::schema(), out);
  out->payload = std::move(payload);
  out->ok = true;
  return true;
}

void AppendMarkerToScene(const automsgs::msgs::visualization_msgs::Marker& m,
                         foxglove::schemas::SceneUpdate* scene) {
  using M = automsgs::msgs::visualization_msgs::Marker;
  if (m.action() == M::DELETE) {
    foxglove::schemas::SceneEntityDeletion del;
    del.type = foxglove::schemas::SceneEntityDeletion::SceneEntityDeletionType::MATCHING_ID;
    del.id = MarkerEntityId(m);
    scene->deletions.push_back(std::move(del));
    return;
  }
  if (m.action() == M::DELETEALL) {
    foxglove::schemas::SceneEntityDeletion del;
    del.type = foxglove::schemas::SceneEntityDeletion::SceneEntityDeletionType::ALL;
    scene->deletions.push_back(std::move(del));
    return;
  }

  foxglove::schemas::SceneEntity entity;
  entity.timestamp = StampFromHeader(m.header());
  entity.frame_id = m.header().frame_id();
  entity.id = MarkerEntityId(m);
  entity.frame_locked = m.frame_locked();
  if (m.has_lifetime()) {
    entity.lifetime = DurationFromRos(m.lifetime());
  }

  const double sx = m.has_scale() ? std::max(1e-6, static_cast<double>(m.scale().x())) : 1.0;
  const double sy = m.has_scale() ? std::max(1e-6, static_cast<double>(m.scale().y())) : 1.0;
  const double sz = m.has_scale() ? std::max(1e-6, static_cast<double>(m.scale().z())) : 1.0;
  std::optional<foxglove::schemas::Color> col;
  if (m.has_color()) {
    col = ColorFromRgba(m.color());
  }

  auto base_pose = m.has_pose() ? FoxPoseFromGeom(m.pose()) : std::optional<foxglove::schemas::Pose>{};

  switch (m.type()) {
    case M::ARROW: {
      foxglove::schemas::ArrowPrimitive ar;
      ar.pose = base_pose;
      ar.shaft_length = sx;
      ar.shaft_diameter = std::max(0.02, sy * 0.2);
      ar.head_length = std::max(0.05, sx * 0.2);
      ar.head_diameter = std::max(0.04, sz * 0.3);
      ar.color = col;
      entity.arrows.push_back(std::move(ar));
      break;
    }
    case M::CUBE: {
      foxglove::schemas::CubePrimitive c;
      c.pose = base_pose;
      foxglove::schemas::Vector3 dim;
      dim.x = sx;
      dim.y = sy;
      dim.z = sz;
      c.size = dim;
      c.color = col;
      entity.cubes.push_back(std::move(c));
      break;
    }
    case M::SPHERE: {
      foxglove::schemas::SpherePrimitive s;
      s.pose = base_pose;
      foxglove::schemas::Vector3 dim;
      dim.x = sx;
      dim.y = sy;
      dim.z = sz;
      s.size = dim;
      s.color = col;
      entity.spheres.push_back(std::move(s));
      break;
    }
    case M::CYLINDER: {
      foxglove::schemas::CylinderPrimitive cy;
      cy.pose = base_pose;
      foxglove::schemas::Vector3 dim;
      dim.x = sx;
      dim.y = sy;
      dim.z = sz;
      cy.size = dim;
      cy.color = col;
      entity.cylinders.push_back(std::move(cy));
      break;
    }
    case M::LINE_STRIP:
    case M::LINE_LIST: {
      if (m.points_size() < 2 && m.type() == M::LINE_STRIP) {
        return;
      }
      foxglove::schemas::LinePrimitive line;
      line.type = (m.type() == M::LINE_STRIP)
                      ? foxglove::schemas::LinePrimitive::LineType::LINE_STRIP
                      : foxglove::schemas::LinePrimitive::LineType::LINE_LIST;
      line.pose = base_pose;
      line.thickness = std::max(0.02, sy * 0.1);
      line.scale_invariant = false;
      line.color = col;
      for (int i = 0; i < m.points_size(); ++i) {
        const auto& p = m.points(i);
        foxglove::schemas::Point3 pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        line.points.push_back(pt);
      }
      if (!line.points.empty()) {
        entity.lines.push_back(std::move(line));
      }
      break;
    }
    case M::POINTS: {
      const double diam = std::min(0.2, std::max(0.02, std::min({sx, sy, sz}) * 0.1));
      const int max_pts = 10000;
      for (int i = 0; i < m.points_size() && i < max_pts; ++i) {
        const auto& p = m.points(i);
        foxglove::schemas::SpherePrimitive s;
        foxglove::schemas::Pose sp;
        foxglove::schemas::Vector3 pos;
        pos.x = p.x();
        pos.y = p.y();
        pos.z = p.z();
        sp.position = pos;
        s.pose = sp;
        foxglove::schemas::Vector3 dim;
        dim.x = dim.y = dim.z = diam;
        s.size = dim;
        if (col.has_value()) {
          s.color = col;
        }
        entity.spheres.push_back(std::move(s));
      }
      break;
    }
    case M::CUBE_LIST: {
      for (int i = 0; i < m.points_size(); ++i) {
        const auto& p = m.points(i);
        foxglove::schemas::CubePrimitive c;
        foxglove::schemas::Pose sp;
        foxglove::schemas::Vector3 pos;
        pos.x = p.x();
        pos.y = p.y();
        pos.z = p.z();
        sp.position = pos;
        c.pose = sp;
        foxglove::schemas::Vector3 dim;
        dim.x = sx;
        dim.y = sy;
        dim.z = sz;
        c.size = dim;
        c.color = col;
        entity.cubes.push_back(std::move(c));
      }
      break;
    }
    case M::SPHERE_LIST: {
      for (int i = 0; i < m.points_size(); ++i) {
        const auto& p = m.points(i);
        foxglove::schemas::SpherePrimitive s;
        foxglove::schemas::Pose sp;
        foxglove::schemas::Vector3 pos;
        pos.x = p.x();
        pos.y = p.y();
        pos.z = p.z();
        sp.position = pos;
        s.pose = sp;
        foxglove::schemas::Vector3 dim;
        dim.x = sx;
        dim.y = sy;
        dim.z = sz;
        s.size = dim;
        s.color = col;
        entity.spheres.push_back(std::move(s));
      }
      break;
    }
    case M::TEXT_VIEW_FACING: {
      foxglove::schemas::TextPrimitive tx;
      tx.pose = base_pose;
      tx.billboard = true;
      tx.font_size = std::max(0.2, sy);
      tx.scale_invariant = false;
      tx.color = col;
      tx.text = m.text();
      entity.texts.push_back(std::move(tx));
      break;
    }
    case M::MESH_RESOURCE: {
      foxglove::schemas::ModelPrimitive model;
      model.pose = base_pose;
      model.url = m.mesh_resource();
      model.override_color = col.has_value();
      model.color = col;
      entity.models.push_back(std::move(model));
      break;
    }
    case M::TRIANGLE_LIST: {
      if (m.points_size() < 3) {
        return;
      }
      foxglove::schemas::TriangleListPrimitive tri;
      tri.pose = base_pose;
      tri.color = col;
      for (int i = 0; i < m.points_size(); ++i) {
        const auto& p = m.points(i);
        foxglove::schemas::Point3 pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        tri.points.push_back(pt);
      }
      entity.triangles.push_back(std::move(tri));
      break;
    }
    default:
      return;
  }

  if (entity.arrows.empty() && entity.cubes.empty() && entity.spheres.empty() &&
      entity.cylinders.empty() && entity.lines.empty() && entity.triangles.empty() &&
      entity.texts.empty() && entity.models.empty()) {
    return;
  }
  scene->entities.push_back(std::move(entity));
}

bool FillMarker(const void* data, size_t len, FoxgloveConvertedMessage* out) {
  automsgs::msgs::visualization_msgs::Marker m;
  if (!m.ParseFromArray(data, static_cast<int>(len))) {
    return false;
  }
  foxglove::schemas::SceneUpdate scene;
  AppendMarkerToScene(m, &scene);
  if (scene.entities.empty() && scene.deletions.empty()) {
    return false;
  }
  std::string payload = SerializeFoxglove(std::move(scene));
  if (payload.empty()) {
    return false;
  }
  CopySchemaToOut(foxglove::schemas::SceneUpdate::schema(), out);
  out->payload = std::move(payload);
  out->ok = true;
  return true;
}

bool FillMarkerArray(const void* data, size_t len, FoxgloveConvertedMessage* out) {
  automsgs::msgs::visualization_msgs::MarkerArray arr;
  if (!arr.ParseFromArray(data, static_cast<int>(len))) {
    return false;
  }
  foxglove::schemas::SceneUpdate scene;
  for (int i = 0; i < arr.markers_size(); ++i) {
    AppendMarkerToScene(arr.markers(i), &scene);
  }
  if (scene.entities.empty() && scene.deletions.empty()) {
    return false;
  }
  std::string payload = SerializeFoxglove(std::move(scene));
  if (payload.empty()) {
    return false;
  }
  CopySchemaToOut(foxglove::schemas::SceneUpdate::schema(), out);
  out->payload = std::move(payload);
  out->ok = true;
  return true;
}

bool FillPath(const void* data, size_t len, FoxgloveConvertedMessage* out) {
  automsgs::msgs::nav_msgs::Path path;
  if (!path.ParseFromArray(data, static_cast<int>(len))) {
    return false;
  }
  foxglove::schemas::SceneUpdate scene = PathToSceneUpdate(path);
  std::string payload = SerializeFoxglove(std::move(scene));
  if (payload.empty()) {
    return false;
  }
  CopySchemaToOut(foxglove::schemas::SceneUpdate::schema(), out);
  out->payload = std::move(payload);
  out->ok = true;
  return true;
}

bool FillBoundingBox3D(const void* data, size_t len, FoxgloveConvertedMessage* out) {
  automsgs::msgs::vision_msgs::BoundingBox3D bbox;
  if (!bbox.ParseFromArray(data, static_cast<int>(len)) || !bbox.has_center()) {
    return false;
  }
  foxglove::schemas::SceneUpdate scene;
  AddBboxCubeEntity(bbox, "bounding_box3d", /*hdr=*/nullptr, ColorFromIndex(0), &scene);
  if (scene.entities.empty()) {
    return false;
  }
  std::string payload = SerializeFoxglove(std::move(scene));
  if (payload.empty()) {
    return false;
  }
  CopySchemaToOut(foxglove::schemas::SceneUpdate::schema(), out);
  out->payload = std::move(payload);
  out->ok = true;
  return true;
}

bool FillBoundingBox3DArray(const void* data, size_t len, FoxgloveConvertedMessage* out) {
  automsgs::msgs::vision_msgs::BoundingBox3DArray arr;
  if (!arr.ParseFromArray(data, static_cast<int>(len)) || arr.boxes_size() == 0) {
    return false;
  }
  foxglove::schemas::SceneUpdate scene;
  for (int i = 0; i < arr.boxes_size(); ++i) {
    AddBboxCubeEntity(arr.boxes(i), "bbox_" + std::to_string(i), &arr.header(),
                      ColorFromIndex(static_cast<size_t>(i)), &scene);
  }
  if (scene.entities.empty()) {
    return false;
  }
  std::string payload = SerializeFoxglove(std::move(scene));
  if (payload.empty()) {
    return false;
  }
  CopySchemaToOut(foxglove::schemas::SceneUpdate::schema(), out);
  out->payload = std::move(payload);
  out->ok = true;
  return true;
}

bool FillDetection3D(const void* data, size_t len, FoxgloveConvertedMessage* out) {
  automsgs::msgs::vision_msgs::Detection3D det;
  if (!det.ParseFromArray(data, static_cast<int>(len))) {
    return false;
  }
  foxglove::schemas::SceneUpdate scene;
  AppendDetection3DToScene(det, 0, &scene);
  if (scene.entities.empty()) {
    return false;
  }
  std::string payload = SerializeFoxglove(std::move(scene));
  if (payload.empty()) {
    return false;
  }
  CopySchemaToOut(foxglove::schemas::SceneUpdate::schema(), out);
  out->payload = std::move(payload);
  out->ok = true;
  return true;
}

bool FillDetection3DArray(const void* data, size_t len, FoxgloveConvertedMessage* out) {
  automsgs::msgs::vision_msgs::Detection3DArray arr;
  if (!arr.ParseFromArray(data, static_cast<int>(len)) || arr.detections_size() == 0) {
    return false;
  }
  foxglove::schemas::SceneUpdate scene;
  for (int i = 0; i < arr.detections_size(); ++i) {
    AppendDetection3DToScene(arr.detections(i), static_cast<size_t>(i), &scene);
  }
  if (scene.entities.empty()) {
    return false;
  }
  std::string payload = SerializeFoxglove(std::move(scene));
  if (payload.empty()) {
    return false;
  }
  CopySchemaToOut(foxglove::schemas::SceneUpdate::schema(), out);
  out->payload = std::move(payload);
  out->ok = true;
  return true;
}

bool FillMultiEchoAsPointCloud(const void* data, size_t len, FoxgloveConvertedMessage* out) {
  automsgs::msgs::sensor_msgs::MultiEchoLaserScan scan;
  if (!scan.ParseFromArray(data, static_cast<int>(len))) {
    return false;
  }
  const int n = scan.ranges_size();
  if (n <= 0) {
    return false;
  }
  size_t valid = 0;
  for (int i = 0; i < n; ++i) {
    const auto& le = scan.ranges(i);
    for (int j = 0; j < le.echoes_size(); ++j) {
      const float r = le.echoes(j);
      if (std::isfinite(r) && r >= scan.range_min() && r <= scan.range_max()) {
        ++valid;
      }
    }
  }
  if (valid == 0) {
    return false;
  }

  foxglove::schemas::PointCloud fg;
  fg.timestamp = StampFromHeader(scan.header());
  fg.frame_id = scan.header().frame_id();
  fg.pose = std::nullopt;
  fg.point_stride = 12;
  {
    foxglove::schemas::PackedElementField fx;
    fx.name = "x";
    fx.offset = 0;
    fx.type = NT::FLOAT32;
    fg.fields.push_back(fx);
    foxglove::schemas::PackedElementField fy;
    fy.name = "y";
    fy.offset = 4;
    fy.type = NT::FLOAT32;
    fg.fields.push_back(fy);
    foxglove::schemas::PackedElementField fz;
    fz.name = "z";
    fz.offset = 8;
    fz.type = NT::FLOAT32;
    fg.fields.push_back(fz);
  }
  fg.data.resize(valid * 12);
  uint8_t* dst = reinterpret_cast<uint8_t*>(fg.data.data());
  size_t off = 0;
  for (int i = 0; i < n; ++i) {
    const float ang = scan.angle_min() + static_cast<float>(i) * scan.angle_increment();
    const float c = std::cos(static_cast<double>(ang));
    const float s = std::sin(static_cast<double>(ang));
    const auto& le = scan.ranges(i);
    for (int j = 0; j < le.echoes_size(); ++j) {
      const float r = le.echoes(j);
      if (!std::isfinite(r) || r < scan.range_min() || r > scan.range_max()) {
        continue;
      }
      const float x = r * c;
      const float y = r * s;
      const float z = 0.f;
      std::memcpy(dst + off, &x, 4);
      std::memcpy(dst + off + 4, &y, 4);
      std::memcpy(dst + off + 8, &z, 4);
      off += 12;
    }
  }

  std::string payload = SerializeFoxglove(std::move(fg));
  if (payload.empty()) {
    return false;
  }
  CopySchemaToOut(foxglove::schemas::PointCloud::schema(), out);
  out->payload = std::move(payload);
  out->ok = true;
  return true;
}

bool FillGridCells(const void* data, size_t len, FoxgloveConvertedMessage* out) {
  automsgs::msgs::nav_msgs::GridCells grid;
  if (!grid.ParseFromArray(data, static_cast<int>(len)) || grid.cells_size() == 0) {
    return false;
  }
  const float cw = grid.cell_width() > 1e-6f ? grid.cell_width() : 1.f;
  const float ch = grid.cell_height() > 1e-6f ? grid.cell_height() : 1.f;
  const float cz = std::max(1e-3f, std::min(cw, ch) * 0.2f);

  foxglove::schemas::SceneUpdate scene;
  foxglove::schemas::SceneEntity ent;
  ent.timestamp = StampFromHeader(grid.header());
  ent.frame_id = grid.header().frame_id();
  ent.id = "grid_cells";
  ent.frame_locked = false;
  foxglove::schemas::Color cell_color;
  cell_color.r = 0.35;
  cell_color.g = 0.65;
  cell_color.b = 0.95;
  cell_color.a = 0.75;

  for (int i = 0; i < grid.cells_size(); ++i) {
    const auto& pt = grid.cells(i);
    foxglove::schemas::CubePrimitive cube;
    foxglove::schemas::Pose pose;
    foxglove::schemas::Vector3 pos;
    pos.x = pt.x();
    pos.y = pt.y();
    pos.z = pt.z();
    pose.position = pos;
    cube.pose = pose;
    foxglove::schemas::Vector3 sz;
    sz.x = cw;
    sz.y = ch;
    sz.z = cz;
    cube.size = sz;
    cube.color = cell_color;
    ent.cubes.push_back(std::move(cube));
  }
  scene.entities.push_back(std::move(ent));

  std::string payload = SerializeFoxglove(std::move(scene));
  if (payload.empty()) {
    return false;
  }
  CopySchemaToOut(foxglove::schemas::SceneUpdate::schema(), out);
  out->payload = std::move(payload);
  out->ok = true;
  return true;
}

}  // namespace

bool TryConvertBodyToFoxglove(AutomsgsFoxgloveStrategy strategy,
                                const std::string& proto_full_name,
                                const void* serialized_data,
                                const std::size_t serialized_size,
                                FoxgloveConvertedMessage* out) {
  if (out == nullptr || serialized_data == nullptr || serialized_size == 0) {
    return false;
  }
  out->ok = false;
  out->payload.clear();
  out->schema_name.clear();
  out->schema_data.clear();

  using S = AutomsgsFoxgloveStrategy;

  if (strategy == S::kConvertToSceneUpdate) {
    if (proto_full_name == "automsgs.msgs.nav_msgs.Path") {
      return FillPath(serialized_data, serialized_size, out);
    }
    if (proto_full_name == "automsgs.msgs.visualization_msgs.Marker") {
      return FillMarker(serialized_data, serialized_size, out);
    }
    if (proto_full_name == "automsgs.msgs.visualization_msgs.MarkerArray") {
      return FillMarkerArray(serialized_data, serialized_size, out);
    }
    if (proto_full_name == "automsgs.msgs.nav_msgs.GridCells") {
      return FillGridCells(serialized_data, serialized_size, out);
    }
    if (proto_full_name == "automsgs.msgs.vision_msgs.BoundingBox3D") {
      return FillBoundingBox3D(serialized_data, serialized_size, out);
    }
    if (proto_full_name == "automsgs.msgs.vision_msgs.BoundingBox3DArray") {
      return FillBoundingBox3DArray(serialized_data, serialized_size, out);
    }
    if (proto_full_name == "automsgs.msgs.vision_msgs.Detection3D") {
      return FillDetection3D(serialized_data, serialized_size, out);
    }
    if (proto_full_name == "automsgs.msgs.vision_msgs.Detection3DArray") {
      return FillDetection3DArray(serialized_data, serialized_size, out);
    }
    return false;
  }

  if (strategy == S::kConvertToPoseInFrame) {
    if (proto_full_name == "automsgs.msgs.geometry_msgs.PoseStamped") {
      return FillPoseStamped(serialized_data, serialized_size, out);
    }
    if (proto_full_name == "automsgs.msgs.nav_msgs.Odometry") {
      return FillOdometry(serialized_data, serialized_size, out);
    }
    return false;
  }

  if (strategy == S::kConvertToFrameTransform &&
      proto_full_name == "automsgs.msgs.geometry_msgs.TransformStamped") {
    return FillTransformStamped(serialized_data, serialized_size, out);
  }

  if (strategy == S::kConvertToPosesInFrame &&
      proto_full_name == "automsgs.msgs.geometry_msgs.PoseArray") {
    return FillPoseArray(serialized_data, serialized_size, out);
  }

  if (strategy == S::kConvertToLaserScan &&
      proto_full_name == "automsgs.msgs.sensor_msgs.LaserScan") {
    return FillLaserScan(serialized_data, serialized_size, out);
  }

  if (strategy == S::kConvertToPointCloud) {
    if (proto_full_name == "automsgs.msgs.sensor_msgs.PointCloud2") {
      return FillPointCloud2(serialized_data, serialized_size, out);
    }
    if (proto_full_name == "automsgs.msgs.sensor_msgs.PointCloud") {
      return FillPointCloudLegacy(serialized_data, serialized_size, out);
    }
    if (proto_full_name == "automsgs.msgs.sensor_msgs.MultiEchoLaserScan") {
      return FillMultiEchoAsPointCloud(serialized_data, serialized_size, out);
    }
    return false;
  }

  if (strategy == S::kConvertToGrid &&
      proto_full_name == "automsgs.msgs.nav_msgs.OccupancyGrid") {
    return FillOccupancyGrid(serialized_data, serialized_size, out);
  }

  if (strategy == S::kConvertToLocationFix &&
      proto_full_name == "automsgs.msgs.sensor_msgs.NavSatFix") {
    return FillNavSatFix(serialized_data, serialized_size, out);
  }

  return false;
}

}  // namespace converter
}  // namespace autoviz

#else

#include "autonomy/autoviz/core/convert/message/convert_automsgs_message.hpp"
#include "autonomy/autoviz/core/convert/registry/automsgs_foxglove_registry.hpp"

#include <cstddef>
#include <string>

namespace autoviz {
namespace converter {

bool TryConvertBodyToFoxglove(AutomsgsFoxgloveStrategy, const std::string&, const void*,
                                      std::size_t, FoxgloveConvertedMessage*) {
  return false;
}

}  // namespace converter
}  // namespace autoviz

#endif
