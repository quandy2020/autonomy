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

#include "autonomy/visualization/adapters/foxglove_converters.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <zlib.h>

#include <automsgs/msgs/geometry_msgs/point_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/polygon_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_array.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/map_msgs/map_msgs.pb.h>
#include <automsgs/msgs/planning_msgs/planning_msgs.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/compressed_image.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/multi_echo_laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/nav_sat_fix.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/sensor_msgs/channel_float32.pb.h>
#include <automsgs/msgs/sensor_msgs/point_field.pb.h>
#include <automsgs/msgs/sensor_msgs/range.pb.h>
#include <automsgs/msgs/std_msgs/color_rgba.pb.h>
#include <automsgs/msgs/std_msgs/header.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>
#include <automsgs/msgs/vision_msgs/bounding_box2d.pb.h>
#include <automsgs/msgs/vision_msgs/bounding_box2d_array.pb.h>
#include <automsgs/msgs/vision_msgs/bounding_box3d.pb.h>
#include <automsgs/msgs/vision_msgs/bounding_box3d_array.pb.h>
#include <automsgs/msgs/vision_msgs/detection2d.pb.h>
#include <automsgs/msgs/vision_msgs/detection2d_array.pb.h>
#include <automsgs/msgs/vision_msgs/detection3d.pb.h>
#include <automsgs/msgs/vision_msgs/detection3d_array.pb.h>
#include <automsgs/msgs/visualization_msgs/marker.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/utils/occ_grid_values.hpp"

namespace autonomy {
namespace visualization {
namespace {

using automsgs::msgs::geometry_msgs::PointStamped;
using automsgs::msgs::geometry_msgs::PolygonStamped;
using automsgs::msgs::geometry_msgs::Pose;
using automsgs::msgs::geometry_msgs::PoseArray;
using automsgs::msgs::geometry_msgs::PoseStamped;
using automsgs::msgs::geometry_msgs::TransformStamped;
using automsgs::msgs::geometry_msgs::TransformStampeds;
using automsgs::msgs::map_msgs::Costmap;
using automsgs::msgs::map_msgs::CostmapUpdate;
using automsgs::msgs::map_msgs::GridCells;
using automsgs::msgs::map_msgs::GridMap;
using automsgs::msgs::map_msgs::OccupancyGrid;
using automsgs::msgs::map_msgs::OccupancyGridUpdate;
using automsgs::msgs::map_msgs::VoxelGrid;
using automsgs::msgs::planning_msgs::Odometry;
using automsgs::msgs::planning_msgs::Path;
using automsgs::msgs::sensor_msgs::CameraInfo;
using automsgs::msgs::sensor_msgs::CompressedImage;
using automsgs::msgs::sensor_msgs::Image;
using automsgs::msgs::sensor_msgs::LaserScan;
using automsgs::msgs::sensor_msgs::MultiEchoLaserScan;
using automsgs::msgs::sensor_msgs::NavSatFix;
using automsgs::msgs::sensor_msgs::PointCloud;
using automsgs::msgs::sensor_msgs::PointCloud2;
using automsgs::msgs::sensor_msgs::PointField;
using automsgs::msgs::sensor_msgs::ChannelFloat32;
using automsgs::msgs::sensor_msgs::Range;
using automsgs::msgs::std_msgs::ColorRGBA;
using automsgs::msgs::std_msgs::Header;
using automsgs::msgs::tf2_msgs::TFMessage;
using automsgs::msgs::vision_msgs::BoundingBox2D;
using automsgs::msgs::vision_msgs::BoundingBox2DArray;
using automsgs::msgs::vision_msgs::BoundingBox3D;
using automsgs::msgs::vision_msgs::BoundingBox3DArray;
using automsgs::msgs::vision_msgs::Detection2D;
using automsgs::msgs::vision_msgs::Detection2DArray;
using automsgs::msgs::vision_msgs::Detection3D;
using automsgs::msgs::vision_msgs::Detection3DArray;
using automsgs::msgs::visualization_msgs::Marker;
using automsgs::msgs::visualization_msgs::MarkerArray;

using FoxgloveArrowPrimitive = foxglove::messages::ArrowPrimitive;
using FoxgloveCameraCalibration = foxglove::messages::CameraCalibration;
using FoxgloveCubePrimitive = foxglove::messages::CubePrimitive;
using FoxgloveColor = foxglove::messages::Color;
using FoxgloveCompressedImage = foxglove::messages::CompressedImage;
using FoxgloveFrameTransform = foxglove::messages::FrameTransform;
using FoxgloveFrameTransforms = foxglove::messages::FrameTransforms;
using FoxgloveGrid = foxglove::messages::Grid;
using FoxgloveImageAnnotations = foxglove::messages::ImageAnnotations;
using FoxgloveLaserScan = foxglove::messages::LaserScan;
using FoxgloveLinePrimitive = foxglove::messages::LinePrimitive;
using FoxgloveLocationFix = foxglove::messages::LocationFix;
using FoxgloveOdometry = foxglove::messages::Odometry;
using FoxglovePackedElementField = foxglove::messages::PackedElementField;
using FoxglovePoint2 = foxglove::messages::Point2;
using FoxglovePoint3 = foxglove::messages::Point3;
using FoxglovePoint3InFrame = foxglove::messages::Point3InFrame;
using FoxglovePointsAnnotation = foxglove::messages::PointsAnnotation;
using FoxglovePointCloud = foxglove::messages::PointCloud;
using FoxglovePose = foxglove::messages::Pose;
using FoxglovePoseInFrame = foxglove::messages::PoseInFrame;
using FoxglovePosesInFrame = foxglove::messages::PosesInFrame;
using FoxgloveQuaternion = foxglove::messages::Quaternion;
using FoxgloveRawImage = foxglove::messages::RawImage;
using FoxgloveSceneEntity = foxglove::messages::SceneEntity;
using FoxgloveSceneEntityDeletion = foxglove::messages::SceneEntityDeletion;
using FoxgloveSceneUpdate = foxglove::messages::SceneUpdate;
using FoxgloveSpherePrimitive = foxglove::messages::SpherePrimitive;
using FoxgloveTimestamp = foxglove::messages::Timestamp;
using FoxgloveVector2 = foxglove::messages::Vector2;
using FoxgloveVector3 = foxglove::messages::Vector3;
using FoxgloveVoxelGrid = foxglove::messages::VoxelGrid;

constexpr char kLaserScanType[] =
    "automsgs.msgs.sensor_msgs.LaserScan";
constexpr char kMultiEchoLaserScanType[] =
    "automsgs.msgs.sensor_msgs.MultiEchoLaserScan";
constexpr char kMarkerType[] =
    "automsgs.msgs.visualization_msgs.Marker";
constexpr char kMarkerArrayType[] =
    "automsgs.msgs.visualization_msgs.MarkerArray";
constexpr char kTransformStampedsType[] =
    "automsgs.msgs.geometry_msgs.TransformStampeds";
constexpr char kTransformStampedType[] =
    "automsgs.msgs.geometry_msgs.TransformStamped";
constexpr char kPointCloud2Type[] =
    "automsgs.msgs.sensor_msgs.PointCloud2";
constexpr char kPointCloudType[] =
    "automsgs.msgs.sensor_msgs.PointCloud";
constexpr char kImageType[] = "automsgs.msgs.sensor_msgs.Image";
constexpr char kTfMessageType[] = "automsgs.msgs.tf2_msgs.TFMessage";
constexpr char kPoseStampedType[] =
    "automsgs.msgs.geometry_msgs.PoseStamped";
constexpr char kPoseArrayType[] =
    "automsgs.msgs.geometry_msgs.PoseArray";
constexpr char kPointStampedType[] =
    "automsgs.msgs.geometry_msgs.PointStamped";
constexpr char kPolygonStampedType[] =
    "automsgs.msgs.geometry_msgs.PolygonStamped";
constexpr char kPathType[] = "automsgs.msgs.planning_msgs.Path";
constexpr char kOdometryType[] =
    "automsgs.msgs.planning_msgs.Odometry";
constexpr char kCompressedImageType[] =
    "automsgs.msgs.sensor_msgs.CompressedImage";
constexpr char kCameraInfoType[] =
    "automsgs.msgs.sensor_msgs.CameraInfo";
constexpr char kNavSatFixType[] =
    "automsgs.msgs.sensor_msgs.NavSatFix";
constexpr char kOccupancyGridType[] =
    "automsgs.msgs.map_msgs.OccupancyGrid";
constexpr char kVoxelGridType[] = "automsgs.msgs.map_msgs.VoxelGrid";
constexpr char kCostmapType[] = "automsgs.msgs.map_msgs.Costmap";
constexpr char kCostmapUpdateType[] =
    "automsgs.msgs.map_msgs.CostmapUpdate";
constexpr char kGridMapType[] = "automsgs.msgs.map_msgs.GridMap";
constexpr char kGridCellsType[] = "automsgs.msgs.map_msgs.GridCells";
constexpr char kOccupancyGridUpdateType[] =
    "automsgs.msgs.map_msgs.OccupancyGridUpdate";
constexpr char kRangeType[] = "automsgs.msgs.sensor_msgs.Range";
constexpr char kBoundingBox2DType[] =
    "automsgs.msgs.vision_msgs.BoundingBox2D";
constexpr char kBoundingBox2DArrayType[] =
    "automsgs.msgs.vision_msgs.BoundingBox2DArray";
constexpr char kDetection2DType[] =
    "automsgs.msgs.vision_msgs.Detection2D";
constexpr char kDetection2DArrayType[] =
    "automsgs.msgs.vision_msgs.Detection2DArray";
constexpr char kBoundingBox3DType[] =
    "automsgs.msgs.vision_msgs.BoundingBox3D";
constexpr char kBoundingBox3DArrayType[] =
    "automsgs.msgs.vision_msgs.BoundingBox3DArray";
constexpr char kDetection3DType[] =
    "automsgs.msgs.vision_msgs.Detection3D";
constexpr char kDetection3DArrayType[] =
    "automsgs.msgs.vision_msgs.Detection3DArray";

constexpr char kFoxgloveLaserScanSchema[] = "foxglove.LaserScan";
constexpr char kFoxgloveSceneUpdateSchema[] = "foxglove.SceneUpdate";
constexpr char kFoxgloveFrameTransformsSchema[] = "foxglove.FrameTransforms";
constexpr char kFoxgloveFrameTransformSchema[] = "foxglove.FrameTransform";
constexpr char kFoxglovePointCloudSchema[] = "foxglove.PointCloud";
constexpr char kFoxgloveRawImageSchema[] = "foxglove.RawImage";
constexpr char kFoxglovePoseInFrameSchema[] = "foxglove.PoseInFrame";
constexpr char kFoxglovePosesInFrameSchema[] = "foxglove.PosesInFrame";
constexpr char kFoxglovePoint3InFrameSchema[] = "foxglove.Point3InFrame";
constexpr char kFoxgloveOdometrySchema[] = "foxglove.Odometry";
constexpr char kFoxgloveCompressedImageSchema[] = "foxglove.CompressedImage";
constexpr char kFoxgloveCameraCalibrationSchema[] = "foxglove.CameraCalibration";
constexpr char kFoxgloveLocationFixSchema[] = "foxglove.LocationFix";
constexpr char kFoxgloveGridSchema[] = "foxglove.Grid";
constexpr char kFoxgloveVoxelGridSchema[] = "foxglove.VoxelGrid";
constexpr char kFoxgloveImageAnnotationsSchema[] = "foxglove.ImageAnnotations";

constexpr int32_t kMarkerAdd = 0;
constexpr int32_t kMarkerDelete = 2;
constexpr int32_t kMarkerDeleteAll = 3;
constexpr int32_t kMarkerCube = 1;
constexpr int32_t kMarkerSphere = 2;
constexpr int32_t kMarkerLineStrip = 4;

constexpr float kDefaultMapResolution = 0.05f;

template <size_t N>
void CopyRepeatedToArray(const google::protobuf::RepeatedField<double>& source,
                         std::array<double, N>* target) {
  if (target == nullptr) {
    return;
  }
  target->fill(0.0);
  const size_t count =
      std::min(N, static_cast<size_t>(source.size()));
  for (size_t i = 0; i < count; ++i) {
    (*target)[i] = source[static_cast<int>(i)];
  }
}

float FirstEchoOrNan(
    const ::automsgs::msgs::sensor_msgs::LaserEcho& echo) {
  if (echo.echoes_size() == 0) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return echo.echoes(0);
}

std::string PackRepeatedUint32AsBytes(
    const google::protobuf::RepeatedField<uint32_t>& data) {
  std::string packed;
  packed.reserve(data.size() * sizeof(uint32_t));
  for (const uint32_t value : data) {
    packed.push_back(static_cast<char>(value & 0xFF));
    packed.push_back(static_cast<char>((value >> 8) & 0xFF));
    packed.push_back(static_cast<char>((value >> 16) & 0xFF));
    packed.push_back(static_cast<char>((value >> 24) & 0xFF));
  }
  return packed;
}

void AppendFloat(google::protobuf::RepeatedField<uint32_t>* data, float value) {
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(float));
  data->Add(bits);
}

void AppendFloatToBytes(std::string* data, float value) {
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(float));
  data->push_back(static_cast<char>(bits & 0xFF));
  data->push_back(static_cast<char>((bits >> 8) & 0xFF));
  data->push_back(static_cast<char>((bits >> 16) & 0xFF));
  data->push_back(static_cast<char>((bits >> 24) & 0xFF));
}

std::string NormalizeImageFormat(std::string format) {
  format.erase(std::remove(format.begin(), format.end(), ' '), format.end());
  const size_t semicolon = format.find(';');
  if (semicolon != std::string::npos) {
    format = format.substr(0, semicolon);
  }
  std::transform(format.begin(), format.end(), format.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  if (format == "jpg") {
    format = "jpeg";
  }
  return format;
}

bool IsFoxgloveCompressedFormat(const std::string& format) {
  return format == "jpeg" || format == "png" || format == "webp" ||
         format == "avif";
}

void AppendBigEndian32(std::string* out, uint32_t value) {
  out->push_back(static_cast<char>((value >> 24) & 0xFF));
  out->push_back(static_cast<char>((value >> 16) & 0xFF));
  out->push_back(static_cast<char>((value >> 8) & 0xFF));
  out->push_back(static_cast<char>(value & 0xFF));
}

void AppendPngChunk(std::string* png, const char type[4],
                    const std::string& data) {
  AppendBigEndian32(png, static_cast<uint32_t>(data.size()));
  png->append(type, 4);
  png->append(data);
  uLong crc = crc32(0, Z_NULL, 0);
  crc = crc32(crc, reinterpret_cast<const Bytef*>(type), 4);
  crc = crc32(crc, reinterpret_cast<const Bytef*>(data.data()), data.size());
  AppendBigEndian32(png, static_cast<uint32_t>(crc));
}

bool InferRawImageSize(size_t pixel_count, uint32_t* width, uint32_t* height) {
  if (width == nullptr || height == nullptr || pixel_count == 0) {
    return false;
  }
  const size_t side =
      static_cast<size_t>(std::lround(std::sqrt(static_cast<double>(pixel_count))));
  if (side > 0 && side * side == pixel_count) {
    *width = static_cast<uint32_t>(side);
    *height = static_cast<uint32_t>(side);
    return true;
  }
  constexpr uint32_t kCandidateWidths[] = {1920, 1280, 640, 320, 160, 64};
  for (const uint32_t candidate : kCandidateWidths) {
    if (pixel_count % candidate == 0) {
      *width = candidate;
      *height = static_cast<uint32_t>(pixel_count / candidate);
      return true;
    }
  }
  return false;
}

bool EncodeRawImageAsPng(const std::string& format, uint32_t width,
                         uint32_t height, const std::string& raw,
                         std::string* png) {
  if (png == nullptr || width == 0 || height == 0) {
    return false;
  }

  uint8_t color_type = 0;
  uint32_t bytes_per_pixel = 0;
  if (format == "rgb8") {
    color_type = 2;
    bytes_per_pixel = 3;
  } else if (format == "bgr8") {
    color_type = 2;
    bytes_per_pixel = 3;
  } else if (format == "rgba8") {
    color_type = 6;
    bytes_per_pixel = 4;
  } else if (format == "bgra8") {
    color_type = 6;
    bytes_per_pixel = 4;
  } else if (format == "mono8" || format == "8uc1") {
    color_type = 0;
    bytes_per_pixel = 1;
  } else {
    return false;
  }

  const size_t expected = static_cast<size_t>(width) * height * bytes_per_pixel;
  if (raw.size() != expected) {
    return false;
  }

  std::string filtered;
  filtered.reserve(static_cast<size_t>(height) * (1 + width * bytes_per_pixel));
  for (uint32_t y = 0; y < height; ++y) {
    filtered.push_back('\0');
    const size_t row_offset = static_cast<size_t>(y) * width * bytes_per_pixel;
    if (format == "bgr8") {
      for (uint32_t x = 0; x < width; ++x) {
        const size_t idx = row_offset + static_cast<size_t>(x) * 3;
        filtered.push_back(raw[idx + 2]);
        filtered.push_back(raw[idx + 1]);
        filtered.push_back(raw[idx]);
      }
    } else if (format == "bgra8") {
      for (uint32_t x = 0; x < width; ++x) {
        const size_t idx = row_offset + static_cast<size_t>(x) * 4;
        filtered.push_back(raw[idx + 2]);
        filtered.push_back(raw[idx + 1]);
        filtered.push_back(raw[idx]);
        filtered.push_back(raw[idx + 3]);
      }
    } else {
      filtered.append(raw.data() + row_offset, width * bytes_per_pixel);
    }
  }

  uLongf compressed_size = compressBound(filtered.size());
  std::string compressed(compressed_size, '\0');
  const int result = compress2(
      reinterpret_cast<Bytef*>(compressed.data()), &compressed_size,
      reinterpret_cast<const Bytef*>(filtered.data()),
      static_cast<uLong>(filtered.size()), Z_DEFAULT_COMPRESSION);
  if (result != Z_OK) {
    return false;
  }
  compressed.resize(compressed_size);

  png->clear();
  png->append("\x89PNG\r\n\x1a\n", 8);

  std::string ihdr;
  AppendBigEndian32(&ihdr, width);
  AppendBigEndian32(&ihdr, height);
  ihdr.push_back(static_cast<char>(8));
  ihdr.push_back(static_cast<char>(color_type));
  ihdr.push_back('\0');
  ihdr.push_back('\0');
  ihdr.push_back('\0');
  AppendPngChunk(png, "IHDR", ihdr);
  AppendPngChunk(png, "IDAT", compressed);
  AppendPngChunk(png, "IEND", std::string());

  return true;
}

FoxgloveTimestamp ToFoxgloveTimestamp(const Header& header) {
  FoxgloveTimestamp timestamp;
  timestamp.sec = static_cast<uint32_t>(header.stamp().sec());
  timestamp.nsec = header.stamp().nanosec();
  return timestamp;
}

bool TryConvertRawCompressedImage(const CompressedImage& source,
                                  std::string* adapted_payload) {
  if (adapted_payload == nullptr) {
    return false;
  }

  const std::string format = NormalizeImageFormat(source.format());
  uint32_t bytes_per_pixel = 0;
  if (format == "rgb8" || format == "bgr8") {
    bytes_per_pixel = 3;
  } else if (format == "rgba8" || format == "bgra8") {
    bytes_per_pixel = 4;
  } else if (format == "mono8" || format == "8uc1") {
    bytes_per_pixel = 1;
  } else {
    return false;
  }
  if (source.data().empty() || source.data().size() % bytes_per_pixel != 0) {
    return false;
  }

  const size_t pixel_count = source.data().size() / bytes_per_pixel;
  uint32_t width = 0;
  uint32_t height = 0;
  if (!InferRawImageSize(pixel_count, &width, &height)) {
    return false;
  }

  std::string png;
  if (!EncodeRawImageAsPng(format, width, height, source.data(), &png)) {
    return false;
  }

  FoxgloveCompressedImage target;
  target.timestamp = ToFoxgloveTimestamp(source.header());
  target.frame_id = source.header().frame_id();
  target.format = "png";
  target.data.assign(reinterpret_cast<const std::byte*>(png.data()),
                     reinterpret_cast<const std::byte*>(png.data() + png.size()));
  return EncodeFoxgloveMessage(&target, adapted_payload);
}

FoxgloveColor ToFoxgloveColor(const ColorRGBA& color) {
  FoxgloveColor target;
  target.r = color.r();
  target.g = color.g();
  target.b = color.b();
  target.a = color.a();
  return target;
}

FoxglovePose ToFoxglovePose(const Pose& pose) {
  FoxglovePose target;
  FoxgloveVector3 position;
  position.x = pose.position().x();
  position.y = pose.position().y();
  position.z = pose.position().z();
  target.position = position;

  FoxgloveQuaternion orientation;
  orientation.x = pose.orientation().x();
  orientation.y = pose.orientation().y();
  orientation.z = pose.orientation().z();
  orientation.w = pose.orientation().w();
  target.orientation = orientation;
  return target;
}

FoxgloveVector3 ToFoxgloveVector3(double x, double y, double z) {
  FoxgloveVector3 vector;
  vector.x = x;
  vector.y = y;
  vector.z = z;
  return vector;
}

std::string MarkerEntityId(const Marker& marker) {
  return marker.ns() + "/" + std::to_string(marker.id());
}

FoxgloveLaserScan ToFoxgloveLaserScan(const LaserScan& source) {
  FoxgloveLaserScan target;
  target.timestamp = ToFoxgloveTimestamp(source.header());
  target.frame_id = source.header().frame_id();
  target.start_angle = source.angle_min();
  target.end_angle = source.angle_max();
  target.ranges.reserve(static_cast<size_t>(source.ranges_size()));
  target.intensities.reserve(static_cast<size_t>(source.intensities_size()));
  for (const float range : source.ranges()) {
    target.ranges.push_back(range);
  }
  for (const float intensity : source.intensities()) {
    target.intensities.push_back(intensity);
  }
  return target;
}

bool AppendMarkerEntity(const Marker& marker, FoxgloveSceneUpdate* update) {
  if (update == nullptr) {
    return false;
  }

  if (marker.action() == kMarkerDeleteAll) {
    FoxgloveSceneEntityDeletion deletion;
    deletion.timestamp = ToFoxgloveTimestamp(marker.header());
    deletion.type =
        FoxgloveSceneEntityDeletion::SceneEntityDeletionType::ALL;
    update->deletions.push_back(std::move(deletion));
    return true;
  }

  if (marker.action() == kMarkerDelete) {
    FoxgloveSceneEntityDeletion deletion;
    deletion.timestamp = ToFoxgloveTimestamp(marker.header());
    deletion.type =
        FoxgloveSceneEntityDeletion::SceneEntityDeletionType::MATCHING_ID;
    deletion.id = MarkerEntityId(marker);
    update->deletions.push_back(std::move(deletion));
    return true;
  }

  FoxgloveSceneEntity entity;
  entity.timestamp = ToFoxgloveTimestamp(marker.header());
  entity.frame_id = marker.header().frame_id();
  entity.id = MarkerEntityId(marker);
  entity.frame_locked = true;

  if (marker.type() == kMarkerSphere) {
    FoxgloveSpherePrimitive sphere;
    sphere.pose = ToFoxglovePose(marker.pose());
    sphere.size = ToFoxgloveVector3(marker.scale().x(), marker.scale().y(),
                                    marker.scale().z());
    sphere.color = ToFoxgloveColor(marker.color());
    entity.spheres.push_back(std::move(sphere));
  } else if (marker.type() == kMarkerCube) {
    FoxgloveCubePrimitive cube;
    cube.pose = ToFoxglovePose(marker.pose());
    cube.size = ToFoxgloveVector3(marker.scale().x(), marker.scale().y(),
                                  marker.scale().z());
    cube.color = ToFoxgloveColor(marker.color());
    entity.cubes.push_back(std::move(cube));
  } else if (marker.type() == kMarkerLineStrip) {
    FoxgloveLinePrimitive line;
    line.type = FoxgloveLinePrimitive::LineType::LINE_STRIP;
    line.thickness = marker.scale().x();
    line.color = ToFoxgloveColor(marker.color());
    line.points.reserve(static_cast<size_t>(marker.points_size()));
    for (const auto& point : marker.points()) {
      FoxglovePoint3 foxglove_point;
      foxglove_point.x = point.x();
      foxglove_point.y = point.y();
      foxglove_point.z = point.z();
      line.points.push_back(foxglove_point);
    }
    entity.lines.push_back(std::move(line));
  } else {
    return false;
  }

  update->entities.push_back(std::move(entity));
  return true;
}

FoxgloveFrameTransform ToFoxgloveTransform(const TransformStamped& source) {
  FoxgloveFrameTransform target;
  target.timestamp = ToFoxgloveTimestamp(source.header());
  target.parent_frame_id = source.header().frame_id();
  target.child_frame_id = source.child_frame_id();
  target.translation = ToFoxgloveVector3(source.transform().translation().x(),
                                         source.transform().translation().y(),
                                         source.transform().translation().z());

  FoxgloveQuaternion rotation;
  rotation.x = source.transform().rotation().x();
  rotation.y = source.transform().rotation().y();
  rotation.z = source.transform().rotation().z();
  rotation.w = source.transform().rotation().w();
  target.rotation = rotation;
  return target;
}

FoxglovePackedElementField::NumericType ToFoxgloveNumericType(
    uint32_t datatype) {
  switch (datatype) {
    case 1:
      return FoxglovePackedElementField::NumericType::INT8;
    case 2:
      return FoxglovePackedElementField::NumericType::UINT8;
    case 3:
      return FoxglovePackedElementField::NumericType::INT16;
    case 4:
      return FoxglovePackedElementField::NumericType::UINT16;
    case 5:
      return FoxglovePackedElementField::NumericType::INT32;
    case 6:
      return FoxglovePackedElementField::NumericType::UINT32;
    case 7:
      return FoxglovePackedElementField::NumericType::FLOAT32;
    case 8:
      return FoxglovePackedElementField::NumericType::FLOAT64;
    default:
      return FoxglovePackedElementField::NumericType::UNKNOWN;
  }
}

FoxglovePointCloud ToFoxglovePointCloud(const PointCloud2& source) {
  FoxglovePointCloud target;
  target.timestamp = ToFoxgloveTimestamp(source.header());
  target.frame_id = source.header().frame_id();
  target.point_stride = source.point_step();
  target.fields.reserve(static_cast<size_t>(source.fields_size()));
  for (const auto& field : source.fields()) {
    FoxglovePackedElementField target_field;
    target_field.name = field.name();
    target_field.offset = field.offset();
    target_field.type = ToFoxgloveNumericType(field.datatype());
    target.fields.push_back(std::move(target_field));
  }

  target.data.assign(
      reinterpret_cast<const std::byte*>(source.data().data()),
      reinterpret_cast<const std::byte*>(source.data().data() +
                                         source.data().size()));
  return target;
}

FoxgloveRawImage ToFoxgloveRawImage(const Image& source) {
  FoxgloveRawImage target;
  target.timestamp = ToFoxgloveTimestamp(source.header());
  target.frame_id = source.header().frame_id();
  target.width = source.width();
  target.height = source.height();
  target.encoding = source.encoding();
  target.step = source.step();
  target.data.assign(reinterpret_cast<const std::byte*>(source.data().data()),
                     reinterpret_cast<const std::byte*>(source.data().data() +
                                                          source.data().size()));
  return target;
}

FoxgloveSceneUpdate ToFoxglovePolygon(const PolygonStamped& source) {
  FoxgloveSceneUpdate update;
  FoxgloveSceneEntity entity;
  entity.timestamp = ToFoxgloveTimestamp(source.header());
  entity.frame_id = source.header().frame_id();
  entity.id = "polygon";
  entity.frame_locked = true;

  FoxgloveLinePrimitive line;
  line.type = FoxgloveLinePrimitive::LineType::LINE_LOOP;
  line.thickness = 0.05;
  FoxgloveColor color;
  color.r = 0.0;
  color.g = 1.0;
  color.b = 0.0;
  color.a = 1.0;
  line.color = color;
  line.points.reserve(static_cast<size_t>(source.polygon().points_size()));
  for (const auto& point : source.polygon().points()) {
    FoxglovePoint3 foxglove_point;
    foxglove_point.x = point.x();
    foxglove_point.y = point.y();
    foxglove_point.z = point.z();
    line.points.push_back(foxglove_point);
  }
  entity.lines.push_back(std::move(line));
  update.entities.push_back(std::move(entity));
  return update;
}

void OccupancyValueToRgba(int32_t value, uint8_t* red, uint8_t* green,
                          uint8_t* blue, uint8_t* alpha) {
  const map::costmap_2d::utils::Rgba8 rgba =
      map::costmap_2d::utils::OccupancyCellToRgba(
          static_cast<int8_t>(value));
  *red = rgba.r;
  *green = rgba.g;
  *blue = rgba.b;
  *alpha = rgba.a;
}

FoxgloveGrid ToFoxgloveGrid(const OccupancyGrid& source) {
  FoxgloveGrid target;
  target.timestamp = ToFoxgloveTimestamp(source.header());
  target.frame_id = source.header().frame_id();
  target.pose = ToFoxglovePose(source.info().origin());

  FoxgloveVector2 cell_size;
  cell_size.x = source.info().resolution();
  cell_size.y = source.info().resolution();
  target.cell_size = cell_size;
  target.column_count = source.info().width();
  target.cell_stride = 4;
  target.row_stride = source.info().width() * target.cell_stride;

  FoxglovePackedElementField red_field;
  red_field.name = "red";
  red_field.offset = 0;
  red_field.type = FoxglovePackedElementField::NumericType::UINT8;
  target.fields.push_back(red_field);

  FoxglovePackedElementField green_field;
  green_field.name = "green";
  green_field.offset = 1;
  green_field.type = FoxglovePackedElementField::NumericType::UINT8;
  target.fields.push_back(green_field);

  FoxglovePackedElementField blue_field;
  blue_field.name = "blue";
  blue_field.offset = 2;
  blue_field.type = FoxglovePackedElementField::NumericType::UINT8;
  target.fields.push_back(blue_field);

  FoxglovePackedElementField alpha_field;
  alpha_field.name = "alpha";
  alpha_field.offset = 3;
  alpha_field.type = FoxglovePackedElementField::NumericType::UINT8;
  target.fields.push_back(alpha_field);

  target.data.reserve(static_cast<size_t>(source.data_size()) * 4);
  for (const int32_t value : source.data()) {
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
    uint8_t alpha = 0;
    OccupancyValueToRgba(value, &red, &green, &blue, &alpha);
    target.data.push_back(static_cast<std::byte>(red));
    target.data.push_back(static_cast<std::byte>(green));
    target.data.push_back(static_cast<std::byte>(blue));
    target.data.push_back(static_cast<std::byte>(alpha));
  }
  return target;
}

FoxgloveVoxelGrid ToFoxgloveVoxelGrid(const VoxelGrid& source) {
  FoxgloveVoxelGrid target;
  target.timestamp = ToFoxgloveTimestamp(source.header());
  target.frame_id = source.header().frame_id();

  FoxglovePose pose;
  FoxgloveVector3 position;
  position.x = source.origin().x();
  position.y = source.origin().y();
  position.z = source.origin().z();
  pose.position = position;
  FoxgloveQuaternion orientation;
  orientation.w = 1.0;
  pose.orientation = orientation;
  target.pose = pose;

  target.row_count = source.size_y();
  target.column_count = source.size_x();
  target.cell_size =
      ToFoxgloveVector3(source.resolutions().x(), source.resolutions().y(),
                        source.resolutions().z());
  target.cell_stride = 1;
  target.row_stride = source.size_x();
  target.slice_stride = source.size_x() * source.size_y();

  FoxglovePackedElementField field;
  field.name = "occupancy";
  field.offset = 0;
  field.type = FoxglovePackedElementField::NumericType::UINT8;
  target.fields.push_back(std::move(field));

  const uint32_t size_x = source.size_x();
  const uint32_t size_y = source.size_y();
  const uint32_t size_z = source.size_z();
  target.data.resize(static_cast<size_t>(size_x) * size_y * size_z,
                     static_cast<std::byte>(0));

  for (uint32_t z = 0; z < size_z; ++z) {
    for (uint32_t y = 0; y < size_y; ++y) {
      for (uint32_t x = 0; x < size_x; ++x) {
        const size_t column_index = static_cast<size_t>(x) + y * size_x;
        uint32_t column = 0;
        if (column_index < static_cast<size_t>(source.data_size())) {
          column = source.data(static_cast<int>(column_index));
        }
        const bool occupied = (column >> z) & 1U;
        const size_t index =
            static_cast<size_t>(z) * target.slice_stride + y * size_x + x;
        target.data[index] =
            static_cast<std::byte>(occupied ? 100 : 0);
      }
    }
  }
  return target;
}

void CostValueToRgba(uint8_t cost, uint8_t* red, uint8_t* green,
                     uint8_t* blue, uint8_t* alpha) {
  const map::costmap_2d::CostRgba8 rgba =
      map::costmap_2d::CostCellToRgba(cost);
  *red = rgba.r;
  *green = rgba.g;
  *blue = rgba.b;
  *alpha = rgba.a;
}

FoxgloveGrid ToFoxgloveCostGrid(const Header& header, const Pose& origin,
                                float resolution, uint32_t column_count,
                                uint32_t row_count, const std::string& data) {
  FoxgloveGrid target;
  target.timestamp = ToFoxgloveTimestamp(header);
  target.frame_id = header.frame_id();
  target.pose = ToFoxglovePose(origin);

  FoxgloveVector2 cell_size;
  cell_size.x = resolution;
  cell_size.y = resolution;
  target.cell_size = cell_size;
  target.column_count = column_count;
  target.cell_stride = 4;
  target.row_stride = column_count * target.cell_stride;

  FoxglovePackedElementField red_field;
  red_field.name = "red";
  red_field.offset = 0;
  red_field.type = FoxglovePackedElementField::NumericType::UINT8;
  target.fields.push_back(red_field);

  FoxglovePackedElementField green_field;
  green_field.name = "green";
  green_field.offset = 1;
  green_field.type = FoxglovePackedElementField::NumericType::UINT8;
  target.fields.push_back(green_field);

  FoxglovePackedElementField blue_field;
  blue_field.name = "blue";
  blue_field.offset = 2;
  blue_field.type = FoxglovePackedElementField::NumericType::UINT8;
  target.fields.push_back(blue_field);

  FoxglovePackedElementField alpha_field;
  alpha_field.name = "alpha";
  alpha_field.offset = 3;
  alpha_field.type = FoxglovePackedElementField::NumericType::UINT8;
  target.fields.push_back(alpha_field);

  const size_t expected =
      static_cast<size_t>(column_count) * row_count;
  target.data.reserve(expected * 4);
  for (size_t i = 0; i < expected; ++i) {
    const uint8_t cost =
        i < data.size() ? static_cast<uint8_t>(data[i]) : 0;
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
    uint8_t alpha = 0;
    CostValueToRgba(cost, &red, &green, &blue, &alpha);
    target.data.push_back(static_cast<std::byte>(red));
    target.data.push_back(static_cast<std::byte>(green));
    target.data.push_back(static_cast<std::byte>(blue));
    target.data.push_back(static_cast<std::byte>(alpha));
  }
  return target;
}

FoxgloveGrid ToFoxgloveCostmap(const Costmap& source) {
  const auto& metadata = source.metadata();
  return ToFoxgloveCostGrid(source.header(), metadata.origin(),
                            metadata.resolution(), metadata.size_x(),
                            metadata.size_y(), source.data());
}

FoxgloveGrid ToFoxgloveCostmapUpdate(const CostmapUpdate& source) {
  Pose origin;
  origin.mutable_position()->set_x(
      static_cast<double>(source.x()) * kDefaultMapResolution);
  origin.mutable_position()->set_y(
      static_cast<double>(source.y()) * kDefaultMapResolution);
  origin.mutable_position()->set_z(0.0);
  origin.mutable_orientation()->set_w(1.0);
  return ToFoxgloveCostGrid(source.header(), origin, kDefaultMapResolution,
                            source.size_x(), source.size_y(), source.data());
}

FoxgloveGrid ToFoxgloveOccupancyGridUpdate(
    const OccupancyGridUpdate& source) {
  Pose origin;
  origin.mutable_position()->set_x(
      static_cast<double>(source.x()) * kDefaultMapResolution);
  origin.mutable_position()->set_y(
      static_cast<double>(source.y()) * kDefaultMapResolution);
  origin.mutable_position()->set_z(0.0);
  origin.mutable_orientation()->set_w(1.0);

  FoxgloveGrid target;
  target.timestamp = ToFoxgloveTimestamp(source.header());
  target.frame_id = source.header().frame_id();
  target.pose = ToFoxglovePose(origin);

  FoxgloveVector2 cell_size;
  cell_size.x = kDefaultMapResolution;
  cell_size.y = kDefaultMapResolution;
  target.cell_size = cell_size;
  target.column_count = source.width();
  target.cell_stride = 4;
  target.row_stride = source.width() * target.cell_stride;

  FoxglovePackedElementField red_field;
  red_field.name = "red";
  red_field.offset = 0;
  red_field.type = FoxglovePackedElementField::NumericType::UINT8;
  target.fields.push_back(red_field);

  FoxglovePackedElementField green_field;
  green_field.name = "green";
  green_field.offset = 1;
  green_field.type = FoxglovePackedElementField::NumericType::UINT8;
  target.fields.push_back(green_field);

  FoxglovePackedElementField blue_field;
  blue_field.name = "blue";
  blue_field.offset = 2;
  blue_field.type = FoxglovePackedElementField::NumericType::UINT8;
  target.fields.push_back(blue_field);

  FoxglovePackedElementField alpha_field;
  alpha_field.name = "alpha";
  alpha_field.offset = 3;
  alpha_field.type = FoxglovePackedElementField::NumericType::UINT8;
  target.fields.push_back(alpha_field);

  target.data.reserve(static_cast<size_t>(source.data_size()) * 4);
  for (const int32_t value : source.data()) {
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
    uint8_t alpha = 0;
    OccupancyValueToRgba(value, &red, &green, &blue, &alpha);
    target.data.push_back(static_cast<std::byte>(red));
    target.data.push_back(static_cast<std::byte>(green));
    target.data.push_back(static_cast<std::byte>(blue));
    target.data.push_back(static_cast<std::byte>(alpha));
  }
  return target;
}

FoxgloveGrid ToFoxgloveGridMap(const GridMap& source) {
  FoxgloveGrid target;
  const auto& info = source.info();
  target.timestamp = ToFoxgloveTimestamp(info.header());
  target.frame_id = info.header().frame_id();

  const float resolution = info.resolution();
  const uint32_t column_count =
      static_cast<uint32_t>(std::lround(info.length_x() / resolution));
  const uint32_t row_count =
      static_cast<uint32_t>(std::lround(info.length_y() / resolution));

  Pose corner_pose = info.pose();
  corner_pose.mutable_position()->set_x(
      info.pose().position().x() - info.length_x() * 0.5);
  corner_pose.mutable_position()->set_y(
      info.pose().position().y() - info.length_y() * 0.5);
  target.pose = ToFoxglovePose(corner_pose);

  FoxgloveVector2 cell_size;
  cell_size.x = resolution;
  cell_size.y = resolution;
  target.cell_size = cell_size;
  target.column_count = column_count;
  target.cell_stride = sizeof(float);
  target.row_stride = column_count * target.cell_stride;

  FoxglovePackedElementField elevation_field;
  elevation_field.name = "elevation";
  elevation_field.offset = 0;
  elevation_field.type = FoxglovePackedElementField::NumericType::FLOAT32;
  target.fields.push_back(elevation_field);

  const size_t cell_count =
      static_cast<size_t>(column_count) * row_count;
  target.data.resize(cell_count * target.cell_stride, static_cast<std::byte>(0));

  if (!source.data().empty()) {
    const auto& layer = source.data(0);
    for (size_t i = 0; i < cell_count && i < static_cast<size_t>(layer.data_size());
         ++i) {
      const float value = layer.data(static_cast<int>(i));
      std::memcpy(target.data.data() + i * target.cell_stride, &value,
                  sizeof(float));
    }
  }
  return target;
}

FoxgloveColor DefaultDetectionColor() {
  FoxgloveColor color;
  color.r = 0.1;
  color.g = 0.9;
  color.b = 0.2;
  color.a = 0.6;
  return color;
}

FoxgloveColor DefaultGridCellColor() {
  FoxgloveColor color;
  color.r = 1.0;
  color.g = 0.5;
  color.b = 0.0;
  color.a = 0.8;
  return color;
}

void AppendCubeEntity(FoxgloveSceneUpdate* update,
                      const FoxgloveTimestamp& timestamp,
                      const std::string& frame_id, const std::string& id,
                      const Pose& center_pose, double size_x, double size_y,
                      double size_z, const FoxgloveColor& color) {
  if (update == nullptr) {
    return;
  }
  FoxgloveSceneEntity entity;
  entity.timestamp = timestamp;
  entity.frame_id = frame_id;
  entity.id = id;
  entity.frame_locked = true;

  FoxgloveCubePrimitive cube;
  cube.pose = ToFoxglovePose(center_pose);
  cube.size = ToFoxgloveVector3(size_x, size_y, size_z);
  cube.color = color;
  entity.cubes.push_back(std::move(cube));
  update->entities.push_back(std::move(entity));
}

void AppendBoundingBox3D(FoxgloveSceneUpdate* update,
                         const FoxgloveTimestamp& timestamp,
                         const std::string& frame_id, const std::string& id,
                         const BoundingBox3D& box,
                         const FoxgloveColor& color) {
  AppendCubeEntity(update, timestamp, frame_id, id, box.center(), box.size().x(),
                   box.size().y(), box.size().z(), color);
}

FoxgloveSceneUpdate ToFoxgloveGridCells(const GridCells& source) {
  FoxgloveSceneUpdate update;
  const FoxgloveTimestamp timestamp = ToFoxgloveTimestamp(source.header());
  const FoxgloveColor color = DefaultGridCellColor();
  const double depth = 0.05;
  for (int i = 0; i < source.cells_size(); ++i) {
    Pose center;
    center.mutable_position()->set_x(source.cells(i).x());
    center.mutable_position()->set_y(source.cells(i).y());
    center.mutable_position()->set_z(source.cells(i).z() + depth * 0.5);
    center.mutable_orientation()->set_w(1.0);
    AppendCubeEntity(&update, timestamp, source.header().frame_id(),
                     "grid_cell/" + std::to_string(i), center,
                     source.cell_width(), source.cell_height(), depth, color);
  }
  return update;
}

FoxgloveSceneUpdate ToFoxgloveRange(const Range& source) {
  FoxgloveSceneUpdate update;
  FoxgloveSceneEntity entity;
  entity.timestamp = ToFoxgloveTimestamp(source.header());
  entity.frame_id = source.header().frame_id();
  entity.id = "range";
  entity.frame_locked = true;

  float range = source.range();
  if (!std::isfinite(range) || range < source.min_range()) {
    range = source.max_range();
  }
  range = std::clamp(range, source.min_range(), source.max_range());

  FoxgloveArrowPrimitive arrow;
  FoxglovePose pose;
  FoxgloveQuaternion orientation;
  orientation.w = 1.0;
  pose.orientation = orientation;
  FoxgloveVector3 position;
  position.x = range * 0.5;
  position.y = 0.0;
  position.z = 0.0;
  pose.position = position;
  arrow.pose = pose;
  arrow.shaft_length = range;
  arrow.shaft_diameter = 0.04;
  arrow.head_length = std::min(0.15f, range * 0.2f);
  arrow.head_diameter = 0.08;
  FoxgloveColor color;
  color.r = 0.2;
  color.g = 0.6;
  color.b = 1.0;
  color.a = 0.9;
  arrow.color = color;
  entity.arrows.push_back(std::move(arrow));
  update.entities.push_back(std::move(entity));
  return update;
}

FoxglovePoint2 TransformBoxCorner(double cx, double cy, double theta,
                                    double lx, double ly) {
  const double cos_t = std::cos(theta);
  const double sin_t = std::sin(theta);
  FoxglovePoint2 point;
  point.x = cx + lx * cos_t - ly * sin_t;
  point.y = cy + lx * sin_t + ly * cos_t;
  return point;
}

void AppendBoundingBox2DAnnotation(const BoundingBox2D& box,
                                   FoxgloveImageAnnotations* annotations,
                                   const FoxgloveColor& color) {
  if (annotations == nullptr) {
    return;
  }
  const double cx = box.center().position().x();
  const double cy = box.center().position().y();
  const double theta = box.center().theta();
  const double hx = box.size_x() * 0.5;
  const double hy = box.size_y() * 0.5;

  FoxglovePointsAnnotation annotation;
  annotation.type = FoxglovePointsAnnotation::PointsAnnotationType::LINE_LOOP;
  annotation.outline_color = color;
  annotation.points.push_back(TransformBoxCorner(cx, cy, theta, -hx, -hy));
  annotation.points.push_back(TransformBoxCorner(cx, cy, theta, hx, -hy));
  annotation.points.push_back(TransformBoxCorner(cx, cy, theta, hx, hy));
  annotation.points.push_back(TransformBoxCorner(cx, cy, theta, -hx, hy));
  annotations->points.push_back(std::move(annotation));
}

FoxgloveImageAnnotations ToFoxgloveImageAnnotations(
    const BoundingBox2D& box) {
  FoxgloveImageAnnotations annotations;
  AppendBoundingBox2DAnnotation(box, &annotations, DefaultDetectionColor());
  return annotations;
}

FoxgloveImageAnnotations ToFoxgloveImageAnnotations(
    const BoundingBox2DArray& source) {
  FoxgloveImageAnnotations annotations;
  annotations.timestamp = ToFoxgloveTimestamp(source.header());
  for (int i = 0; i < source.boxes_size(); ++i) {
    FoxgloveColor color = DefaultDetectionColor();
    color.g = 0.3 + 0.4 * (static_cast<double>(i) /
                           std::max(1, source.boxes_size() - 1));
    AppendBoundingBox2DAnnotation(source.boxes(i), &annotations, color);
  }
  return annotations;
}

FoxgloveImageAnnotations ToFoxgloveImageAnnotations(
    const Detection2D& source) {
  FoxgloveImageAnnotations annotations;
  annotations.timestamp = ToFoxgloveTimestamp(source.header());
  AppendBoundingBox2DAnnotation(source.bbox(), &annotations,
                                DefaultDetectionColor());
  return annotations;
}

FoxgloveImageAnnotations ToFoxgloveImageAnnotations(
    const Detection2DArray& source) {
  FoxgloveImageAnnotations annotations;
  annotations.timestamp = ToFoxgloveTimestamp(source.header());
  for (int i = 0; i < source.detections_size(); ++i) {
    AppendBoundingBox2DAnnotation(source.detections(i).bbox(), &annotations,
                                  DefaultDetectionColor());
  }
  return annotations;
}

FoxgloveSceneUpdate ToFoxgloveSceneUpdate(const BoundingBox3D& box,
                                          const std::string& frame_id,
                                          const FoxgloveTimestamp& timestamp,
                                          const std::string& id) {
  FoxgloveSceneUpdate update;
  AppendBoundingBox3D(&update, timestamp, frame_id, id, box,
                      DefaultDetectionColor());
  return update;
}

FoxgloveSceneUpdate ToFoxgloveSceneUpdate(const BoundingBox3DArray& source) {
  FoxgloveSceneUpdate update;
  const FoxgloveTimestamp timestamp = ToFoxgloveTimestamp(source.header());
  for (int i = 0; i < source.boxes_size(); ++i) {
    AppendBoundingBox3D(&update, timestamp, source.header().frame_id(),
                        "bbox3d/" + std::to_string(i), source.boxes(i),
                        DefaultDetectionColor());
  }
  return update;
}

FoxgloveSceneUpdate ToFoxgloveSceneUpdate(const Detection3D& source) {
  FoxgloveSceneUpdate update;
  const FoxgloveTimestamp timestamp = ToFoxgloveTimestamp(source.header());
  AppendBoundingBox3D(&update, timestamp, source.header().frame_id(),
                      source.id().empty() ? "detection3d" : source.id(),
                      source.bbox(), DefaultDetectionColor());
  return update;
}

FoxgloveSceneUpdate ToFoxgloveSceneUpdate(const Detection3DArray& source) {
  FoxgloveSceneUpdate update;
  const FoxgloveTimestamp timestamp = ToFoxgloveTimestamp(source.header());
  for (int i = 0; i < source.detections_size(); ++i) {
    const auto& detection = source.detections(i);
    const std::string id =
        detection.id().empty()
            ? "detection3d/" + std::to_string(i)
            : detection.id();
    AppendBoundingBox3D(&update, timestamp, source.header().frame_id(), id,
                        detection.bbox(), DefaultDetectionColor());
  }
  return update;
}

FoxgloveLocationFix::PositionCovarianceType ToFoxgloveCovarianceType(
    uint32_t covariance_type) {
  switch (covariance_type) {
    case 1:
      return FoxgloveLocationFix::PositionCovarianceType::APPROXIMATED;
    case 2:
      return FoxgloveLocationFix::PositionCovarianceType::DIAGONAL_KNOWN;
    case 3:
      return FoxgloveLocationFix::PositionCovarianceType::KNOWN;
    default:
      return FoxgloveLocationFix::PositionCovarianceType::UNKNOWN;
  }
}

}  // namespace

bool AssignFoxgloveSchema(const foxglove::Schema& schema,
                          std::string* descriptor_set) {
  if (descriptor_set == nullptr || schema.data == nullptr ||
      schema.data_len == 0) {
    return false;
  }
  descriptor_set->assign(reinterpret_cast<const char*>(schema.data),
                         schema.data_len);
  return true;
}

bool PrepareFoxgloveChannelForType(const std::string& source_message_type,
                                   ChannelSnapshot* channel) {
  if (channel == nullptr) {
    return false;
  }

#define PREPARE_FOXGLOVE(Type, SchemaName)                          \
  if (source_message_type == Type) {                                  \
    return PrepareFoxgloveChannel<Foxglove##SchemaName>(channel,    \
                                                        kFoxglove##SchemaName##Schema); \
  }

  PREPARE_FOXGLOVE(kLaserScanType, LaserScan)
  PREPARE_FOXGLOVE(kMultiEchoLaserScanType, LaserScan)
  PREPARE_FOXGLOVE(kMarkerType, SceneUpdate)
  PREPARE_FOXGLOVE(kMarkerArrayType, SceneUpdate)
  PREPARE_FOXGLOVE(kTransformStampedsType, FrameTransforms)
  PREPARE_FOXGLOVE(kTfMessageType, FrameTransforms)
  PREPARE_FOXGLOVE(kTransformStampedType, FrameTransform)
  PREPARE_FOXGLOVE(kPointCloud2Type, PointCloud)
  PREPARE_FOXGLOVE(kPointCloudType, PointCloud)
  PREPARE_FOXGLOVE(kImageType, RawImage)
  PREPARE_FOXGLOVE(kPoseStampedType, PoseInFrame)
  PREPARE_FOXGLOVE(kPoseArrayType, PosesInFrame)
  PREPARE_FOXGLOVE(kPathType, PosesInFrame)
  PREPARE_FOXGLOVE(kPointStampedType, Point3InFrame)
  PREPARE_FOXGLOVE(kPolygonStampedType, SceneUpdate)
  PREPARE_FOXGLOVE(kOdometryType, Odometry)
  PREPARE_FOXGLOVE(kCompressedImageType, CompressedImage)
  PREPARE_FOXGLOVE(kCameraInfoType, CameraCalibration)
  PREPARE_FOXGLOVE(kNavSatFixType, LocationFix)
  PREPARE_FOXGLOVE(kOccupancyGridType, Grid)
  PREPARE_FOXGLOVE(kOccupancyGridUpdateType, Grid)
  PREPARE_FOXGLOVE(kCostmapType, Grid)
  PREPARE_FOXGLOVE(kCostmapUpdateType, Grid)
  PREPARE_FOXGLOVE(kGridMapType, Grid)
  PREPARE_FOXGLOVE(kGridCellsType, SceneUpdate)
  PREPARE_FOXGLOVE(kVoxelGridType, VoxelGrid)
  PREPARE_FOXGLOVE(kRangeType, SceneUpdate)
  PREPARE_FOXGLOVE(kBoundingBox2DType, ImageAnnotations)
  PREPARE_FOXGLOVE(kBoundingBox2DArrayType, ImageAnnotations)
  PREPARE_FOXGLOVE(kDetection2DType, ImageAnnotations)
  PREPARE_FOXGLOVE(kDetection2DArrayType, ImageAnnotations)
  PREPARE_FOXGLOVE(kBoundingBox3DType, SceneUpdate)
  PREPARE_FOXGLOVE(kBoundingBox3DArrayType, SceneUpdate)
  PREPARE_FOXGLOVE(kDetection3DType, SceneUpdate)
  PREPARE_FOXGLOVE(kDetection3DArrayType, SceneUpdate)

#undef PREPARE_FOXGLOVE

  return false;
}

bool ConvertToFoxglovePayload(const std::string& source_message_type,
                              const std::string& payload,
                              std::string* adapted_payload) {
  if (adapted_payload == nullptr) {
    return false;
  }

  if (source_message_type == kLaserScanType) {
    LaserScan source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveLaserScan target = ToFoxgloveLaserScan(source);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kMultiEchoLaserScanType) {
    MultiEchoLaserScan source;
    if (!source.ParseFromString(payload)) {
      return false;
    }

    LaserScan intermediate;
    *intermediate.mutable_header() = source.header();
    intermediate.set_angle_min(source.angle_min());
    intermediate.set_angle_max(source.angle_max());
    intermediate.set_angle_increment(source.angle_increment());
    intermediate.set_time_increment(source.time_increment());
    intermediate.set_scan_time(source.scan_time());
    intermediate.set_range_min(source.range_min());
    intermediate.set_range_max(source.range_max());
    for (const auto& range : source.ranges()) {
      intermediate.add_ranges(FirstEchoOrNan(range));
    }
    for (const auto& intensity : source.intensities()) {
      intermediate.add_intensities(FirstEchoOrNan(intensity));
    }

    FoxgloveLaserScan target = ToFoxgloveLaserScan(intermediate);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kMarkerType) {
    Marker source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveSceneUpdate update;
    if (!AppendMarkerEntity(source, &update)) {
      return false;
    }
    return EncodeFoxgloveMessage(&update, adapted_payload);
  }

  if (source_message_type == kMarkerArrayType) {
    MarkerArray source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveSceneUpdate update;
    for (const auto& marker : source.markers()) {
      if (!AppendMarkerEntity(marker, &update)) {
        return false;
      }
    }
    return EncodeFoxgloveMessage(&update, adapted_payload);
  }

  if (source_message_type == kTransformStampedsType ||
      source_message_type == kTfMessageType) {
    FoxgloveFrameTransforms target;
    if (source_message_type == kTransformStampedsType) {
      TransformStampeds source;
      if (!source.ParseFromString(payload)) {
        return false;
      }
      target.transforms.reserve(
          static_cast<size_t>(source.transforms_size()));
      for (const auto& transform : source.transforms()) {
        target.transforms.push_back(ToFoxgloveTransform(transform));
      }
    } else {
      TFMessage source;
      if (!source.ParseFromString(payload)) {
        return false;
      }
      target.transforms.reserve(
          static_cast<size_t>(source.transforms_size()));
      for (const auto& transform : source.transforms()) {
        target.transforms.push_back(ToFoxgloveTransform(transform));
      }
    }
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kTransformStampedType) {
    TransformStamped source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveFrameTransform target = ToFoxgloveTransform(source);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kPointCloud2Type) {
    PointCloud2 source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxglovePointCloud target = ToFoxglovePointCloud(source);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kPointCloudType) {
    PointCloud source;
    if (!source.ParseFromString(payload)) {
      return false;
    }

    PointCloud2 intermediate;
    *intermediate.mutable_header() = source.header();
    intermediate.set_height(1);
    intermediate.set_width(static_cast<uint32_t>(source.points_size()));
    intermediate.set_is_bigendian(false);
    intermediate.set_is_dense(true);

    auto add_field = [&intermediate](const std::string& name, uint32_t offset) {
      auto* field = intermediate.add_fields();
      field->set_name(name);
      field->set_offset(offset);
      field->set_datatype(
          ::automsgs::msgs::sensor_msgs::PointField::FLOAT32);
      field->set_count(1);
    };
    add_field("x", 0);
    add_field("y", 4);
    add_field("z", 8);
    add_field("intensity", 12);
    intermediate.set_point_step(16);
    intermediate.set_row_step(intermediate.point_step() * intermediate.width());

    const ChannelFloat32* intensity_channel = nullptr;
    for (const auto& channel : source.channels()) {
      if (channel.name() == "intensity") {
        intensity_channel = &channel;
        break;
      }
    }

    for (int i = 0; i < source.points_size(); ++i) {
      const auto& point = source.points(i);
      AppendFloatToBytes(intermediate.mutable_data(),
                         static_cast<float>(point.x()));
      AppendFloatToBytes(intermediate.mutable_data(),
                         static_cast<float>(point.y()));
      AppendFloatToBytes(intermediate.mutable_data(),
                         static_cast<float>(point.z()));
      float intensity = 0.0f;
      if (intensity_channel != nullptr && i < intensity_channel->values_size()) {
        intensity = intensity_channel->values(i);
      }
      AppendFloatToBytes(intermediate.mutable_data(), intensity);
    }

    FoxglovePointCloud target = ToFoxglovePointCloud(intermediate);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kImageType) {
    Image source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveRawImage target = ToFoxgloveRawImage(source);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kPoseStampedType) {
    PoseStamped source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxglovePoseInFrame target;
    target.timestamp = ToFoxgloveTimestamp(source.header());
    target.frame_id = source.header().frame_id();
    target.pose = ToFoxglovePose(source.pose());
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kPoseArrayType) {
    PoseArray source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxglovePosesInFrame target;
    target.timestamp = ToFoxgloveTimestamp(source.header());
    target.frame_id = source.header().frame_id();
    target.poses.reserve(static_cast<size_t>(source.poses_size()));
    for (const auto& pose : source.poses()) {
      target.poses.push_back(ToFoxglovePose(pose));
    }
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kPathType) {
    Path source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxglovePosesInFrame target;
    target.timestamp = ToFoxgloveTimestamp(source.header());
    target.frame_id = source.header().frame_id();
    target.poses.reserve(static_cast<size_t>(source.poses_size()));
    for (const auto& pose_stamped : source.poses()) {
      target.poses.push_back(ToFoxglovePose(pose_stamped.pose()));
    }
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kPointStampedType) {
    PointStamped source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxglovePoint3InFrame target;
    target.timestamp = ToFoxgloveTimestamp(source.header());
    target.frame_id = source.header().frame_id();
    FoxglovePoint3 point;
    point.x = source.point().x();
    point.y = source.point().y();
    point.z = source.point().z();
    target.point = point;
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kPolygonStampedType) {
    PolygonStamped source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveSceneUpdate target = ToFoxglovePolygon(source);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kOdometryType) {
    Odometry source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveOdometry target;
    target.timestamp = ToFoxgloveTimestamp(source.header());
    target.frame_id = source.header().frame_id();
    target.body_frame_id = source.child_frame_id();
    target.pose = ToFoxglovePose(source.pose().pose().pose());
    target.linear_velocity = ToFoxgloveVector3(source.twist().twist().linear().x(),
                                               source.twist().twist().linear().y(),
                                               source.twist().twist().linear().z());
    target.angular_velocity = ToFoxgloveVector3(
        source.twist().twist().angular().x(),
        source.twist().twist().angular().y(),
        source.twist().twist().angular().z());
    CopyRepeatedToArray(source.pose().covariance(), &target.pose_covariance);
    CopyRepeatedToArray(source.twist().covariance(),
                        &target.velocity_covariance);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kCompressedImageType) {
    CompressedImage source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    const std::string format = NormalizeImageFormat(source.format());
    if (IsFoxgloveCompressedFormat(format)) {
      FoxgloveCompressedImage target;
      target.timestamp = ToFoxgloveTimestamp(source.header());
      target.frame_id = source.header().frame_id();
      target.format = format;
      target.data.assign(
          reinterpret_cast<const std::byte*>(source.data().data()),
          reinterpret_cast<const std::byte*>(source.data().data() +
                                             source.data().size()));
      return EncodeFoxgloveMessage(&target, adapted_payload);
    }
    return TryConvertRawCompressedImage(source, adapted_payload);
  }

  if (source_message_type == kCameraInfoType) {
    CameraInfo source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveCameraCalibration target;
    target.timestamp = ToFoxgloveTimestamp(source.header());
    target.frame_id = source.header().frame_id();
    target.width = source.width();
    target.height = source.height();
    target.distortion_model = source.distortion_model();
    target.d.assign(source.d().begin(), source.d().end());
    CopyRepeatedToArray(source.k(), &target.k);
    CopyRepeatedToArray(source.r(), &target.r);
    CopyRepeatedToArray(source.p(), &target.p);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kNavSatFixType) {
    NavSatFix source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveLocationFix target;
    target.timestamp = ToFoxgloveTimestamp(source.header());
    target.frame_id = source.header().frame_id();
    target.latitude = source.latitude();
    target.longitude = source.longitude();
    target.altitude = source.altitude();
    CopyRepeatedToArray(source.position_covariance(),
                        &target.position_covariance);
    target.position_covariance_type =
        ToFoxgloveCovarianceType(source.position_covariance_type());
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kOccupancyGridType) {
    OccupancyGrid source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveGrid target = ToFoxgloveGrid(source);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kVoxelGridType) {
    VoxelGrid source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveVoxelGrid target = ToFoxgloveVoxelGrid(source);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kOccupancyGridUpdateType) {
    OccupancyGridUpdate source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveGrid target = ToFoxgloveOccupancyGridUpdate(source);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kCostmapType) {
    Costmap source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveGrid target = ToFoxgloveCostmap(source);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kCostmapUpdateType) {
    CostmapUpdate source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveGrid target = ToFoxgloveCostmapUpdate(source);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kGridMapType) {
    GridMap source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveGrid target = ToFoxgloveGridMap(source);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kGridCellsType) {
    GridCells source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveSceneUpdate target = ToFoxgloveGridCells(source);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kRangeType) {
    Range source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveSceneUpdate target = ToFoxgloveRange(source);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kBoundingBox2DType) {
    BoundingBox2D source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveImageAnnotations target = ToFoxgloveImageAnnotations(source);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kBoundingBox2DArrayType) {
    BoundingBox2DArray source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveImageAnnotations target = ToFoxgloveImageAnnotations(source);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kDetection2DType) {
    Detection2D source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveImageAnnotations target = ToFoxgloveImageAnnotations(source);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kDetection2DArrayType) {
    Detection2DArray source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveImageAnnotations target = ToFoxgloveImageAnnotations(source);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kBoundingBox3DType) {
    BoundingBox3D source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveTimestamp timestamp;
    timestamp.sec = 0;
    timestamp.nsec = 0;
    FoxgloveSceneUpdate target =
        ToFoxgloveSceneUpdate(source, "map", timestamp, "bbox3d");
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kBoundingBox3DArrayType) {
    BoundingBox3DArray source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveSceneUpdate target = ToFoxgloveSceneUpdate(source);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kDetection3DType) {
    Detection3D source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveSceneUpdate target = ToFoxgloveSceneUpdate(source);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  if (source_message_type == kDetection3DArrayType) {
    Detection3DArray source;
    if (!source.ParseFromString(payload)) {
      return false;
    }
    FoxgloveSceneUpdate target = ToFoxgloveSceneUpdate(source);
    return EncodeFoxgloveMessage(&target, adapted_payload);
  }

  return false;
}

}  // namespace visualization
}  // namespace autonomy
