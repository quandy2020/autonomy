/*
 * grid_map_converter.cpp
 */

#include "autonomy/map/grid_map/grid_map_msgs/grid_map_converter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <unordered_map>

#include <automsgs/msgs/sensor_msgs/point_cloud2_iterator.hpp>
#include <automsgs/msgs/time_utils.hpp>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_core/grid_map_core.hpp"
#include "autonomy/map/grid_map/grid_map_cv/grid_map_cv_converter.hpp"
#include "autonomy/map/grid_map/grid_map_msgs/msg_helpers.hpp"

namespace grid_map {
namespace {

void setIdentityOrientation(automsgs::msgs::geometry_msgs::Pose* pose) {
  pose->mutable_orientation()->set_x(0.0);
  pose->mutable_orientation()->set_y(0.0);
  pose->mutable_orientation()->set_z(0.0);
  pose->mutable_orientation()->set_w(1.0);
}

void setHeaderFromGridMap(const GridMap& gridMap,
                          automsgs::msgs::std_msgs::Header* header) {
  *header->mutable_stamp() =
      automsgs::msgs::builtin_interfaces::TimeFromNanoseconds(
          gridMap.getTimestamp());
  header->set_frame_id(gridMap.getFrameId());
}

int encodingToCvType(const std::string& encoding) {
  if (encoding == "mono8") {
    return CV_8UC1;
  }
  if (encoding == "mono16") {
    return CV_16UC1;
  }
  if (encoding == "bgr8" || encoding == "rgb8") {
    return CV_8UC3;
  }
  if (encoding == "bgra8" || encoding == "rgba8") {
    return CV_8UC4;
  }
  if (encoding == "bgr16" || encoding == "rgb16") {
    return CV_16UC3;
  }
  if (encoding == "bgra16" || encoding == "rgba16") {
    return CV_16UC4;
  }
  return -1;
}

bool matFromImageMessage(const automsgs::msgs::sensor_msgs::Image& image,
                         cv::Mat* mat) {
  const int cvType = encodingToCvType(image.encoding());
  if (cvType < 0) {
    AERROR << "Unsupported image encoding: " << image.encoding();
    return false;
  }
  const size_t elemSize = CV_ELEM_SIZE(cvType);
  const size_t expected =
      static_cast<size_t>(image.height()) * static_cast<size_t>(image.step());
  if (image.data().size() < expected) {
    AERROR << "Image data size does not match height/step.";
    return false;
  }
  *mat = cv::Mat(static_cast<int>(image.height()),
                 static_cast<int>(image.width()), cvType);
  if (image.step() == static_cast<uint32_t>(image.width()) * elemSize) {
    std::memcpy(mat->data, image.data().data(), expected);
  } else {
    for (uint32_t r = 0; r < image.height(); ++r) {
      std::memcpy(mat->ptr(static_cast<int>(r)),
                  image.data().data() + r * image.step(),
                  static_cast<size_t>(image.width()) * elemSize);
    }
  }
  return true;
}

bool imageMessageFromMat(const cv::Mat& mat, const std::string& encoding,
                         const GridMap& gridMap,
                         automsgs::msgs::sensor_msgs::Image* image) {
  if (!mat.isContinuous()) {
    AERROR << "OpenCV image must be continuous for conversion.";
    return false;
  }
  setHeaderFromGridMap(gridMap, image->mutable_header());
  image->set_height(static_cast<uint32_t>(mat.rows));
  image->set_width(static_cast<uint32_t>(mat.cols));
  image->set_encoding(encoding);
  image->set_is_bigendian(false);
  image->set_step(static_cast<uint32_t>(mat.step));
  image->set_data(reinterpret_cast<const char*>(mat.data),
                  static_cast<size_t>(mat.rows) * mat.step);
  return true;
}

}  // namespace

bool GridMapConverter::fromMessage(
    const automsgs::msgs::map_msgs::GridMap& message, GridMap& gridMap,
    const std::vector<std::string>& layers, bool copyBasicLayers,
    bool copyAllNonBasicLayers) {
  const auto& info = message.info();
  gridMap.setTimestamp(
      automsgs::msgs::builtin_interfaces::TimeToNanoseconds(info.header().stamp()));
  gridMap.setFrameId(info.header().frame_id());
  gridMap.setGeometry(
      Length(info.length_x(), info.length_y()), info.resolution(),
      Position(info.pose().position().x(), info.pose().position().y()));

  if (message.layers_size() != message.data_size()) {
    AERROR << "Different number of layers and data in grid map message.";
    return false;
  }

  for (int i = 0; i < message.layers_size(); ++i) {
    const std::string& layer = message.layers(i);
    if (!copyAllNonBasicLayers &&
        std::find(layers.begin(), layers.end(), layer) == layers.end()) {
      continue;
    }

    Matrix data;
    if (!multiArrayMessageCopyToMatrixEigen(message.data(i), data)) {
      return false;
    }
    gridMap.add(layer, data);
  }

  if (copyBasicLayers) {
    std::vector<std::string> basic_layers;
    basic_layers.reserve(message.basic_layers_size());
    for (const auto& layer : message.basic_layers()) {
      basic_layers.push_back(layer);
    }
    gridMap.setBasicLayers(basic_layers);
  }

  gridMap.setStartIndex(
      Index(message.outer_start_index(), message.inner_start_index()));
  return true;
}

bool GridMapConverter::fromMessage(
    const automsgs::msgs::map_msgs::GridMap& message, GridMap& gridMap) {
  return fromMessage(message, gridMap, std::vector<std::string>(), true, true);
}

void GridMapConverter::toMessage(const GridMap& gridMap,
                                 automsgs::msgs::map_msgs::GridMap& message) {
  toMessage(gridMap, gridMap.getLayers(), message);
}

void GridMapConverter::toMessage(const GridMap& gridMap,
                                const std::vector<std::string>& layers,
                                automsgs::msgs::map_msgs::GridMap& message) {
  message.Clear();
  auto* info = message.mutable_info();
  auto* header = info->mutable_header();
  *header->mutable_stamp() =
      automsgs::msgs::builtin_interfaces::TimeFromNanoseconds(
          gridMap.getTimestamp());
  header->set_frame_id(gridMap.getFrameId());
  info->set_resolution(static_cast<float>(gridMap.getResolution()));
  info->set_length_x(static_cast<float>(gridMap.getLength().x()));
  info->set_length_y(static_cast<float>(gridMap.getLength().y()));
  info->mutable_pose()->mutable_position()->set_x(gridMap.getPosition().x());
  info->mutable_pose()->mutable_position()->set_y(gridMap.getPosition().y());
  info->mutable_pose()->mutable_position()->set_z(0.0);
  setIdentityOrientation(info->mutable_pose());

  for (const auto& layer : layers) {
    message.add_layers(layer);
  }
  for (const auto& layer : gridMap.getBasicLayers()) {
    message.add_basic_layers(layer);
  }

  for (const auto& layer : layers) {
    auto* data_array = message.add_data();
    matrixEigenCopyToMultiArrayMessage(gridMap.get(layer), *data_array);
  }

  message.set_outer_start_index(
      static_cast<uint32_t>(gridMap.getStartIndex()(0)));
  message.set_inner_start_index(
      static_cast<uint32_t>(gridMap.getStartIndex()(1)));
}

bool GridMapConverter::fromOccupancyGrid(
    const automsgs::msgs::map_msgs::OccupancyGrid& occupancyGrid,
    const std::string& layer, GridMap& gridMap) {
  const Size size(occupancyGrid.info().width(), occupancyGrid.info().height());
  const double resolution = occupancyGrid.info().resolution();
  const Length length = resolution * size.cast<double>();
  const std::string& frameId = occupancyGrid.header().frame_id();
  Position position(occupancyGrid.info().origin().position().x(),
                    occupancyGrid.info().origin().position().y());
  // Different conventions of center of map.
  position += 0.5 * length.matrix();

  const auto& orientation = occupancyGrid.info().origin().orientation();
  const bool is_identity =
      (orientation.w() == 1.0f ||
       (orientation.x() == 0.0f && orientation.y() == 0.0f &&
        orientation.z() == 0.0f && orientation.w() == 0.0f));
  if (!is_identity) {
    AWARN << "Conversion of occupancy grid: Grid maps do not support "
             "orientation.";
    return false;
  }

  if (static_cast<size_t>(size.prod()) !=
      static_cast<size_t>(occupancyGrid.data_size())) {
    AWARN << "Conversion of occupancy grid: Size of data does not correspond "
             "to width * height.";
    return false;
  }

  if ((gridMap.getSize() != size).any() ||
      gridMap.getResolution() != resolution ||
      (gridMap.getLength() != length).any() ||
      gridMap.getPosition() != position || gridMap.getFrameId() != frameId ||
      !gridMap.getStartIndex().isZero()) {
    gridMap.setTimestamp(automsgs::msgs::builtin_interfaces::TimeToNanoseconds(
        occupancyGrid.header().stamp()));
    gridMap.setFrameId(frameId);
    gridMap.setGeometry(length, resolution, position);
  }

  Matrix data(size(0), size(1));
  const int n = occupancyGrid.data_size();
  for (int reverse_i = 0; reverse_i < n; ++reverse_i) {
    const int src = n - 1 - reverse_i;
    const int value = occupancyGrid.data(src);
    data(reverse_i) = (value != -1) ? static_cast<float>(value)
                                    : std::numeric_limits<float>::quiet_NaN();
  }

  gridMap.add(layer, data);
  return true;
}

void GridMapConverter::toOccupancyGrid(
    const GridMap& gridMap, const std::string& layer, float dataMin,
    float dataMax, automsgs::msgs::map_msgs::OccupancyGrid& occupancyGrid) {
  occupancyGrid.Clear();
  occupancyGrid.mutable_header()->set_frame_id(gridMap.getFrameId());
  *occupancyGrid.mutable_header()->mutable_stamp() =
      automsgs::msgs::builtin_interfaces::TimeFromNanoseconds(
          gridMap.getTimestamp());

  auto* info = occupancyGrid.mutable_info();
  *info->mutable_map_load_time() = occupancyGrid.header().stamp();
  info->set_resolution(static_cast<float>(gridMap.getResolution()));
  info->set_width(static_cast<uint32_t>(gridMap.getSize()(0)));
  info->set_height(static_cast<uint32_t>(gridMap.getSize()(1)));

  const Position position =
      gridMap.getPosition() - 0.5 * gridMap.getLength().matrix();
  info->mutable_origin()->mutable_position()->set_x(position.x());
  info->mutable_origin()->mutable_position()->set_y(position.y());
  info->mutable_origin()->mutable_position()->set_z(0.0);
  setIdentityOrientation(info->mutable_origin());

  const size_t nCells = static_cast<size_t>(gridMap.getSize().prod());
  occupancyGrid.mutable_data()->Resize(static_cast<int>(nCells), 0);

  constexpr float cellMin = 0.0f;
  constexpr float cellMax = 100.0f;
  constexpr float cellRange = cellMax - cellMin;
  const float range = dataMax - dataMin;

  for (GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
    float value =
        (gridMap.at(layer, *iterator) - dataMin) / (range == 0.0f ? 1.0f : range);
    if (std::isnan(value)) {
      value = -1.0f;
    } else {
      value = cellMin + std::min(std::max(0.0f, value), 1.0f) * cellRange;
    }
    const size_t index =
        getLinearIndexFromIndex(iterator.getUnwrappedIndex(), gridMap.getSize(),
                                false);
    // Reverse cell order because of different conventions.
    occupancyGrid.set_data(static_cast<int>(nCells - index - 1),
                           static_cast<int32_t>(value));
  }
}

bool GridMapConverter::saveToFile(const GridMap& gridMap,
                                  const std::string& filename,
                                  const std::vector<std::string>& layers) {
  automsgs::msgs::map_msgs::GridMap message;
  if (layers.empty()) {
    toMessage(gridMap, message);
  } else {
    toMessage(gridMap, layers, message);
  }
  std::ofstream out(filename, std::ios::binary);
  if (!out) {
    AERROR << "Failed to open file for writing: " << filename;
    return false;
  }
  if (!message.SerializeToOstream(&out)) {
    AERROR << "Failed to serialize grid map to: " << filename;
    return false;
  }
  return true;
}

bool GridMapConverter::loadFromFile(const std::string& filename,
                                   GridMap& gridMap) {
  std::ifstream in(filename, std::ios::binary);
  if (!in) {
    AERROR << "Failed to open file for reading: " << filename;
    return false;
  }
  automsgs::msgs::map_msgs::GridMap message;
  if (!message.ParseFromIstream(&in)) {
    AERROR << "Failed to parse grid map from: " << filename;
    return false;
  }
  return fromMessage(message, gridMap);
}

void GridMapConverter::toPointCloud(
    const GridMap& gridMap, const std::string& pointLayer,
    automsgs::msgs::sensor_msgs::PointCloud2& pointCloud) {
  toPointCloud(gridMap, gridMap.getLayers(), pointLayer, pointCloud);
}

void GridMapConverter::toPointCloud(
    const GridMap& gridMap, const std::vector<std::string>& layers,
    const std::string& pointLayer,
    automsgs::msgs::sensor_msgs::PointCloud2& pointCloud) {
  pointCloud.Clear();
  setHeaderFromGridMap(gridMap, pointCloud.mutable_header());
  pointCloud.set_is_dense(false);

  std::vector<std::string> fieldNames;
  for (const auto& layer : layers) {
    if (layer == pointLayer) {
      fieldNames.push_back("x");
      fieldNames.push_back("y");
      fieldNames.push_back("z");
    } else if (layer == "color") {
      fieldNames.push_back("rgb");
    } else {
      fieldNames.push_back(layer);
    }
  }

  int offset = 0;
  for (const auto& name : fieldNames) {
    auto* field = pointCloud.add_fields();
    field->set_name(name);
    field->set_count(1);
    field->set_datatype(automsgs::msgs::sensor_msgs::PointField::FLOAT32);
    field->set_offset(static_cast<uint32_t>(offset));
    offset += 4;
  }

  const size_t maxNumberOfPoints = gridMap.getSize().prod();
  pointCloud.set_height(1);
  pointCloud.set_width(static_cast<uint32_t>(maxNumberOfPoints));
  pointCloud.set_point_step(static_cast<uint32_t>(offset));
  pointCloud.set_row_step(pointCloud.width() * pointCloud.point_step());
  pointCloud.mutable_data()->resize(pointCloud.height() * pointCloud.row_step());

  std::unordered_map<std::string,
                     automsgs::msgs::sensor_msgs::PointCloud2Iterator<float>>
      fieldIterators;
  for (const auto& name : fieldNames) {
    fieldIterators.emplace(
        name, automsgs::msgs::sensor_msgs::PointCloud2Iterator<float>(
                  pointCloud, name));
  }

  GridMapIterator mapIterator(gridMap);
  const bool hasBasicLayers = !gridMap.getBasicLayers().empty();
  size_t realNumberOfPoints = 0;
  for (size_t i = 0; i < maxNumberOfPoints; ++i) {
    if (hasBasicLayers && !gridMap.isValid(*mapIterator)) {
      ++mapIterator;
      continue;
    }

    Position3 position;
    if (!gridMap.getPosition3(pointLayer, *mapIterator, position)) {
      ++mapIterator;
      continue;
    }

    for (auto& iterator : fieldIterators) {
      if (iterator.first == "x") {
        *iterator.second = static_cast<float>(position.x());
      } else if (iterator.first == "y") {
        *iterator.second = static_cast<float>(position.y());
      } else if (iterator.first == "z") {
        *iterator.second = static_cast<float>(position.z());
      } else if (iterator.first == "rgb") {
        *iterator.second = gridMap.at("color", *mapIterator);
      } else {
        *iterator.second = gridMap.at(iterator.first, *mapIterator);
      }
    }

    ++mapIterator;
    for (auto& iterator : fieldIterators) {
      ++iterator.second;
    }
    ++realNumberOfPoints;
  }

  if (realNumberOfPoints != maxNumberOfPoints) {
    pointCloud.set_width(static_cast<uint32_t>(realNumberOfPoints));
    pointCloud.set_row_step(pointCloud.width() * pointCloud.point_step());
    pointCloud.mutable_data()->resize(pointCloud.height() *
                                      pointCloud.row_step());
  }
}

void GridMapConverter::toPointCloud(
    const SignedDistanceField& signedDistanceField,
    automsgs::msgs::sensor_msgs::PointCloud2& pointCloud, size_t decimation,
    const std::function<bool(float)>& condition) {
  pointCloud.Clear();
  *pointCloud.mutable_header()->mutable_stamp() =
      automsgs::msgs::builtin_interfaces::TimeFromNanoseconds(
          signedDistanceField.getTime());
  pointCloud.mutable_header()->set_frame_id(signedDistanceField.getFrameId());

  const std::vector<std::string> fieldNames{"x", "y", "z", "intensity"};
  size_t offset = 0;
  for (const auto& name : fieldNames) {
    auto* field = pointCloud.add_fields();
    field->set_name(name);
    field->set_count(1);
    field->set_datatype(automsgs::msgs::sensor_msgs::PointField::FLOAT32);
    field->set_offset(static_cast<uint32_t>(offset));
    offset += sizeof(float);
  }

  const size_t pointCloudMaxSize = signedDistanceField.size();
  pointCloud.set_height(1);
  pointCloud.set_width(static_cast<uint32_t>(pointCloudMaxSize));
  pointCloud.set_point_step(static_cast<uint32_t>(offset));
  pointCloud.set_row_step(pointCloud.width() * pointCloud.point_step());
  pointCloud.mutable_data()->resize(pointCloud.height() * pointCloud.row_step());

  automsgs::msgs::sensor_msgs::PointCloud2Iterator<float> iter_x(pointCloud,
                                                                 "x");
  automsgs::msgs::sensor_msgs::PointCloud2Iterator<float> iter_y(pointCloud,
                                                                 "y");
  automsgs::msgs::sensor_msgs::PointCloud2Iterator<float> iter_z(pointCloud,
                                                                 "z");
  automsgs::msgs::sensor_msgs::PointCloud2Iterator<float> iter_i(pointCloud,
                                                                 "intensity");

  size_t addedPoints = 0;
  signedDistanceField.filterPoints(
      [&](const Position3& p, float sdfValue,
          const SignedDistanceField::Derivative3&) {
        if (condition(sdfValue)) {
          *iter_x = static_cast<float>(p.x());
          *iter_y = static_cast<float>(p.y());
          *iter_z = static_cast<float>(p.z());
          *iter_i = sdfValue;
          ++iter_x;
          ++iter_y;
          ++iter_z;
          ++iter_i;
          ++addedPoints;
        }
      },
      decimation);

  if (addedPoints != pointCloudMaxSize) {
    pointCloud.set_width(static_cast<uint32_t>(addedPoints));
    pointCloud.set_row_step(pointCloud.width() * pointCloud.point_step());
    pointCloud.mutable_data()->resize(pointCloud.height() *
                                      pointCloud.row_step());
  }
}

bool GridMapConverter::initializeFromImage(
    const automsgs::msgs::sensor_msgs::Image& image, double resolution,
    GridMap& gridMap, const Position& position) {
  const double lengthX = resolution * image.height();
  const double lengthY = resolution * image.width();
  gridMap.setGeometry(Length(lengthX, lengthY), resolution, position);
  gridMap.setFrameId(image.header().frame_id());
  gridMap.setTimestamp(automsgs::msgs::builtin_interfaces::TimeToNanoseconds(
      image.header().stamp()));
  return true;
}

bool GridMapConverter::addLayerFromImage(
    const automsgs::msgs::sensor_msgs::Image& image, const std::string& layer,
    GridMap& gridMap, float lowerValue, float upperValue,
    double alphaThreshold) {
  cv::Mat mat;
  if (!matFromImageMessage(image, &mat)) {
    return false;
  }
  switch (encodingToCvType(image.encoding())) {
    case CV_8UC1:
      return GridMapCvConverter::addLayerFromImage<unsigned char, 1>(
          mat, layer, gridMap, lowerValue, upperValue, alphaThreshold);
    case CV_8UC3:
      return GridMapCvConverter::addLayerFromImage<unsigned char, 3>(
          mat, layer, gridMap, lowerValue, upperValue, alphaThreshold);
    case CV_8UC4:
      return GridMapCvConverter::addLayerFromImage<unsigned char, 4>(
          mat, layer, gridMap, lowerValue, upperValue, alphaThreshold);
    case CV_16UC1:
      return GridMapCvConverter::addLayerFromImage<unsigned short, 1>(
          mat, layer, gridMap, lowerValue, upperValue, alphaThreshold);
    case CV_16UC3:
      return GridMapCvConverter::addLayerFromImage<unsigned short, 3>(
          mat, layer, gridMap, lowerValue, upperValue, alphaThreshold);
    case CV_16UC4:
      return GridMapCvConverter::addLayerFromImage<unsigned short, 4>(
          mat, layer, gridMap, lowerValue, upperValue, alphaThreshold);
    default:
      AERROR << "Expected MONO8/16 or RGB(A)/BGR(A) 8/16 image encoding.";
      return false;
  }
}

bool GridMapConverter::addColorLayerFromImage(
    const automsgs::msgs::sensor_msgs::Image& image, const std::string& layer,
    GridMap& gridMap) {
  cv::Mat mat;
  if (!matFromImageMessage(image, &mat)) {
    return false;
  }
  switch (encodingToCvType(image.encoding())) {
    case CV_8UC3:
      return GridMapCvConverter::addColorLayerFromImage<unsigned char, 3>(
          mat, layer, gridMap);
    case CV_8UC4:
      return GridMapCvConverter::addColorLayerFromImage<unsigned char, 4>(
          mat, layer, gridMap);
    case CV_16UC3:
      return GridMapCvConverter::addColorLayerFromImage<unsigned short, 3>(
          mat, layer, gridMap);
    case CV_16UC4:
      return GridMapCvConverter::addColorLayerFromImage<unsigned short, 4>(
          mat, layer, gridMap);
    default:
      AERROR << "Expected RGB(A)/BGR(A) 8/16 image encoding for color layer.";
      return false;
  }
}

bool GridMapConverter::toImage(
    const GridMap& gridMap, const std::string& layer, const std::string& encoding,
    automsgs::msgs::sensor_msgs::Image& image) {
  cv::Mat mat;
  if (!toCvImage(gridMap, layer, encoding, mat)) {
    return false;
  }
  return imageMessageFromMat(mat, encoding, gridMap, &image);
}

bool GridMapConverter::toImage(
    const GridMap& gridMap, const std::string& layer, const std::string& encoding,
    float lowerValue, float upperValue,
    automsgs::msgs::sensor_msgs::Image& image) {
  cv::Mat mat;
  if (!toCvImage(gridMap, layer, encoding, lowerValue, upperValue, mat)) {
    return false;
  }
  return imageMessageFromMat(mat, encoding, gridMap, &image);
}

bool GridMapConverter::toCvImage(const GridMap& gridMap, const std::string& layer,
                                const std::string& encoding, cv::Mat& image) {
  const float minValue = gridMap.get(layer).minCoeffOfFinites();
  const float maxValue = gridMap.get(layer).maxCoeffOfFinites();
  return toCvImage(gridMap, layer, encoding, minValue, maxValue, image);
}

bool GridMapConverter::toCvImage(const GridMap& gridMap, const std::string& layer,
                                const std::string& encoding, float lowerValue,
                                float upperValue, cv::Mat& image) {
  switch (encodingToCvType(encoding)) {
    case CV_8UC1:
      return GridMapCvConverter::toImage<unsigned char, 1>(
          gridMap, layer, encodingToCvType(encoding), lowerValue, upperValue,
          image);
    case CV_8UC3:
      return GridMapCvConverter::toImage<unsigned char, 3>(
          gridMap, layer, encodingToCvType(encoding), lowerValue, upperValue,
          image);
    case CV_8UC4:
      return GridMapCvConverter::toImage<unsigned char, 4>(
          gridMap, layer, encodingToCvType(encoding), lowerValue, upperValue,
          image);
    case CV_16UC1:
      return GridMapCvConverter::toImage<unsigned short, 1>(
          gridMap, layer, encodingToCvType(encoding), lowerValue, upperValue,
          image);
    case CV_16UC3:
      return GridMapCvConverter::toImage<unsigned short, 3>(
          gridMap, layer, encodingToCvType(encoding), lowerValue, upperValue,
          image);
    case CV_16UC4:
      return GridMapCvConverter::toImage<unsigned short, 4>(
          gridMap, layer, encodingToCvType(encoding), lowerValue, upperValue,
          image);
    default:
      AERROR << "Expected MONO8/16 or RGB(A)/BGR(A) 8/16 image encoding.";
      return false;
  }
}

}  // namespace grid_map
