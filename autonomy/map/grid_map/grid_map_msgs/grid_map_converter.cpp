/*
 * grid_map_converter.cpp
 */

#include "autonomy/map/grid_map/grid_map_msgs/grid_map_converter.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>

#include <automsgs/msgs/time_utils.hpp>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_core/grid_map_core.hpp"
#include "autonomy/map/grid_map/grid_map_msgs/msg_helpers.hpp"

namespace grid_map {
namespace {

void setIdentityOrientation(automsgs::msgs::geometry_msgs::Pose* pose) {
  pose->mutable_orientation()->set_x(0.0);
  pose->mutable_orientation()->set_y(0.0);
  pose->mutable_orientation()->set_z(0.0);
  pose->mutable_orientation()->set_w(1.0);
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

}  // namespace grid_map
