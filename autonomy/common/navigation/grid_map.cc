/**
 * @file grid_map.cc
 * @brief Internal Opt grid map implementation reused by native navigation.
 * @details Keeps the geometry implementation independent of ROS and map ownership.
 */
#include "opt/navigation/grid_map.h"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace opt::navigation {

GridMap::GridMap(int width, int height, double resolution, double origin_x,
                 double origin_y, std::string frame_id)
    : width_(width), height_(height), resolution_(resolution),
      origin_x_(origin_x), origin_y_(origin_y), frame_id_(std::move(frame_id)) {
  if (width <= 0 || height <= 0 || !std::isfinite(resolution) || resolution <= 0.0 ||
      !std::isfinite(origin_x) || !std::isfinite(origin_y) || frame_id_.empty() ||
      !std::isfinite(origin_x + width * resolution) ||
      !std::isfinite(origin_y + height * resolution)) {
    throw std::invalid_argument("Invalid grid geometry or frame");
  }
  if (static_cast<std::size_t>(width) > cells_.max_size() / static_cast<std::size_t>(height)) {
    throw std::length_error("Grid dimensions overflow");
  }
  cells_.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
}

bool GridMap::Contains(int x, int y) const {
  return x >= 0 && x < width_ && y >= 0 && y < height_;
}
std::size_t GridMap::Index(int x, int y) const {
  if (!Contains(x, y)) throw std::out_of_range("Grid cell outside map");
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) +
         static_cast<std::size_t>(x);
}
CellIndex GridMap::FromIndex(std::size_t index) const {
  if (index >= size()) throw std::out_of_range("Invalid grid index");
  return {static_cast<int>(index % width_), static_cast<int>(index / width_)};
}
std::optional<CellIndex> GridMap::WorldToCell(const Pose2d& pose) const {
  if (!IsFinite(pose)) return std::nullopt;
  const double x = (pose.x - origin_x_) / resolution_;
  const double y = (pose.y - origin_y_) / resolution_;
  if (!std::isfinite(x) || !std::isfinite(y) || x < 0.0 || y < 0.0 ||
      x >= width_ || y >= height_) return std::nullopt;
  return CellIndex{static_cast<int>(std::floor(x)), static_cast<int>(std::floor(y))};
}
Pose2d GridMap::CellCenter(int x, int y) const {
  Index(x, y);
  return {origin_x_ + (x + 0.5) * resolution_, origin_y_ + (y + 0.5) * resolution_, 0.0};
}
const GridCell& GridMap::At(int x, int y) const { return cells_.at(Index(x, y)); }
void GridMap::Set(int x, int y, const GridCell& cell) { cells_.at(Index(x, y)) = cell; }
void GridMap::Fill(const GridCell& cell) { std::fill(cells_.begin(), cells_.end(), cell); }

}  // namespace opt::navigation
