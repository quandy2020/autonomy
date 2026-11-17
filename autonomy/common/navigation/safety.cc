/**
 * @file safety.cc
 * @brief Internal Opt safety implementation reused by native navigation.
 * @details Keeps the geometry implementation independent of ROS and map ownership.
 */
#include "opt/navigation/safety.h"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace opt::navigation {
namespace {
constexpr double kFar = 1e30;

// Lower envelope of squared-distance parabolas. Infinite sites are omitted.
std::vector<double> SquaredDistanceTransform(const std::vector<double>& input) {
  const int n = static_cast<int>(input.size());
  std::vector<int> sites;
  std::vector<double> boundaries;
  for (int q = 0; q < n; ++q) {
    if (input[q] >= kFar / 2.0) continue;
    double boundary = -std::numeric_limits<double>::infinity();
    while (!sites.empty()) {
      const int p = sites.back();
      boundary = ((input[q] + static_cast<double>(q) * q) -
                  (input[p] + static_cast<double>(p) * p)) / (2.0 * (q - p));
      if (boundary > boundaries.back()) break;
      sites.pop_back();
      boundaries.pop_back();
    }
    if (sites.empty()) boundary = -std::numeric_limits<double>::infinity();
    sites.push_back(q);
    boundaries.push_back(boundary);
  }
  std::vector<double> output(input.size(), kFar);
  if (sites.empty()) return output;
  std::size_t k = 0;
  for (int q = 0; q < n; ++q) {
    while (k + 1 < sites.size() && boundaries[k + 1] < q) ++k;
    const double delta = q - sites[k];
    output[q] = delta * delta + input[sites[k]];
  }
  return output;
}
}  // namespace

GeometrySafety::GeometrySafety(std::shared_ptr<const GridMap> map, SafetyOptions options)
    : map_(std::move(map)), options_(options) {
  const double values[] = {options.robot_radius, options.horizontal_margin, options.robot_height,
                           options.vertical_margin, options.max_floor_step};
  for (double value : values) {
    if (!std::isfinite(value) || value < 0.0) throw std::invalid_argument("Invalid safety option");
  }
  if (!map_ || options.robot_radius <= 0.0 || options.robot_height <= 0.0) {
    throw std::invalid_argument("Safety needs a map and positive robot dimensions");
  }
  BuildDistanceField();
}

void GeometrySafety::BuildDistanceField() {
  distances_.assign(map_->size(), kFar);
  for (int x = 0; x < map_->width(); ++x) {
    std::vector<double> column(map_->height(), kFar);
    for (int y = 0; y < map_->height(); ++y) {
      if (map_->At(x, y).state != CellState::kFree) column[y] = 0.0;
    }
    const auto transformed = SquaredDistanceTransform(column);
    for (int y = 0; y < map_->height(); ++y) distances_[map_->Index(x, y)] = transformed[y];
  }
  for (int y = 0; y < map_->height(); ++y) {
    std::vector<double> row(map_->width());
    for (int x = 0; x < map_->width(); ++x) row[x] = distances_[map_->Index(x, y)];
    const auto transformed = SquaredDistanceTransform(row);
    for (int x = 0; x < map_->width(); ++x) {
      distances_[map_->Index(x, y)] = std::sqrt(transformed[x]) * map_->resolution();
    }
  }
}

double GeometrySafety::Clearance(const Pose2d& pose) const {
  const auto cell = map_->WorldToCell(pose);
  if (!cell) return 0.0;
  const Pose2d center = map_->CellCenter(cell->x, cell->y);
  // Subtract a cell's circumradius, then use the distance function's Lipschitz bound.
  const double obstacle = distances_[map_->Index(cell->x, cell->y)] -
                          map_->resolution() / std::sqrt(2.0) - Distance(center, pose);
  const double boundary = std::min({pose.x - map_->origin_x(), pose.y - map_->origin_y(),
      map_->origin_x() + map_->width() * map_->resolution() - pose.x,
      map_->origin_y() + map_->height() * map_->resolution() - pose.y});
  return std::max(0.0, std::min(obstacle, boundary));
}

SafetyResult GeometrySafety::VerifyPose(const Pose2d& pose, double extra_padding) const {
  if (!IsFinite(pose) || !std::isfinite(extra_padding) || extra_padding < 0.0) {
    return {false, 0.0, "invalid pose or padding"};
  }
  const auto cell = map_->WorldToCell(pose);
  if (!cell) return {false, 0.0, "outside map"};
  const double clearance = Clearance(pose);
  const double radius = options_.robot_radius + options_.horizontal_margin + extra_padding;
  if (!std::isfinite(radius) || clearance + 1e-12 < radius) {
    return {false, clearance, "insufficient horizontal clearance"};
  }
  const double res = map_->resolution();
  const int x0 = std::max(0, static_cast<int>(std::floor((pose.x - radius - map_->origin_x()) / res)));
  const int y0 = std::max(0, static_cast<int>(std::floor((pose.y - radius - map_->origin_y()) / res)));
  const int x1 = std::min(map_->width() - 1,
      static_cast<int>(std::floor((pose.x + radius - map_->origin_x()) / res)));
  const int y1 = std::min(map_->height() - 1,
      static_cast<int>(std::floor((pose.y + radius - map_->origin_y()) / res)));
  double min_floor = std::numeric_limits<double>::infinity();
  double max_floor = -std::numeric_limits<double>::infinity();
  double min_ceiling = std::numeric_limits<double>::infinity();
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      const double left = map_->origin_x() + x * res;
      const double bottom = map_->origin_y() + y * res;
      const double dx = pose.x - std::clamp(pose.x, left, left + res);
      const double dy = pose.y - std::clamp(pose.y, bottom, bottom + res);
      if (dx * dx + dy * dy > radius * radius + 1e-12) continue;
      const auto& surface = map_->At(x, y);
      if (surface.state != CellState::kFree) return {false, clearance, "unknown or occupied footprint"};
      if (!std::isfinite(surface.floor_height) || std::isnan(surface.ceiling_height) ||
          surface.ceiling_height == -std::numeric_limits<double>::infinity()) {
        return {false, clearance, "unknown surface geometry"};
      }
      min_floor = std::min(min_floor, surface.floor_height);
      max_floor = std::max(max_floor, surface.floor_height);
      min_ceiling = std::min(min_ceiling, surface.ceiling_height);
    }
  }
  if (max_floor - min_floor > options_.max_floor_step + 1e-12) {
    return {false, clearance, "floor step exceeds limit"};
  }
  if (min_ceiling - max_floor < options_.robot_height + options_.vertical_margin) {
    return {false, clearance, "insufficient vertical clearance"};
  }
  return {true, clearance, "safe"};
}

SafetyResult GeometrySafety::VerifySegment(const Pose2d& from, const Pose2d& to) const {
  if (!IsFinite(from) || !IsFinite(to)) return {false, 0.0, "invalid segment"};
  if (!map_->WorldToCell(from) || !map_->WorldToCell(to)) return {false, 0.0, "outside map"};
  const double length = Distance(from, to);
  if (length <= 1e-12) return VerifyPose(to);
  if (!std::isfinite(length) || length / map_->resolution() > 1000000.0) {
    return {false, 0.0, "segment exceeds validation budget"};
  }
  const auto first = VerifyPose(from), last = VerifyPose(to);
  if (!first.safe) return first;
  if (!last.safe) return last;
  double clearance = std::min(first.clearance, last.clearance);
  struct Interval { double begin, end; int depth; };
  std::vector<Interval> intervals{{0, 1, 0}};
  std::size_t checks = 0;
  while (!intervals.empty()) {
    const auto interval = intervals.back(); intervals.pop_back();
    if (++checks > 200000) return {false, clearance, "segment exceeds validation budget"};
    const double u = (interval.begin + interval.end) * 0.5;
    const Pose2d midpoint{from.x + u * (to.x - from.x), from.y + u * (to.y - from.y),
                         from.yaw + u * NormalizeAngle(to.yaw - from.yaw)};
    // The expanded midpoint disc contains the entire subsegment's swept footprint.
    // Subdivide near obstacles rather than adding fixed padding to tight endpoints,
    // which previously made a safe robot unable to move away from an obstacle.
    const double padding = length * (interval.end - interval.begin) * 0.5;
    const auto bounded = VerifyPose(midpoint, padding);
    clearance = std::min(clearance, bounded.clearance);
    if (bounded.safe) continue;
    const auto point = VerifyPose(midpoint);
    if (!point.safe) return {false, clearance, point.reason};
    if (interval.depth >= 30 || padding < 1e-8)
      return {false, clearance, "continuous segment clearance not certified"};
    intervals.push_back({u, interval.end, interval.depth + 1});
    intervals.push_back({interval.begin, u, interval.depth + 1});
  }
  return {true, clearance, "safe"};
}

SafetyResult GeometrySafety::VerifyPath(const Path& path) const {
  if (path.frame_id != map_->frame_id() || path.poses.empty()) {
    return {false, 0.0, "empty path or frame mismatch"};
  }
  auto result = VerifyPose(path.poses.front());
  if (!result.safe) return result;
  double clearance = result.clearance;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    result = VerifySegment(path.poses[i - 1], path.poses[i]);
    clearance = std::min(clearance, result.clearance);
    if (!result.safe) return {false, clearance, result.reason};
  }
  return {true, clearance, "safe"};
}

}  // namespace opt::navigation
