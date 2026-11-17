/**
 * @file reachability.cc
 * @brief Internal Opt reachability implementation reused by native navigation.
 * @details Keeps the geometry implementation independent of ROS and map ownership.
 */
#include "opt/navigation/reachability.h"

#include <algorithm>
#include <array>
#include <queue>
#include <stdexcept>
#include <utility>

namespace opt::navigation {

ReachabilityField::ReachabilityField(std::shared_ptr<const GeometrySafety> safety, Pose2d start,
                                     std::size_t max_expansions,
                                     std::function<bool()> cancel_checker)
    : safety_(std::move(safety)), start_(start) {
  if (!safety_) throw std::invalid_argument("Reachability needs a safety checker");
  const auto& map = safety_->map();
  distance_.assign(map.size(), std::numeric_limits<double>::infinity());
  parent_.assign(map.size(), map.size());
  settled_.assign(map.size(), false);
  const auto source = map.WorldToCell(start);
  if (!source || !safety_->VerifyPose(start).safe) return;
  const Pose2d center = map.CellCenter(source->x, source->y);
  if (!safety_->VerifySegment(start, center).safe) return;
  const std::size_t source_index = map.Index(source->x, source->y);
  distance_[source_index] = Distance(start, center);
  using Entry = std::pair<double, std::size_t>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  queue.push({distance_[source_index], source_index});
  std::vector<int> traversable(map.size(), -1);
  traversable[source_index] = 1;
  constexpr std::array<CellIndex, 4> offsets{{{1, 0}, {0, 1}, {-1, 0}, {0, -1}}};
  while (!queue.empty()) {
    if (cancel_checker && cancel_checker()) { canceled_ = true; break; }
    const auto [cost, index] = queue.top();
    queue.pop();
    if (cost > distance_[index]) continue;
    if (settled_[index]) continue;
    if (max_expansions != 0 && expansions_ >= max_expansions) {
      budget_exhausted_ = true;
      break;
    }
    settled_[index] = true;
    ++expansions_;
    const auto current = map.FromIndex(index);
    const auto from = map.CellCenter(current.x, current.y);
    for (const auto& offset : offsets) {
      const int x = current.x + offset.x;
      const int y = current.y + offset.y;
      if (!map.Contains(x, y)) continue;
      const std::size_t next = map.Index(x, y);
      const double next_cost = cost + map.resolution();
      if (next_cost + 1e-12 >= distance_[next]) continue;
      const auto to = map.CellCenter(x, y);
      if (traversable[next] < 0) traversable[next] = safety_->VerifyPose(to).safe ? 1 : 0;
      if (!traversable[next] || !safety_->VerifySegment(from, to).safe) continue;
      distance_[next] = next_cost;
      parent_[next] = index;
      queue.push({next_cost, next});
    }
  }
}

double ReachabilityField::DistanceTo(const Pose2d& target) const {
  const auto& map = safety_->map();
  const auto cell = map.WorldToCell(target);
  if (!cell) return std::numeric_limits<double>::infinity();
  const double cost = distance_[map.Index(cell->x, cell->y)];
  if (!settled_[map.Index(cell->x, cell->y)] || !std::isfinite(cost) ||
      !safety_->VerifyPose(target).safe ||
      !safety_->VerifySegment(map.CellCenter(cell->x, cell->y), target).safe) {
    return std::numeric_limits<double>::infinity();
  }
  return cost + Distance(map.CellCenter(cell->x, cell->y), target);
}

Path ReachabilityField::PathTo(const Pose2d& target) const {
  Path path;
  path.frame_id = safety_->map().frame_id();
  if (!std::isfinite(DistanceTo(target))) return path;
  const auto& map = safety_->map();
  const auto cell = map.WorldToCell(target);
  std::vector<Pose2d> reversed;
  std::size_t index = map.Index(cell->x, cell->y);
  while (index < map.size()) {
    const auto at = map.FromIndex(index);
    reversed.push_back(map.CellCenter(at.x, at.y));
    index = parent_[index];
  }
  path.poses.push_back(start_);
  for (auto it = reversed.rbegin(); it != reversed.rend(); ++it) {
    if (Distance(path.poses.back(), *it) > 1e-10) path.poses.push_back(*it);
  }
  if (Distance(path.poses.back(), target) > 1e-10) path.poses.push_back(target);
  else path.poses.back() = target;
  for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
    path.poses[i].yaw = std::atan2(path.poses[i + 1].y - path.poses[i].y,
                                 path.poses[i + 1].x - path.poses[i].x);
  }
  path.poses.back().yaw = target.yaw;
  return path;
}

PlanningResult GridPlanner::Plan(const PlanningRequest& request) const {
  PlanningResult result;
  result.request_id = request.request_id;
  result.goal_epoch = request.goal_epoch;
  result.goal_revision = request.goal_revision;
  result.map_revision = request.map.revision;
  if (!request.map.grid) { result.reason = "missing map"; return result; }
  auto safety = std::make_shared<GeometrySafety>(request.map.grid, options_);
  ReachabilityField reachable(safety, request.start);
  result.path = reachable.PathTo(request.target);
  const auto check = safety->VerifyPath(result.path);
  result.success = check.safe;
  result.reason = check.reason;
  return result;
}

}  // namespace opt::navigation
