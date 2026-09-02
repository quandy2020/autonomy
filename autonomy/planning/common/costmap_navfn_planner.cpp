/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/planning/common/costmap_navfn_planner.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <vector>

#include <automsgs/msgs/geometry_msgs/pose.pb.h>

#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/planning/planner/navfn/navfn.hpp"

namespace autonomy::planning::common {
namespace {

using map::costmap_2d::Costmap2D;
using map::costmap_2d::FREE_SPACE;
using planner::navfn::NavFn;

bool WorldToMap(const Costmap2D& costmap, double wx, double wy, unsigned int* mx,
                unsigned int* my) {
  if (mx == nullptr || my == nullptr) {
    return false;
  }
  if (!costmap.worldToMap(wx, wy, *mx, *my)) {
    return false;
  }
  return *mx < costmap.getSizeInCellsX() && *my < costmap.getSizeInCellsY();
}

void MapToWorld(const Costmap2D& costmap, double mx, double my, double* wx,
                double* wy) {
  costmap.mapToWorld(static_cast<unsigned int>(mx),
                     static_cast<unsigned int>(my), *wx, *wy);
}

double SquaredDistance(const automsgs::msgs::geometry_msgs::Pose& a,
                       const automsgs::msgs::geometry_msgs::Pose& b) {
  const double dx = a.position().x() - b.position().x();
  const double dy = a.position().y() - b.position().y();
  return dx * dx + dy * dy;
}

double PointPotential(const NavFn& planner, const Costmap2D& costmap,
                      const automsgs::msgs::geometry_msgs::Point& world_point) {
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!WorldToMap(costmap, world_point.x(), world_point.y(), &mx, &my)) {
    return std::numeric_limits<double>::max();
  }
  const unsigned int index = my * planner.nx + mx;
  return planner.potarr[index];
}

bool ExtractPath(const NavFn& planner, const Costmap2D& costmap,
                 const automsgs::msgs::geometry_msgs::Pose& goal,
                 std::vector<automsgs::msgs::geometry_msgs::Point>* path) {
  if (path == nullptr) {
    return false;
  }
  path->clear();

  unsigned int mx = 0;
  unsigned int my = 0;
  if (!WorldToMap(costmap, goal.position().x(), goal.position().y(), &mx,
                  &my)) {
    return false;
  }
  int map_goal[2] = {static_cast<int>(mx), static_cast<int>(my)};

  NavFn& mutable_planner = const_cast<NavFn&>(planner);
  mutable_planner.setStart(map_goal);

  const int max_cycles =
      (costmap.getSizeInCellsX() >= costmap.getSizeInCellsY())
          ? static_cast<int>(costmap.getSizeInCellsX() * 4)
          : static_cast<int>(costmap.getSizeInCellsY() * 4);
  if (mutable_planner.calcPath(max_cycles) == 0) {
    return false;
  }

  float* x = mutable_planner.getPathX();
  float* y = mutable_planner.getPathY();
  const int len = mutable_planner.getPathLen();
  double length = 0.0;
  automsgs::msgs::geometry_msgs::Point prev;
  bool has_prev = false;
  for (int i = len - 1; i >= 0; --i) {
    double world_x = 0.0;
    double world_y = 0.0;
    MapToWorld(costmap, x[i], y[i], &world_x, &world_y);
    automsgs::msgs::geometry_msgs::Point pt;
    pt.set_x(world_x);
    pt.set_y(world_y);
    if (has_prev) {
      length += std::hypot(pt.x() - prev.x(), pt.y() - prev.y());
    }
    prev = pt;
    has_prev = true;
    path->push_back(pt);
  }
  if (!path->empty()) {
    return true;
  }
  (void)length;
  return false;
}

}  // namespace

double CostmapNavfnPlanner::Plan(
    const Costmap2D& costmap, double from_x, double from_y, double to_x,
    double to_y, std::vector<automsgs::msgs::geometry_msgs::Point>* path,
    const CostmapNavfnPlannerOptions& options,
    std::function<bool()> cancel_checker) {
  if (path == nullptr) {
    return std::numeric_limits<double>::infinity();
  }
  path->clear();

  unsigned int start_mx = 0;
  unsigned int start_my = 0;
  unsigned int goal_mx = 0;
  unsigned int goal_my = 0;
  if (!WorldToMap(costmap, from_x, from_y, &start_mx, &start_my) ||
      !WorldToMap(costmap, to_x, to_y, &goal_mx, &goal_my)) {
    return std::numeric_limits<double>::infinity();
  }

  const unsigned int size_x = costmap.getSizeInCellsX();
  const unsigned int size_y = costmap.getSizeInCellsY();
  const size_t map_size = static_cast<size_t>(size_x) * size_y;

  std::vector<unsigned char> cost_copy(map_size);
  std::memcpy(cost_copy.data(), costmap.getCharMap(), map_size);
  cost_copy[static_cast<size_t>(start_my) * size_x + start_mx] = FREE_SPACE;
  cost_copy[static_cast<size_t>(goal_my) * size_x + goal_mx] = FREE_SPACE;

  auto planner = std::make_unique<NavFn>(static_cast<int>(size_x),
                                         static_cast<int>(size_y));
  planner->setNavArr(size_x, size_y);
  planner->setCostmap(cost_copy.data(), true, options.allow_unknown);

  int map_start[2] = {static_cast<int>(start_mx), static_cast<int>(start_my)};
  int map_goal[2] = {static_cast<int>(goal_mx), static_cast<int>(goal_my)};
  planner->setStart(map_goal);
  planner->setGoal(map_start);

  if (!cancel_checker) {
    cancel_checker = []() { return false; };
  }

  const bool propagated = options.use_astar
                              ? planner->calcNavFnAstar(cancel_checker)
                              : planner->calcNavFnDijkstra(cancel_checker, true);
  if (!propagated || planner->wasPropagationCancelled()) {
    return std::numeric_limits<double>::infinity();
  }

  automsgs::msgs::geometry_msgs::Pose goal_pose;
  goal_pose.mutable_position()->set_x(to_x);
  goal_pose.mutable_position()->set_y(to_y);

  automsgs::msgs::geometry_msgs::Pose best_pose = goal_pose;
  bool found = false;
  double potential = PointPotential(*planner, costmap, goal_pose.position());
  if (potential < POT_HIGH) {
    best_pose = goal_pose;
    found = true;
  } else if (options.tolerance_m > 0.0) {
    const double resolution = costmap.getResolution();
    double best_sdist = std::numeric_limits<double>::max();
    automsgs::msgs::geometry_msgs::Pose probe = goal_pose;
    probe.mutable_position()->set_y(to_y - options.tolerance_m);
    while (probe.position().y() <= to_y + options.tolerance_m) {
      probe.mutable_position()->set_x(to_x - options.tolerance_m);
      while (probe.position().x() <= to_x + options.tolerance_m) {
        potential = PointPotential(*planner, costmap, probe.position());
        const double sdist = SquaredDistance(probe, goal_pose);
        if (potential < POT_HIGH && sdist < best_sdist) {
          best_sdist = sdist;
          best_pose = probe;
          found = true;
        }
        probe.mutable_position()->set_x(probe.position().x() + resolution);
      }
      probe.mutable_position()->set_y(probe.position().y() + resolution);
    }
  }

  if (!found || !ExtractPath(*planner, costmap, best_pose, path)) {
    return std::numeric_limits<double>::infinity();
  }

  double length = 0.0;
  for (size_t i = 1; i < path->size(); ++i) {
    length += std::hypot(path->at(i).x() - path->at(i - 1).x(),
                         path->at(i).y() - path->at(i - 1).y());
  }
  return length;
}

}  // namespace autonomy::planning::common
