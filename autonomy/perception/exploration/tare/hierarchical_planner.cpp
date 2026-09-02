/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/perception/exploration/tare/hierarchical_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "autonomy/perception/exploration/common/planning_utilities.hpp"
#include "autonomy/perception/exploration/common/planning_utilities.hpp"
#include "autonomy/perception/exploration/common/planning_utilities.hpp"
#include "autonomy/perception/exploration/tare/tsp_solver.hpp"

namespace autonomy::perception::exploration {
namespace {

double YawFromOdom(const automsgs::msgs::nav_msgs::Odometry& odom) {
  const auto& q = odom.pose().pose().pose().orientation();
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

}  // namespace

HierarchicalPlanner::HierarchicalPlanner(
    const proto::ExplorationOptions& options)
    : env_(options),
      cloud_manager_(options),
      viewpoints_(options),
      grid_world_(options),
      keypose_graph_(options),
      local_planner_(options),
      momentum_planner_(options),
      visualizer_(options),
      terrain_map_(options) {
  SetOptions(options);
}

void HierarchicalPlanner::SetOptions(const proto::ExplorationOptions& options) {
  options_ = options;
  frame_id_ = options.map_frame().empty() ? "map" : options.map_frame();
  env_.SetOptions(options);
  cloud_manager_.SetOptions(options);
  viewpoints_.SetOptions(options);
  grid_world_.SetOptions(options);
  keypose_graph_.SetOptions(options);
  local_planner_.SetOptions(options);
  momentum_planner_.SetOptions(options);
  visualizer_.SetOptions(options);
  terrain_map_.SetOptions(options);
  viewpoints_.SetTerrainMap(&terrain_map_);
}

void HierarchicalPlanner::LoadBoundaries(
    const std::vector<std::string>& config_directories) {
  if (!options_.boundary_ply_path().empty()) {
    automsgs::msgs::geometry_msgs::Polygon boundary;
    if (BoundaryLoader::LoadPolygonFromConfigPath(
            options_.boundary_ply_path(), config_directories, &boundary)) {
      env_.SetExplorationArea(boundary);
    }
  }
  if (!options_.nogo_ply_path().empty()) {
    automsgs::msgs::geometry_msgs::Polygon nogo;
    if (BoundaryLoader::LoadPolygonFromConfigPath(options_.nogo_ply_path(),
                                                  config_directories, &nogo)) {
      env_.SetNogoArea(nogo);
    }
  }
}

void HierarchicalPlanner::UpdateOdometry(
    const automsgs::msgs::nav_msgs::Odometry& odom) {
  env_.UpdateOdometry(odom);
  robot_z_ = odom.pose().pose().pose().position().z();
  frame_id_ = odom.header().frame_id().empty() ? frame_id_
                                               : odom.header().frame_id();
  const double x = odom.pose().pose().pose().position().x();
  const double y = odom.pose().pose().pose().position().y();
  const double yaw = YawFromOdom(odom);
  if (!has_initial_) {
    has_initial_ = true;
    initial_x_ = x;
    initial_y_ = y;
    grid_world_.SetOrigin(x, y);
  }
  keypose_graph_.UpdateRobotPose(x, y, yaw, env_);
}

void HierarchicalPlanner::UpdateDepth(
    const automsgs::msgs::sensor_msgs::Image& depth,
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    const automsgs::msgs::geometry_msgs::Transform& map_t_camera) {
  env_.UpdateDepth(depth, info, map_t_camera);
  cloud_manager_.MarkCovered(env_.robot_x(), env_.robot_y(), robot_z_,
                             options_.sensor_range_m());
  cloud_manager_.RefreshUncovered(env_);
}

void HierarchicalPlanner::UpdatePointCloud(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud) {
  has_point_cloud_ = true;
  cloud_manager_.Update(cloud, env_.robot_x(), env_.robot_y(), robot_z_);
  env_.UpdatePointCloud(cloud);
}

void HierarchicalPlanner::UpdateTerrainMap(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud) {
  terrain_map_.UpdateFromPointCloud(cloud, env_.robot_x(), env_.robot_y(),
                                    robot_z_);
}

bool HierarchicalPlanner::ExecutePlanningCycle() {
  if (options_.use_lidar_primary() && !has_point_cloud_) {
    return false;
  }
  const double robot_x = env_.robot_x();
  const double robot_y = env_.robot_y();

  viewpoints_.Update(env_, &cloud_manager_, robot_x, robot_y, robot_z_);
  const int max_vp = std::max(options_.viewpoint_yaw_samples(), 1) * 3;
  viewpoints_.SelectTopViewpoints(max_vp);
  grid_world_.UpdateCellStatus(viewpoints_, env_);

  global_cell_order_ = SolveGlobalCellOrder();
  BuildCellRoadmap(global_cell_order_);

  automsgs::msgs::nav_msgs::Path candidate =
      local_planner_.Solve(env_, viewpoints_, robot_x, robot_y,
                           GlobalTargets());
  candidate.mutable_header()->set_frame_id(frame_id_);

  if (momentum_planner_.ShouldKeepPreviousPath(
          previous_path_, candidate, robot_x, robot_y, env_.robot_yaw())) {
    path_ = previous_path_;
  } else {
    path_ = candidate;
    momentum_planner_.Reset();
  }
  momentum_planner_.OnPathAccepted();
  previous_path_ = path_;

  exploration_path_.Clear();
  exploration_path_.AppendPath(path_, ExplorationPath::NodeType::kLocalViewpoint);

  waypoint_index_ = 0;
  has_target_ = path_.poses_size() > 0;
  lookahead_ = momentum_planner_.SmoothWaypoint(
      ComputeLookahead(path_), lookahead_, robot_x, robot_y, env_.robot_yaw());

  const float progress = Progress();
  finished_ = progress >= options_.completion_coverage_ratio() &&
              env_.FrontierCellCount() == 0;
  if (finished_ && options_.rush_home() && has_initial_) {
    const double home_dist =
        std::hypot(robot_x - initial_x_, robot_y - initial_y_);
    if (home_dist > options_.at_home_dist_m()) {
      finished_ = false;
      automsgs::msgs::nav_msgs::Path home_path;
      home_path.mutable_header()->set_frame_id(frame_id_);
      std::vector<automsgs::msgs::geometry_msgs::Point> segment;
      if (!keypose_graph_.QueryPathToFirstKeypose(env_, robot_x, robot_y,
                                                  &segment)) {
        PathPlanner::Plan(env_, robot_x, robot_y, initial_x_, initial_y_,
                          &segment);
      }
      for (const auto& pt : segment) {
        *home_path.add_poses() = MakePose(pt.x(), pt.y(), robot_z_, 0.0);
      }
      if (home_path.poses_size() > 0) {
        path_ = home_path;
        has_target_ = true;
        lookahead_ = ComputeLookahead(path_);
      }
    }
  }
  return has_target_;
}

float HierarchicalPlanner::Progress() const {
  return grid_world_.CoverageProgress();
}

float HierarchicalPlanner::ExploredAreaM2() const {
  const double res = env_.costmap().getResolution();
  int covered = 0;
  const unsigned int w = env_.costmap().getSizeInCellsX();
  const unsigned int h = env_.costmap().getSizeInCellsY();
  for (unsigned int y = 0; y < h; ++y) {
    for (unsigned int x = 0; x < w; ++x) {
      double wx = 0.0;
      double wy = 0.0;
      env_.costmap().mapToWorld(x, y, wx, wy);
      if (env_.IsCovered(wx, wy)) {
        ++covered;
      }
    }
  }
  return static_cast<float>(covered) * static_cast<float>(res * res);
}

void HierarchicalPlanner::AdvanceWaypointIndex() {
  if (path_.poses_size() == 0) {
    return;
  }
  if (waypoint_index_ + 1 < static_cast<size_t>(path_.poses_size())) {
    ++waypoint_index_;
  }
  lookahead_ = ComputeLookahead(path_);
}

std::vector<int> HierarchicalPlanner::SolveGlobalCellOrder() {
  const double robot_x = env_.robot_x();
  const double robot_y = env_.robot_y();
  std::vector<int> cells = grid_world_.GetExploringCells(
      robot_x, robot_y, options_.grid_nearby_radius());
  if (cells.empty()) {
    return {};
  }

  std::vector<automsgs::msgs::geometry_msgs::Point> centers;
  centers.reserve(cells.size() + 1);
  automsgs::msgs::geometry_msgs::Point robot;
  robot.set_x(robot_x);
  robot.set_y(robot_y);
  centers.push_back(robot);
  for (int cell : cells) {
    centers.push_back(grid_world_.CellCenter(cell));
  }

  const int n = static_cast<int>(centers.size());
  TspSolver::DataModel model;
  model.depot = 0;
  model.distance_matrix.assign(static_cast<size_t>(n),
                               std::vector<int>(static_cast<size_t>(n), 0));
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) {
      if (i == j) {
        continue;
      }
      const double len = keypose_graph_.QueryDistance(
          env_, centers[static_cast<size_t>(i)].x(),
          centers[static_cast<size_t>(i)].y(),
          centers[static_cast<size_t>(j)].x(),
          centers[static_cast<size_t>(j)].y());
      model.distance_matrix[static_cast<size_t>(i)]
                           [static_cast<size_t>(j)] =
          std::isfinite(len)
              ? static_cast<int>(std::lround(len * 100.0))
              : TspSolver::kInfCost;
    }
  }

  TspSolver solver(std::move(model), options_.tsp_exact_max_nodes());
  TspSolver::SolveOptions tsp_opts;
  tsp_opts.max_exact_nodes = options_.tsp_exact_max_nodes();
  tsp_opts.use_two_opt = options_.tsp_use_two_opt();
  tsp_opts.use_ortools = options_.tsp_use_ortools();
  const std::vector<int> order = solver.Solve(tsp_opts);
  std::vector<int> cell_order;
  cell_order.reserve(order.size());
  for (int idx : order) {
    if (idx <= 0) {
      continue;
    }
    cell_order.push_back(cells[static_cast<size_t>(idx - 1)]);
  }
  return cell_order;
}

void HierarchicalPlanner::BuildCellRoadmap(const std::vector<int>& order) {
  global_debug_path_.Clear();
  global_debug_path_.mutable_header()->set_frame_id(frame_id_);
  if (order.empty()) {
    return;
  }
  if (order.size() == 1) {
    const auto center = grid_world_.CellCenter(order.front());
    *global_debug_path_.add_poses() =
        MakePose(center.x(), center.y(), robot_z_, 0.0);
    return;
  }
  int prev = order.front();
  for (size_t i = 1; i < order.size(); ++i) {
    const int cur = order[i];
    const auto from = grid_world_.CellCenter(prev);
    const auto to = grid_world_.CellCenter(cur);
    std::vector<automsgs::msgs::geometry_msgs::Point> segment;
    keypose_graph_.QueryPath(env_, from.x(), from.y(), to.x(), to.y(),
                             &segment);
    grid_world_.StoreCellPath(prev, cur, segment);
    for (size_t j = 0; j < segment.size(); ++j) {
      if (i > 1 && j == 0) {
        continue;
      }
      *global_debug_path_.add_poses() =
          MakePose(segment[j].x(), segment[j].y(), robot_z_, 0.0);
    }
    prev = cur;
  }
}

automsgs::msgs::geometry_msgs::PoseStamped
HierarchicalPlanner::ComputeLookahead(
    const automsgs::msgs::nav_msgs::Path& path) const {
  if (path.poses_size() == 0) {
    return MakePose(env_.robot_x(), env_.robot_y(), robot_z_, env_.robot_yaw());
  }

  const double robot_x = env_.robot_x();
  const double robot_y = env_.robot_y();
  const double min_dist = options_.lookahead_distance_m();
  double accumulated = 0.0;
  automsgs::msgs::geometry_msgs::PoseStamped last = path.poses(0);
  last.mutable_header()->set_frame_id(frame_id_);

  for (int i = 0; i < path.poses_size(); ++i) {
    const auto& pose = path.poses(i);
    const double x = pose.pose().position().x();
    const double y = pose.pose().position().y();
    if (i == 0) {
      accumulated = std::hypot(x - robot_x, y - robot_y);
    } else {
      const auto& prev = path.poses(i - 1);
      accumulated += std::hypot(
          x - prev.pose().position().x(), y - prev.pose().position().y());
    }
    if (accumulated < min_dist) {
      continue;
    }
    if (options_.use_line_of_sight_lookahead() &&
        !LineOfSightChecker::HasLineOfSight(env_, robot_x, robot_y, x, y)) {
      continue;
    }
    automsgs::msgs::geometry_msgs::PoseStamped lookahead = pose;
    lookahead.mutable_header()->set_frame_id(frame_id_);
    return lookahead;
  }
  return last;
}

automsgs::msgs::geometry_msgs::PoseStamped HierarchicalPlanner::MakePose(
    double x, double y, double z, double yaw) const {
  automsgs::msgs::geometry_msgs::PoseStamped pose;
  pose.mutable_header()->set_frame_id(frame_id_);
  pose.mutable_pose()->mutable_position()->set_x(x);
  pose.mutable_pose()->mutable_position()->set_y(y);
  pose.mutable_pose()->mutable_position()->set_z(z);
  pose.mutable_pose()->mutable_orientation()->set_w(std::cos(yaw * 0.5));
  pose.mutable_pose()->mutable_orientation()->set_z(std::sin(yaw * 0.5));
  return pose;
}

std::vector<automsgs::msgs::geometry_msgs::Point>
HierarchicalPlanner::GlobalTargets() const {
  std::vector<automsgs::msgs::geometry_msgs::Point> targets;
  targets.reserve(global_cell_order_.size());
  for (int cell : global_cell_order_) {
    targets.push_back(grid_world_.CellCenter(cell));
  }
  return targets;
}

void HierarchicalPlanner::Reset() {
  path_.Clear();
  global_debug_path_.Clear();
  previous_path_.Clear();
  exploration_path_.Clear();
  global_cell_order_.clear();
  waypoint_index_ = 0;
  has_target_ = false;
  finished_ = false;
  has_initial_ = false;
  has_point_cloud_ = false;
  grid_world_.Reset();
  keypose_graph_.Reset();
  momentum_planner_.Reset();
}

}  // namespace autonomy::perception::exploration
