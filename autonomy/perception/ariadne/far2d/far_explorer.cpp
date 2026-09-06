/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/perception/exploration/common/planning_utilities.hpp"
#include "autonomy/perception/exploration/core/exploration_options.hpp"
#include "autonomy/perception/exploration/far2d/far_explorer.hpp"
#include "autonomy/perception/exploration/common/planning_utilities.hpp"

#include <cmath>
#include <limits>

#include "autonomy/perception/exploration/common/planning_utilities.hpp"

namespace autonomy::perception::exploration {
namespace {

double YawFromOdom(const automsgs::msgs::nav_msgs::Odometry& odom) {
  const auto& q = odom.pose().pose().pose().orientation();
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

}  // namespace

FarExplorer::FarExplorer() : FarExplorer(DefaultOptions()) {}

FarExplorer::FarExplorer(const proto::ExplorationOptions& options)
    : env_(options),
      scan_handler_(options),
      visibility_graph_(options),
      momentum_planner_(options) {
  Configure(options);
}

void FarExplorer::Configure(const proto::ExplorationOptions& options) {
  options_ = options;
  frame_id_ = options.map_frame().empty() ? "map" : options.map_frame();
  env_.SetOptions(options);
  visibility_graph_.SetOptions(options);
  momentum_planner_.SetOptions(options);
  scan_handler_.SetOptions(options);
}

void FarExplorer::LoadBoundaries(
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

void FarExplorer::UpdateOdometry(
    const automsgs::msgs::nav_msgs::Odometry& odom) {
  env_.UpdateOdometry(odom);
  if (!odom.header().frame_id().empty()) {
    frame_id_ = odom.header().frame_id();
  }
  if (options_.far_use_trajectory_edges()) {
    const double x = env_.robot_x();
    const double y = env_.robot_y();
    const int max_nodes = std::max(options_.far_trajectory_node_count(), 8);
    if (trajectory_.empty() ||
        std::hypot(x - trajectory_.back().first,
                   y - trajectory_.back().second) >=
            options_.keypose_spacing_m()) {
      trajectory_.emplace_back(x, y);
      while (static_cast<int>(trajectory_.size()) > max_nodes) {
        trajectory_.pop_front();
      }
    }
  }
}

void FarExplorer::UpdateDepth(
    const automsgs::msgs::sensor_msgs::Image& depth,
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    const automsgs::msgs::geometry_msgs::Transform& map_t_camera) {
  env_.UpdateDepth(depth, info, map_t_camera);
}

void FarExplorer::UpdatePointCloud(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud) {
  if (!options_.far_is_static_env()) {
    scan_handler_.ResetFrame();
    scan_handler_.MarkScan(cloud, env_.robot_x(), env_.robot_y(), env_.robot_z());
    std::vector<double> dx;
    std::vector<double> dy;
    std::vector<double> dz;
    scan_handler_.ExtractDynamicCloud(&dx, &dy, &dz);
    const double radius = options_.far3d_voxel_dim_m() > 0
                              ? options_.far3d_voxel_dim_m()
                              : 0.25;
    for (size_t i = 0; i < dx.size(); ++i) {
      env_.ClearObstacleNear(dx[i], dy[i], radius);
    }
  }
  env_.UpdatePointCloud(cloud);
}

void FarExplorer::UpdatePriorMap(
    const automsgs::msgs::map_msgs::OccupancyGrid& map) {
  env_.UpdatePriorMap(map);
}

void FarExplorer::UpdatePlannerCostmap(
    const automsgs::msgs::map_msgs::OccupancyGrid& map) {
  env_.UpdatePlannerCostmap(map);
}

void FarExplorer::Reset() {
  visibility_graph_.Reset();
  momentum_planner_.Reset();
  path_.Clear();
  previous_path_.Clear();
  visibility_debug_path_.Clear();
  trajectory_.clear();
  waypoint_index_ = 0;
  has_target_ = false;
  finished_ = false;
  paused_ = false;
}

bool FarExplorer::ExecutePlanningCycle() {
  if (paused_) {
    return false;
  }
  const std::vector<std::pair<double, double>> trajectory(trajectory_.begin(),
                                                           trajectory_.end());
  visibility_graph_.Update(env_, &trajectory);
  visibility_debug_path_ = visibility_graph_.ToDebugPath(frame_id_);
  path_.clear_poses();
  path_.mutable_header()->set_frame_id(frame_id_);
  has_target_ = false;
  finished_ = false;

  if (env_.FrontierCellCount() == 0) {
    finished_ = Progress() >= options_.completion_coverage_ratio();
    return false;
  }

  const int robot_id = visibility_graph_.robot_node_id();
  const int goal_id = SelectGoalNode();
  if (robot_id < 0 || goal_id < 0) {
    if (options_.far_attempt_unknown() && !env_.frontiers().empty()) {
      const auto& frontier = env_.frontiers().front();
      *path_.add_poses() = MakePose(frontier.x(), frontier.y(), 0.0);
      waypoint_index_ = 0;
      has_target_ = true;
      lookahead_ = ComputeLookahead();
      return true;
    }
    return false;
  }

  std::vector<automsgs::msgs::geometry_msgs::Point> segment;
  if (!VisibilityPlanner::Plan(visibility_graph_, robot_id, goal_id,
                               &segment)) {
    return false;
  }

  double yaw = env_.robot_yaw();
  for (size_t i = 0; i < segment.size(); ++i) {
    if (i + 1 < segment.size()) {
      yaw = std::atan2(segment[i + 1].y() - segment[i].y(),
                       segment[i + 1].x() - segment[i].x());
    }
    *path_.add_poses() = MakePose(segment[i].x(), segment[i].y(), yaw);
  }
  has_target_ = path_.poses_size() > 0;
  if (momentum_planner_.ShouldKeepPreviousPath(
          previous_path_, path_, env_.robot_x(), env_.robot_y(),
          env_.robot_yaw())) {
    path_ = previous_path_;
  } else {
    momentum_planner_.Reset();
    waypoint_index_ = 0;
  }
  momentum_planner_.OnPathAccepted();
  previous_path_ = path_;
  lookahead_ = momentum_planner_.SmoothWaypoint(
      ComputeLookahead(), lookahead_, env_.robot_x(), env_.robot_y(),
      env_.robot_yaw());
  return has_target_;
}

int FarExplorer::SelectGoalNode() const {
  const auto& nodes = visibility_graph_.nodes();
  const int robot_id = visibility_graph_.robot_node_id();
  if (robot_id < 0) {
    return -1;
  }

  const double robot_x = env_.robot_x();
  const double robot_y = env_.robot_y();
  int best_id = -1;
  double best_score = -1.0;

  for (const auto& node : nodes) {
    if (!node.is_frontier || node.is_merged) {
      continue;
    }
    std::vector<automsgs::msgs::geometry_msgs::Point> probe;
    if (!VisibilityPlanner::Plan(visibility_graph_, robot_id, node.id,
                                 &probe)) {
      if (!options_.far_attempt_unknown()) {
        continue;
      }
    }
    const double score = ScoreFarGoal(
        env_, options_, node.x, node.y, true, node.frontier_cluster_size,
        robot_x, robot_y);
    if (score > best_score) {
      best_score = score;
      best_id = node.id;
    }
  }
  return best_id;
}

automsgs::msgs::geometry_msgs::PoseStamped FarExplorer::ComputeLookahead()
    const {
  if (path_.poses_size() == 0) {
    return MakePose(env_.robot_x(), env_.robot_y(), env_.robot_yaw());
  }

  const double robot_x = env_.robot_x();
  const double robot_y = env_.robot_y();
  const double min_dist = options_.lookahead_distance_m();
  double accumulated = 0.0;
  const int start = static_cast<int>(waypoint_index_);

  for (int i = start; i < path_.poses_size(); ++i) {
    const auto& pose = path_.poses(i);
    const double x = pose.pose().position().x();
    const double y = pose.pose().position().y();
    if (i == start) {
      accumulated = std::hypot(x - robot_x, y - robot_y);
    } else {
      const auto& prev = path_.poses(i - 1);
      accumulated += std::hypot(x - prev.pose().position().x(),
                                y - prev.pose().position().y());
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
  automsgs::msgs::geometry_msgs::PoseStamped last = path_.poses(
      path_.poses_size() - 1);
  last.mutable_header()->set_frame_id(frame_id_);
  return last;
}

automsgs::msgs::geometry_msgs::PoseStamped FarExplorer::MakePose(
    double x, double y, double yaw) const {
  automsgs::msgs::geometry_msgs::PoseStamped pose;
  pose.mutable_header()->set_frame_id(frame_id_);
  pose.mutable_pose()->mutable_position()->set_x(x);
  pose.mutable_pose()->mutable_position()->set_y(y);
  pose.mutable_pose()->mutable_orientation()->set_w(std::cos(yaw * 0.5));
  pose.mutable_pose()->mutable_orientation()->set_z(std::sin(yaw * 0.5));
  return pose;
}

bool FarExplorer::HasTarget() const { return has_target_; }

bool FarExplorer::GetNextWaypoint(
    automsgs::msgs::geometry_msgs::PoseStamped& waypoint) const {
  if (!has_target_) {
    return false;
  }
  waypoint = lookahead_;
  return true;
}

void FarExplorer::MarkWaypointReached() {
  if (!has_target_ || path_.poses_size() == 0) {
    has_target_ = false;
    return;
  }
  if (waypoint_index_ + 1 >= static_cast<std::size_t>(path_.poses_size())) {
    has_target_ = false;
    waypoint_index_ = 0;
    momentum_planner_.Reset();
    return;
  }
  ++waypoint_index_;
  momentum_planner_.Reset();
  lookahead_ = momentum_planner_.SmoothWaypoint(
      ComputeLookahead(), lookahead_, env_.robot_x(), env_.robot_y(),
      env_.robot_yaw());
}

bool FarExplorer::IsFinished() const { return finished_; }

float FarExplorer::Progress() const {
  const double res = env_.costmap().getResolution();
  int covered = 0;
  int total = 0;
  const unsigned int w = env_.costmap().getSizeInCellsX();
  const unsigned int h = env_.costmap().getSizeInCellsY();
  for (unsigned int y = 0; y < h; ++y) {
    for (unsigned int x = 0; x < w; ++x) {
      double wx = 0.0;
      double wy = 0.0;
      env_.costmap().mapToWorld(x, y, wx, wy);
      if (!env_.IsInExplorationArea(wx, wy)) {
        continue;
      }
      ++total;
      if (env_.IsCovered(wx, wy)) {
        ++covered;
      }
    }
  }
  (void)res;
  return total > 0 ? static_cast<float>(covered) / static_cast<float>(total)
                   : 0.f;
}

float FarExplorer::ExploredAreaM2() const {
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

automsgs::msgs::nav_msgs::Path FarExplorer::GetExplorationPath() const {
  return path_;
}

automsgs::msgs::map_msgs::OccupancyGrid FarExplorer::GetOccupancyGrid(
    const std::string& frame_id) const {
  const std::string id =
      frame_id.empty() ? options_.map_frame() : frame_id;
  return env_.GetOccupancyGridWithOverlay(id, true);
}

automsgs::msgs::nav_msgs::Path FarExplorer::GetGlobalDebugPath() const {
  return visibility_debug_path_;
}

automsgs::msgs::nav_msgs::Path FarExplorer::GetLocalDebugPath() const {
  return path_;
}

bool FarExplorer::GetNavigationBoundary(
    automsgs::msgs::geometry_msgs::Polygon* boundary) const {
  if (boundary == nullptr || !env_.HasExplorationBoundary()) {
    return false;
  }
  *boundary = env_.GetExplorationBoundary();
  return true;
}

automsgs::msgs::visualization_msgs::MarkerArray
FarExplorer::GetVisibilityGraphMarkers(const std::string& frame_id) const {
  if (!options_.publish_vg_markers()) {
    return {};
  }
  const std::string id = frame_id.empty() ? frame_id_ : frame_id;
  return visibility_graph_.ToMarkerArray(id);
}

}  // namespace autonomy::perception::exploration
