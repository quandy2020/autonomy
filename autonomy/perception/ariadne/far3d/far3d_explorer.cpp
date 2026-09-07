/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/perception/exploration/far3d/far3d_explorer.hpp"

#include <cmath>

#include "autonomy/perception/exploration/common/planning_utilities.hpp"
#include "autonomy/perception/exploration/core/exploration_options.hpp"
#include "autonomy/perception/exploration/common/planning_utilities.hpp"
#include "autonomy/perception/exploration/far3d/visibility_graph3.hpp"
#include "autonomy/perception/exploration/common/planning_utilities.hpp"

namespace autonomy::perception::exploration::far3d {

Far3dExplorer::Far3dExplorer() : Far3dExplorer(DefaultOptions()) {}

Far3dExplorer::Far3dExplorer(const proto::ExplorationOptions& options)
    : env_(options),
      contour_detector_(options),
      contour_graph_(options),
      visibility_graph_(options),
      map_handler_(options),
      scan_handler_(options),
      terrain_planner_(options),
      viewpoint_extension_(options),
      momentum_planner_(options) {
  Configure(options);
}

void Far3dExplorer::Configure(const proto::ExplorationOptions& options) {
  options_ = options;
  frame_id_ = options.map_frame().empty() ? "map" : options.map_frame();
  env_.SetOptions(options);
  contour_detector_.SetOptions(options);
  contour_graph_.SetOptions(options);
  visibility_graph_.SetOptions(options);
  map_handler_.SetOptions(options);
  scan_handler_.SetOptions(options);
  terrain_planner_.SetOptions(options);
  viewpoint_extension_.SetOptions(options);
  momentum_planner_.SetOptions(options);
}

void Far3dExplorer::LoadBoundaries(
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
  if (!options_.boundary_vgh_path().empty()) {
    std::vector<VghNode> nodes;
    if (VghLoader::LoadFromConfigPath(options_.boundary_vgh_path(),
                                      config_directories, &nodes)) {
      visibility_graph_.ImportStaticGraph(nodes);
    }
  }
}

void Far3dExplorer::UpdateOdometry(
    const automsgs::msgs::nav_msgs::Odometry& odom) {
  env_.UpdateOdometry(odom);
  robot_z_ = odom.pose().pose().pose().position().z();
  if (!odom.header().frame_id().empty()) {
    frame_id_ = odom.header().frame_id();
  }
}

void Far3dExplorer::UpdateDepth(
    const automsgs::msgs::sensor_msgs::Image& depth,
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    const automsgs::msgs::geometry_msgs::Transform& map_t_camera) {
  env_.UpdateDepth(depth, info, map_t_camera);
}

void Far3dExplorer::UpdatePointCloud(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud) {
  latest_cloud_ = cloud;
  map_handler_.InsertObsCloud(cloud);
  if (!options_.far_is_static_env()) {
    scan_handler_.ResetFrame();
    scan_handler_.MarkScan(cloud, env_.robot_x(), env_.robot_y(), robot_z_);
    std::vector<double> dx;
    std::vector<double> dy;
    std::vector<double> dz;
    scan_handler_.ExtractDynamicCloud(&dx, &dy, &dz);
    for (size_t i = 0; i < dx.size(); ++i) {
      map_handler_.RemoveObsNear(dx[i], dy[i], dz[i], options_.far3d_voxel_dim_m());
    }
  }
  terrain_planner_.SetCenter(env_.robot_x(), env_.robot_y());
  terrain_planner_.SetObstacles(map_handler_);
  env_.UpdatePointCloud(cloud);
}

void Far3dExplorer::UpdatePriorMap(
    const automsgs::msgs::map_msgs::OccupancyGrid& map) {
  env_.UpdatePriorMap(map);
}

void Far3dExplorer::UpdatePlannerCostmap(
    const automsgs::msgs::map_msgs::OccupancyGrid& map) {
  env_.UpdatePlannerCostmap(map);
}

void Far3dExplorer::Reset() {
  visibility_graph_.Reset();
  momentum_planner_.Reset();
  path_.Clear();
  previous_path_.Clear();
  visibility_debug_path_.Clear();
  latest_cloud_.reset();
  waypoint_index_ = 0;
  has_target_ = false;
  finished_ = false;
  paused_ = false;
}

bool Far3dExplorer::ExecutePlanningCycle() {
  if (paused_) {
    return false;
  }
  path_.clear_poses();
  path_.mutable_header()->set_frame_id(frame_id_);
  has_target_ = false;
  finished_ = false;

  if (!latest_cloud_.has_value()) {
    return false;
  }

  const auto polygons = contour_detector_.Detect(
      latest_cloud_.value(), env_.robot_x(), env_.robot_y(), robot_z_);
  contour_graph_.UpdatePolygons(polygons);
  visibility_graph_.Update(env_, contour_graph_, polygons, robot_z_);
  visibility_debug_path_.Clear();
  visibility_debug_path_.mutable_header()->set_frame_id(frame_id_);
  for (const auto& entry : visibility_graph_.nodes()) {
    const NavNode3& node = entry.second;
    for (std::size_t neighbor_id : node.connect_ids) {
      if (entry.first >= neighbor_id) {
        continue;
      }
      const auto neighbor_it = visibility_graph_.nodes().find(neighbor_id);
      if (neighbor_it == visibility_graph_.nodes().end()) {
        continue;
      }
      const NavNode3& neighbor = neighbor_it->second;
      automsgs::msgs::geometry_msgs::PoseStamped pose_a;
      pose_a.mutable_header()->set_frame_id(frame_id_);
      pose_a.mutable_pose()->mutable_position()->set_x(node.x);
      pose_a.mutable_pose()->mutable_position()->set_y(node.y);
      pose_a.mutable_pose()->mutable_position()->set_z(node.z);
      *visibility_debug_path_.add_poses() = pose_a;
      automsgs::msgs::geometry_msgs::PoseStamped pose_b;
      pose_b.mutable_header()->set_frame_id(frame_id_);
      pose_b.mutable_pose()->mutable_position()->set_x(neighbor.x);
      pose_b.mutable_pose()->mutable_position()->set_y(neighbor.y);
      pose_b.mutable_pose()->mutable_position()->set_z(neighbor.z);
      *visibility_debug_path_.add_poses() = pose_b;
    }
  }

  if (env_.FrontierCellCount() == 0) {
    finished_ = Progress() >= options_.completion_coverage_ratio();
    return false;
  }

  const std::size_t goal_id = SelectGoalNode();
  const std::size_t start_id = visibility_graph_.odom_node_id();
  if (goal_id == static_cast<std::size_t>(-1) ||
      (start_id == 0 && goal_id == 0)) {
    if (options_.far_attempt_unknown() && !env_.frontiers().empty()) {
      const auto& f = env_.frontiers().front();
      *path_.add_poses() = MakePose(f.x(), f.y(), robot_z_, 0.0);
      waypoint_index_ = 0;
      has_target_ = true;
      lookahead_ = ComputeLookahead();
      return true;
    }
    return false;
  }

  std::vector<automsgs::msgs::geometry_msgs::Point> segment;
  const GraphPlanResult plan_result = GraphPlanner3::Plan(
      visibility_graph_, env_, start_id, goal_id, true,
      options_.far_is_autoswitch());
  last_nav_mode_ = plan_result.mode;
  segment = plan_result.path;
  if (segment.empty()) {
    return false;
  }

  double yaw = env_.robot_yaw();
  for (size_t i = 0; i < segment.size(); ++i) {
    if (i + 1 < segment.size()) {
      yaw = std::atan2(segment[i + 1].y() - segment[i].y(),
                       segment[i + 1].x() - segment[i].x());
    }
    *path_.add_poses() =
        MakePose(segment[i].x(), segment[i].y(), segment[i].z(), yaw);
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
  if (!contour_graph_.ct_nodes().empty() && options_.far_viewpoint_extend()) {
    for (const auto& ct : contour_graph_.ct_nodes()) {
      if (ct.convexity == ContourConvexity::kConvex) {
        lookahead_ = viewpoint_extension_.Extend(lookahead_, ct, map_handler_);
        break;
      }
    }
  }
  return has_target_;
}

std::size_t Far3dExplorer::SelectGoalNode() const {
  const auto& nodes = visibility_graph_.nodes();
  const std::size_t robot_id = visibility_graph_.odom_node_id();
  double best_score = -1.0;
  std::size_t best_id = static_cast<std::size_t>(-1);

  for (const auto& entry : nodes) {
    const auto& node = entry.second;
    if (node.is_merged || !node.is_frontier) {
      continue;
    }
    std::vector<automsgs::msgs::geometry_msgs::Point> probe;
    if (!GraphPlanner3::Plan(visibility_graph_, robot_id, node.id, &probe)) {
      if (!options_.far_attempt_unknown()) {
        continue;
      }
    }
    const double dist =
        std::hypot(node.x - env_.robot_x(), node.y - env_.robot_y());
    const double score = ScoreFarGoal(env_, options_, node.x, node.y, true,
                                      FindFrontierClusterSize(env_, node.x,
                                                              node.y),
                                      env_.robot_x(), env_.robot_y());
    (void)dist;
    if (score > best_score) {
      best_score = score;
      best_id = node.id;
    }
  }
  return best_id;
}

automsgs::msgs::geometry_msgs::PoseStamped Far3dExplorer::ComputeLookahead()
    const {
  if (path_.poses_size() == 0) {
    return MakePose(env_.robot_x(), env_.robot_y(), robot_z_, env_.robot_yaw());
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

  automsgs::msgs::geometry_msgs::PoseStamped last =
      path_.poses(path_.poses_size() - 1);
  last.mutable_header()->set_frame_id(frame_id_);
  return last;
}

automsgs::msgs::geometry_msgs::PoseStamped Far3dExplorer::MakePose(
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

bool Far3dExplorer::HasTarget() const { return has_target_; }

bool Far3dExplorer::GetNextWaypoint(
    automsgs::msgs::geometry_msgs::PoseStamped& waypoint) const {
  if (!has_target_) {
    return false;
  }
  waypoint = lookahead_;
  return true;
}

void Far3dExplorer::MarkWaypointReached() {
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

bool Far3dExplorer::IsFinished() const { return finished_; }

float Far3dExplorer::Progress() const {
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
  return total > 0 ? static_cast<float>(covered) / static_cast<float>(total)
                   : 0.f;
}

float Far3dExplorer::ExploredAreaM2() const {
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

automsgs::msgs::nav_msgs::Path Far3dExplorer::GetExplorationPath() const {
  return path_;
}

automsgs::msgs::map_msgs::OccupancyGrid Far3dExplorer::GetOccupancyGrid(
    const std::string& frame_id) const {
  const std::string id =
      frame_id.empty() ? options_.map_frame() : frame_id;
  return env_.GetOccupancyGridWithOverlay(id, true);
}

automsgs::msgs::nav_msgs::Path Far3dExplorer::GetGlobalDebugPath() const {
  return visibility_debug_path_;
}

automsgs::msgs::nav_msgs::Path Far3dExplorer::GetLocalDebugPath() const {
  return path_;
}

bool Far3dExplorer::GetNavigationBoundary(
    automsgs::msgs::geometry_msgs::Polygon* boundary) const {
  if (boundary == nullptr || !env_.HasExplorationBoundary()) {
    return false;
  }
  *boundary = env_.GetExplorationBoundary();
  return true;
}

automsgs::msgs::visualization_msgs::MarkerArray
Far3dExplorer::GetVisibilityGraphMarkers(const std::string& frame_id) const {
  if (!options_.publish_vg_markers()) {
    return {};
  }
  const std::string id = frame_id.empty() ? frame_id_ : frame_id;
  return visibility_graph_.ToMarkerArray(id);
}

}  // namespace autonomy::perception::exploration::far3d
