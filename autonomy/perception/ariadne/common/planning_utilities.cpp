/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/perception/exploration/common/planning_utilities.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/planning/common/costmap_navfn_planner.hpp"

namespace autonomy::perception::exploration {
namespace {

bool ParseAsciiPly(std::istream& input,
                   automsgs::msgs::geometry_msgs::Polygon* polygon) {
  if (polygon == nullptr) {
    return false;
  }
  polygon->clear_points();

  std::string line;
  if (!std::getline(input, line) || line != "ply") {
    return false;
  }
  if (!std::getline(input, line) || line != "format ascii 1.0") {
    return false;
  }

  int vertex_count = 0;
  bool in_header = true;
  while (in_header && std::getline(input, line)) {
    if (line.rfind("element vertex", 0) == 0) {
      std::istringstream iss(line);
      std::string element;
      std::string vertex;
      iss >> element >> vertex >> vertex_count;
      continue;
    }
    if (line == "end_header") {
      in_header = false;
      break;
    }
  }
  if (vertex_count <= 0) {
    return false;
  }

  for (int i = 0; i < vertex_count; ++i) {
    if (!std::getline(input, line)) {
      return false;
    }
    std::istringstream iss(line);
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    iss >> x >> y >> z;
    auto* pt = polygon->add_points();
    pt->set_x(x);
    pt->set_y(y);
    pt->set_z(z);
  }
  return polygon->points_size() >= 3;
}

}  // namespace

bool BoundaryLoader::LoadPolygonFromPly(
    const std::string& path,
    automsgs::msgs::geometry_msgs::Polygon* polygon) {
  if (polygon == nullptr) {
    return false;
  }
  std::ifstream input(path);
  if (!input.is_open()) {
    AWARN << "BoundaryLoader: cannot open " << path;
    return false;
  }
  if (!ParseAsciiPly(input, polygon)) {
    AWARN << "BoundaryLoader: failed to parse PLY " << path;
    polygon->clear_points();
    return false;
  }
  AINFO << "BoundaryLoader: loaded " << polygon->points_size()
        << " vertices from " << path;
  return true;
}

bool BoundaryLoader::LoadPolygonFromConfigPath(
    const std::string& relative_or_absolute_path,
    const std::vector<std::string>& config_directories,
    automsgs::msgs::geometry_msgs::Polygon* polygon) {
  if (relative_or_absolute_path.empty() || polygon == nullptr) {
    return false;
  }
  if (!relative_or_absolute_path.empty() &&
      relative_or_absolute_path.front() == '/') {
    return LoadPolygonFromPly(relative_or_absolute_path, polygon);
  }
  if (config_directories.empty()) {
    return LoadPolygonFromPly(relative_or_absolute_path, polygon);
  }
  try {
    ::autonomy::common::ConfigurationFileResolver resolver(config_directories);
    const std::string full_path =
        resolver.GetFullPathOrDie(relative_or_absolute_path);
    return LoadPolygonFromPly(full_path, polygon);
  } catch (const std::exception& ex) {
    AWARN << "BoundaryLoader: resolve failed for "
          << relative_or_absolute_path << ": " << ex.what();
    return false;
  }
}

bool LineOfSightChecker::HasLineOfSight(const PlanningEnv& env, double x0,
                                       double y0, double x1, double y1) {
  const auto& map = env.inflated_costmap();
  const double res = map.getResolution();
  const int steps = static_cast<int>(
      std::ceil(std::hypot(x1 - x0, y1 - y0) / std::max(res, 1e-3)));
  if (steps <= 0) {
    return true;
  }
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const double x = x0 + t * (x1 - x0);
    const double y = y0 + t * (y1 - y0);
    if (env.IsOccupied(x, y)) {
      return false;
    }
  }
  return true;
}

bool LineOfSightChecker::HasLineOfSightOnFreeSpace(const PlanningEnv& env,
                                                   double x0, double y0,
                                                   double x1, double y1) {
  const auto& map = env.inflated_costmap();
  const double res = map.getResolution();
  const int steps = static_cast<int>(
      std::ceil(std::hypot(x1 - x0, y1 - y0) / std::max(res, 1e-3)));
  if (steps <= 0) {
    return true;
  }
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    const double x = x0 + t * (x1 - x0);
    const double y = y0 + t * (y1 - y0);
    if (env.IsOccupied(x, y)) {
      return false;
    }
    if (!env.IsFree(x, y) && !env.IsCovered(x, y)) {
      return false;
    }
  }
  return true;
}

double PathPlanner::Plan(const PlanningEnv& env, double from_x, double from_y,
                         double to_x, double to_y,
                         std::vector<automsgs::msgs::geometry_msgs::Point>* path) {
  if (path == nullptr) {
    return std::numeric_limits<double>::infinity();
  }
  path->clear();
  planning::common::CostmapNavfnPlannerOptions options;
  options.allow_unknown = true;
  options.use_astar = true;
  options.tolerance_m = 0.25;
  return planning::common::CostmapNavfnPlanner::Plan(
      env.inflated_costmap(), from_x, from_y, to_x, to_y, path, options);
}

MomentumPlanner::MomentumPlanner(const proto::ExplorationOptions& options) {
  SetOptions(options);
}

void MomentumPlanner::SetOptions(const proto::ExplorationOptions& options) {
  options_ = options;
}

void MomentumPlanner::Reset() {
  path_momentum_counter_ = 0;
  recorded_path_.clear_poses();
}

bool MomentumPlanner::ShouldKeepPreviousPath(
    const automsgs::msgs::nav_msgs::Path& previous,
    const automsgs::msgs::nav_msgs::Path& candidate, double robot_x,
    double robot_y, double robot_yaw) const {
  if (!options_.use_momentum() || previous.poses_size() == 0 ||
      candidate.poses_size() == 0) {
    return false;
  }
  if (path_momentum_counter_ >= options_.momentum_thred()) {
    return false;
  }
  const double moved =
      std::hypot(robot_x - previous.poses(0).pose().position().x(),
                 robot_y - previous.poses(0).pose().position().y());
  if (moved < options_.momentum_dist_m()) {
    return true;
  }
  const auto& cand = candidate.poses(0);
  const auto& prev = previous.poses(0);
  const double cand_yaw = std::atan2(
      2.0 * cand.pose().orientation().w() * cand.pose().orientation().z(),
      1.0 - 2.0 * cand.pose().orientation().z() * cand.pose().orientation().z());
  const double prev_yaw = std::atan2(
      2.0 * prev.pose().orientation().w() * prev.pose().orientation().z(),
      1.0 - 2.0 * prev.pose().orientation().z() * prev.pose().orientation().z());
  const double heading_dot =
      std::cos(cand_yaw - robot_yaw) * std::cos(prev_yaw - robot_yaw) +
      std::sin(cand_yaw - robot_yaw) * std::sin(prev_yaw - robot_yaw);
  return heading_dot < 0.0;
}

automsgs::msgs::geometry_msgs::PoseStamped MomentumPlanner::SmoothWaypoint(
    const automsgs::msgs::geometry_msgs::PoseStamped& candidate,
    const automsgs::msgs::geometry_msgs::PoseStamped& previous, double /*robot_x*/,
    double /*robot_y*/, double /*robot_yaw*/) const {
  if (!options_.use_momentum()) {
    return candidate;
  }
  automsgs::msgs::geometry_msgs::PoseStamped out = candidate;
  const double blend = 0.35;
  out.mutable_pose()->mutable_position()->set_x(
      blend * previous.pose().position().x() +
      (1.0 - blend) * candidate.pose().position().x());
  out.mutable_pose()->mutable_position()->set_y(
      blend * previous.pose().position().y() +
      (1.0 - blend) * candidate.pose().position().y());
  return out;
}

void MomentumPlanner::OnPathAccepted() {
  ++path_momentum_counter_;
}

void ExplorationPath::Clear() { nodes_.clear(); }

void ExplorationPath::Append(const Node& node) { nodes_.push_back(node); }

void ExplorationPath::AppendPath(const automsgs::msgs::nav_msgs::Path& path,
                                 NodeType type) {
  for (const auto& pose : path.poses()) {
    Node node;
    node.type = type;
    node.position.set_x(pose.pose().position().x());
    node.position.set_y(pose.pose().position().y());
    node.position.set_z(pose.pose().position().z());
    const auto& q = pose.pose().orientation();
    node.yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                          1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    nodes_.push_back(node);
  }
}

double ExplorationPath::Length() const {
  double len = 0.0;
  for (size_t i = 1; i < nodes_.size(); ++i) {
    len += std::hypot(nodes_[i].position.x() - nodes_[i - 1].position.x(),
                      nodes_[i].position.y() - nodes_[i - 1].position.y());
  }
  return len;
}

automsgs::msgs::nav_msgs::Path ExplorationPath::ToNavPath(
    const std::string& frame_id) const {
  automsgs::msgs::nav_msgs::Path path;
  path.mutable_header()->set_frame_id(frame_id);
  for (const auto& node : nodes_) {
    auto* pose = path.add_poses();
    pose->mutable_header()->set_frame_id(frame_id);
    pose->mutable_pose()->mutable_position()->set_x(node.position.x());
    pose->mutable_pose()->mutable_position()->set_y(node.position.y());
    pose->mutable_pose()->mutable_position()->set_z(node.position.z());
    pose->mutable_pose()->mutable_orientation()->set_w(std::cos(node.yaw * 0.5));
    pose->mutable_pose()->mutable_orientation()->set_z(std::sin(node.yaw * 0.5));
  }
  return path;
}

int FindFrontierClusterSize(const PlanningEnv& env, double x, double y) {
  const auto& frontiers = env.frontiers();
  const auto& sizes = env.frontier_cluster_sizes();
  if (frontiers.size() != sizes.size()) {
    return 1;
  }
  int best_size = 1;
  double best_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < frontiers.size(); ++i) {
    const double d = std::hypot(frontiers[i].x() - x, frontiers[i].y() - y);
    if (d < best_dist) {
      best_dist = d;
      best_size = sizes[i];
    }
  }
  return best_size;
}

double ScoreFarGoal(const PlanningEnv& env,
                    const proto::ExplorationOptions& options, double goal_x,
                    double goal_y, bool is_frontier, int frontier_cluster_size,
                    double robot_x, double robot_y) {
  const double dist = std::hypot(goal_x - robot_x, goal_y - robot_y);
  const double dist_w = options.far_goal_distance_weight() > 0
                            ? options.far_goal_distance_weight()
                            : 1.0;
  const double unknown_w = options.far_goal_unknown_weight() > 0
                               ? options.far_goal_unknown_weight()
                               : 0.05;
  double score = dist_w * dist;
  if (is_frontier) {
    const int cluster = frontier_cluster_size > 0
                            ? frontier_cluster_size
                            : FindFrontierClusterSize(env, goal_x, goal_y);
    score += unknown_w * static_cast<double>(cluster);
  }
  return score;
}

}  // namespace autonomy::perception::exploration
