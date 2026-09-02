/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <string>
#include <vector>

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>

#include "autonomy/perception/exploration/common/planning_utilities.hpp"
#include "autonomy/perception/exploration/common/exploration_visualizer.hpp"
#include "autonomy/perception/exploration/tare/grid_world.hpp"
#include "autonomy/perception/exploration/tare/keypose_graph.hpp"
#include "autonomy/perception/exploration/tare/local_coverage_planner.hpp"
#include "autonomy/perception/exploration/common/planning_utilities.hpp"
#include "autonomy/perception/exploration/common/planning_env.hpp"
#include "autonomy/perception/exploration/common/point_cloud_manager.hpp"
#include "autonomy/perception/proto/exploration_options.pb.h"
#include "autonomy/perception/exploration/common/terrain_height_map.hpp"
#include "autonomy/perception/exploration/tare/viewpoint_manager.hpp"

namespace autonomy::perception::exploration {

// TARE hierarchical orchestrator: global GridWorld TSP + local coverage.
class HierarchicalPlanner {
 public:
  explicit HierarchicalPlanner(const proto::ExplorationOptions& options);

  void SetOptions(const proto::ExplorationOptions& options);
  void LoadBoundaries(const std::vector<std::string>& config_directories);

  PlanningEnv& env() { return env_; }
  const PlanningEnv& env() const { return env_; }

  void UpdateOdometry(const automsgs::msgs::nav_msgs::Odometry& odom);
  void UpdateDepth(
      const automsgs::msgs::sensor_msgs::Image& depth,
      const automsgs::msgs::sensor_msgs::CameraInfo& info,
      const automsgs::msgs::geometry_msgs::Transform& map_t_camera);
  void UpdatePointCloud(
      const automsgs::msgs::sensor_msgs::PointCloud2& cloud);
  void UpdateTerrainMap(
      const automsgs::msgs::sensor_msgs::PointCloud2& cloud);

  void Reset();

  bool ExecutePlanningCycle();
  bool HasTarget() const { return has_target_; }
  bool IsFinished() const { return finished_; }
  float Progress() const;
  float ExploredAreaM2() const;

  automsgs::msgs::geometry_msgs::PoseStamped GetLookahead() const {
    return lookahead_;
  }
  const automsgs::msgs::nav_msgs::Path& path() const { return path_; }
  const automsgs::msgs::nav_msgs::Path& global_debug_path() const {
    return global_debug_path_;
  }
  const ExplorationPath& exploration_path() const { return exploration_path_; }
  const GridWorld& grid_world() const { return grid_world_; }
  const KeyposeGraph& keypose_graph() const { return keypose_graph_; }
  void AdvanceWaypointIndex();

 private:
  std::vector<int> SolveGlobalCellOrder();
  void BuildCellRoadmap(const std::vector<int>& order);
  automsgs::msgs::geometry_msgs::PoseStamped ComputeLookahead(
      const automsgs::msgs::nav_msgs::Path& path) const;
  automsgs::msgs::geometry_msgs::PoseStamped MakePose(double x, double y,
                                                        double z,
                                                        double yaw) const;
  std::vector<automsgs::msgs::geometry_msgs::Point> GlobalTargets() const;
  double RobotZ() const { return robot_z_; }

  proto::ExplorationOptions options_;
  PlanningEnv env_;
  PointCloudManager cloud_manager_;
  ViewpointManager viewpoints_;
  GridWorld grid_world_;
  KeyposeGraph keypose_graph_;
  LocalCoveragePlanner local_planner_;
  MomentumPlanner momentum_planner_;
  ExplorationVisualizer visualizer_;
  TerrainHeightMap terrain_map_;

  ExplorationPath exploration_path_;
  automsgs::msgs::nav_msgs::Path path_;
  automsgs::msgs::nav_msgs::Path global_debug_path_;
  automsgs::msgs::nav_msgs::Path previous_path_;
  automsgs::msgs::geometry_msgs::PoseStamped lookahead_;
  std::vector<int> global_cell_order_;
  size_t waypoint_index_{0};
  bool has_target_{false};
  bool finished_{false};
  bool has_initial_{false};
  bool has_point_cloud_{false};
  double initial_x_{0.0};
  double initial_y_{0.0};
  double robot_z_{0.0};
  std::string frame_id_{"map"};
};

}  // namespace autonomy::perception::exploration
