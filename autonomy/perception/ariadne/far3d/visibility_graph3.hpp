/*
 * Copyright 2026 The Openbot Authors
 *
 * FAR3D contour detection, visibility graph, and graph planning.
 */

#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>

#include "autonomy/perception/exploration/common/planning_env.hpp"
#include "autonomy/perception/exploration/far3d/types.hpp"
#include "autonomy/perception/proto/exploration_options.pb.h"

namespace autonomy::perception::exploration::far3d {

struct VghNode {
  std::size_t id{0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  bool is_boundary{false};
  bool is_frontier{false};
  bool is_navpoint{false};
  std::vector<std::size_t> connect_ids;
  std::vector<std::size_t> contour_ids;
};

class VghLoader {
 public:
  static bool LoadFromFile(const std::string& path, std::vector<VghNode>* nodes);

  static bool LoadFromConfigPath(
      const std::string& relative_or_absolute_path,
      const std::vector<std::string>& config_directories,
      std::vector<VghNode>* nodes);
};

class ContourDetector3 {
 public:
  explicit ContourDetector3(const proto::ExplorationOptions& options);

  void SetOptions(const proto::ExplorationOptions& options);

  std::vector<ContourPolygon> Detect(
      const automsgs::msgs::sensor_msgs::PointCloud2& cloud, double robot_x,
      double robot_y, double robot_z) const;

 private:
  proto::ExplorationOptions options_;
};

class ContourGraph3 {
 public:
  explicit ContourGraph3(const proto::ExplorationOptions& options);

  void SetOptions(const proto::ExplorationOptions& options);
  void UpdatePolygons(std::vector<ContourPolygon> polygons);

  const std::vector<ContourPolygon>& polygons() const { return polygons_; }
  const std::vector<CtNode>& ct_nodes() const { return ct_nodes_; }

  bool IsSegmentInFreePolygon(
      const automsgs::msgs::geometry_msgs::Point& a,
      const automsgs::msgs::geometry_msgs::Point& b) const;

  bool ReprojectPointOutsidePolygons(double* x, double* y, double* z) const;

 private:
  void BuildCtNodes();
  static bool SegmentsIntersect2D(double ax, double ay, double bx, double by,
                                  double cx, double cy, double dx, double dy);

  proto::ExplorationOptions options_;
  std::vector<ContourPolygon> polygons_;
  std::vector<CtNode> ct_nodes_;
};

class DynamicVisibilityGraph {
 public:
  explicit DynamicVisibilityGraph(const proto::ExplorationOptions& options);

  void SetOptions(const proto::ExplorationOptions& options);
  void Reset();

  void ImportStaticGraph(const std::vector<VghNode>& static_nodes);

  void Update(const PlanningEnv& env, const ContourGraph3& contour_graph,
              const std::vector<ContourPolygon>& polygons, double robot_z);

  const std::unordered_map<std::size_t, NavNode3>& nodes() const {
    return nodes_;
  }
  std::size_t odom_node_id() const { return odom_node_id_; }

  automsgs::msgs::visualization_msgs::MarkerArray ToMarkerArray(
      const std::string& frame_id) const;

 private:
  std::size_t AddNode(double x, double y, double z, bool is_odom,
                      bool is_frontier);
  bool TryConnect(std::size_t a, std::size_t b, const ContourGraph3& graph,
                  const PlanningEnv& env);
  void ValidateEdges(const ContourGraph3& graph);
  void PruneNodes(double robot_x, double robot_y, double robot_z);
  bool HasLineOfSight(const PlanningEnv& env, const NavNode3& a,
                      const NavNode3& b) const;
  bool IsNearExisting(double x, double y, double z) const;
  void ConnectNodes(std::size_t a, std::size_t b);

  proto::ExplorationOptions options_;
  std::unordered_map<std::size_t, NavNode3> nodes_;
  std::size_t id_tracker_{0};
  std::size_t odom_node_id_{0};
};

enum class NavMode { kAttemptable, kFreeSpace };

struct GraphPlanResult {
  NavMode mode{NavMode::kAttemptable};
  std::vector<automsgs::msgs::geometry_msgs::Point> path;
};

class GraphPlanner3 {
 public:
  static GraphPlanResult Plan(const DynamicVisibilityGraph& graph,
                              const PlanningEnv& env, std::size_t from_id,
                              std::size_t to_id, bool prefer_free_space,
                              bool auto_switch);

  static bool Plan(const DynamicVisibilityGraph& graph, std::size_t from_id,
                   std::size_t to_id,
                   std::vector<automsgs::msgs::geometry_msgs::Point>* path);
};

}  // namespace autonomy::perception::exploration::far3d
