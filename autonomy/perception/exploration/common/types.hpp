/*
 * Copyright 2026 The Openbot Authors
 *
 * Shared exploration types and topic constants.
 */

#pragma once

namespace autonomy::perception::exploration {

constexpr char kExplorationPathTopic[] = "/exploration/path";
constexpr char kExplorationWaypointTopic[] = "/exploration/waypoint";
constexpr char kExplorationMapTopic[] = "/exploration/map";
constexpr char kExplorationGlobalPathTopic[] = "/exploration/global_path";
constexpr char kExplorationLocalPathTopic[] = "/exploration/local_path";
constexpr char kExplorationFinishedTopic[] = "/exploration/finished";
constexpr char kExplorationProgressTopic[] = "/exploration/progress";
constexpr char kNavigationBoundaryTopic[] = "/exploration/navigation_boundary";
constexpr char kExplorationWaypointReachedTopic[] =
    "/exploration/waypoint_reached";
constexpr char kExplorationVgMarkersTopic[] = "/exploration/vg_markers";
constexpr char kExplorationResetTopic[] = "/exploration/reset";
constexpr char kExplorationPauseTopic[] = "/exploration/pause";

enum class CellStatus { kUnseen = 0, kExploring = 1, kCovered = 2 };

struct Viewpoint {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
  double score{0.0};
  int index{-1};
  bool collision{false};
  bool line_of_sight{true};
  bool connected{true};
  bool candidate{false};
};

struct VisibilityNode {
  int id{-1};
  double x{0.0};
  double y{0.0};
  bool is_robot{false};
  bool is_frontier{false};
  bool is_trajectory{false};
  bool is_boundary{false};
  bool is_merged{false};
  bool is_static{false};
  bool is_active{true};
  int frontier_cluster_size{0};
  int clear_dumper_count{0};
};

}  // namespace autonomy::perception::exploration
