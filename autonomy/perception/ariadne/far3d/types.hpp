/*
 * Copyright 2026 The Openbot Authors
 *
 * FAR3D shared types.
 */

#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>

#include <automsgs/msgs/geometry_msgs/point.pb.h>

namespace autonomy::perception::exploration::far3d {

struct NavNode3 {
  std::size_t id{0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  bool is_odom{false};
  bool is_frontier{false};
  bool is_boundary{false};
  bool is_static{false};
  bool is_active{true};
  bool is_merged{false};
  int clear_dumper_count{0};
  std::vector<std::size_t> connect_ids;
  std::unordered_map<std::size_t, int> edge_votes;
};

struct ContourPolygon {
  std::vector<automsgs::msgs::geometry_msgs::Point> vertices;
  double perimeter{0.0};
  bool is_pillar{false};
};

enum class ContourConvexity { kUnknown, kConvex, kConcave, kPillar };

struct CtNode {
  automsgs::msgs::geometry_msgs::Point position;
  ContourConvexity convexity{ContourConvexity::kUnknown};
  double surface_dir_x{0.0};
  double surface_dir_y{0.0};
  int polygon_id{-1};
};

}  // namespace autonomy::perception::exploration::far3d
