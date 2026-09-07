/*
 * Copyright 2026 The Openbot Authors
 */
#include "autonomy/perception/exploration/far3d/visibility_graph3.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/perception/exploration/common/planning_utilities.hpp"
#include <automsgs/msgs/visualization_msgs/marker.pb.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <automsgs/msgs/sensor_msgs/point_field_conversion.hpp>

namespace autonomy::perception::exploration::far3d {

DynamicVisibilityGraph::DynamicVisibilityGraph(
    const proto::ExplorationOptions& options) {
  SetOptions(options);
}

void DynamicVisibilityGraph::SetOptions(
    const proto::ExplorationOptions& options) {
  options_ = options;
}

void DynamicVisibilityGraph::Reset() {
  nodes_.clear();
  id_tracker_ = 0;
  odom_node_id_ = 0;
}

void DynamicVisibilityGraph::ImportStaticGraph(
    const std::vector<VghNode>& static_nodes) {
  std::unordered_map<std::size_t, std::size_t> id_remap;
  for (const auto& src : static_nodes) {
    NavNode3 node;
    node.id = id_tracker_++;
    node.x = src.x;
    node.y = src.y;
    node.z = src.z;
    node.is_boundary = src.is_boundary;
    node.is_frontier = src.is_frontier;
    node.is_static = true;
    node.is_active = false;
    id_remap[src.id] = node.id;
    nodes_[node.id] = std::move(node);
  }
  for (const auto& src : static_nodes) {
    const auto it = id_remap.find(src.id);
    if (it == id_remap.end()) {
      continue;
    }
    auto& node = nodes_[it->second];
    for (std::size_t cid : src.connect_ids) {
      const auto neighbor = id_remap.find(cid);
      if (neighbor != id_remap.end()) {
        ConnectNodes(node.id, neighbor->second);
      }
    }
    for (std::size_t cid : src.contour_ids) {
      const auto neighbor = id_remap.find(cid);
      if (neighbor != id_remap.end()) {
        ConnectNodes(node.id, neighbor->second);
      }
    }
  }
}

void DynamicVisibilityGraph::ConnectNodes(std::size_t a, std::size_t b) {
  if (a == b) {
    return;
  }
  auto it_a = nodes_.find(a);
  auto it_b = nodes_.find(b);
  if (it_a == nodes_.end() || it_b == nodes_.end()) {
    return;
  }
  auto& node_a = it_a->second;
  auto& node_b = it_b->second;
  if (std::find(node_a.connect_ids.begin(), node_a.connect_ids.end(), b) ==
      node_a.connect_ids.end()) {
    node_a.connect_ids.push_back(b);
  }
  if (std::find(node_b.connect_ids.begin(), node_b.connect_ids.end(), a) ==
      node_b.connect_ids.end()) {
    node_b.connect_ids.push_back(a);
  }
}

void DynamicVisibilityGraph::Update(const PlanningEnv& env,
                                    const ContourGraph3& contour_graph,
                                    const std::vector<ContourPolygon>& polygons,
                                    double robot_z) {
  const double robot_x = env.robot_x();
  const double robot_y = env.robot_y();

  if (nodes_.empty()) {
    odom_node_id_ = AddNode(robot_x, robot_y, robot_z, true, false);
  } else {
    auto& odom = nodes_[odom_node_id_];
    odom.x = robot_x;
    odom.y = robot_y;
    odom.z = robot_z;
    odom.is_merged = false;
    odom.is_active = true;
  }

  for (const auto& polygon : polygons) {
    if (polygon.is_pillar) {
      continue;
    }
    for (const auto& vtx : polygon.vertices) {
      if (IsNearExisting(vtx.x(), vtx.y(), vtx.z())) {
        continue;
      }
      AddNode(vtx.x(), vtx.y(), vtx.z(), false, false);
    }
  }

  for (const auto& frontier : env.frontiers()) {
    if (IsNearExisting(frontier.x(), frontier.y(), robot_z)) {
      continue;
    }
    AddNode(frontier.x(), frontier.y(), robot_z, false, true);
  }

  const int pool = std::max(options_.far3d_graph_pool_size(), 64);
  std::vector<std::size_t> ids;
  ids.reserve(nodes_.size());
  for (const auto& entry : nodes_) {
    if (!entry.second.is_merged && entry.second.is_active) {
      ids.push_back(entry.first);
    }
  }
  if (static_cast<int>(ids.size()) > pool) {
    std::sort(ids.begin(), ids.end(), [&](std::size_t a, std::size_t b) {
      const auto& na = nodes_.at(a);
      const auto& nb = nodes_.at(b);
      const double da = std::hypot(na.x - robot_x, na.y - robot_y);
      const double db = std::hypot(nb.x - robot_x, nb.y - robot_y);
      return da > db;
    });
    for (size_t i = static_cast<size_t>(pool); i < ids.size(); ++i) {
      nodes_[ids[i]].is_merged = true;
    }
  }

  for (size_t i = 0; i < ids.size(); ++i) {
    for (size_t j = i + 1; j < ids.size(); ++j) {
      if (nodes_[ids[i]].is_merged || nodes_[ids[j]].is_merged) {
        continue;
      }
      TryConnect(ids[i], ids[j], contour_graph, env);
    }
  }

  ValidateEdges(contour_graph);
  PruneNodes(robot_x, robot_y, robot_z);
}

std::size_t DynamicVisibilityGraph::AddNode(double x, double y, double z,
                                            bool is_odom, bool is_frontier) {
  NavNode3 node;
  node.id = id_tracker_++;
  node.x = x;
  node.y = y;
  node.z = z;
  node.is_odom = is_odom;
  node.is_frontier = is_frontier;
  nodes_[node.id] = std::move(node);
  return node.id;
}

bool DynamicVisibilityGraph::IsNearExisting(double x, double y, double z) const {
  const double margin = options_.far3d_position_filter_m() > 0
                            ? options_.far3d_position_filter_m()
                            : 0.5;
  for (const auto& entry : nodes_) {
    const auto& node = entry.second;
    if (node.is_merged) {
      continue;
    }
    if (std::hypot(node.x - x, node.y - y) < margin &&
        std::abs(node.z - z) < options_.far3d_height_margin_m()) {
      return true;
    }
  }
  return false;
}

bool DynamicVisibilityGraph::TryConnect(std::size_t a, std::size_t b,
                                        const ContourGraph3& graph,
                                        const PlanningEnv& env) {
  auto it_a = nodes_.find(a);
  auto it_b = nodes_.find(b);
  if (it_a == nodes_.end() || it_b == nodes_.end()) {
    return false;
  }
  const auto& na = it_a->second;
  const auto& nb = it_b->second;
  const double max_dist = options_.far_connect_dist_m() > 0
                            ? options_.far_connect_dist_m()
                            : options_.sensor_range_m();
  if (std::hypot(na.x - nb.x, na.y - nb.y) > max_dist) {
    return false;
  }

  if (!HasLineOfSight(env, na, nb)) {
    return false;
  }

  automsgs::msgs::geometry_msgs::Point pa;
  pa.set_x(na.x);
  pa.set_y(na.y);
  pa.set_z(na.z);
  automsgs::msgs::geometry_msgs::Point pb;
  pb.set_x(nb.x);
  pb.set_y(nb.y);
  pb.set_z(nb.z);
  if (!graph.IsSegmentInFreePolygon(pa, pb)) {
    return false;
  }

  auto& node_a = nodes_[a];
  auto& node_b = nodes_[b];
  auto add_edge = [&](NavNode3& from, NavNode3& to) {
    if (std::find(from.connect_ids.begin(), from.connect_ids.end(), to.id) ==
        from.connect_ids.end()) {
      from.connect_ids.push_back(to.id);
    }
    from.edge_votes[to.id] = 0;
  };
  add_edge(node_a, node_b);
  add_edge(node_b, node_a);
  return true;
}

void DynamicVisibilityGraph::ValidateEdges(const ContourGraph3& graph) {
  const int finalize_thred = std::max(options_.far3d_edge_finalize_thred(), 2);
  for (auto& entry : nodes_) {
    NavNode3& node = entry.second;
    if (node.is_merged) {
      continue;
    }
    std::vector<std::size_t> keep;
    for (std::size_t nid : node.connect_ids) {
      auto it = nodes_.find(nid);
      if (it == nodes_.end() || it->second.is_merged) {
        continue;
      }
      automsgs::msgs::geometry_msgs::Point pa;
      pa.set_x(node.x);
      pa.set_y(node.y);
      pa.set_z(node.z);
      automsgs::msgs::geometry_msgs::Point pb;
      pb.set_x(it->second.x);
      pb.set_y(it->second.y);
      pb.set_z(it->second.z);
      if (graph.IsSegmentInFreePolygon(pa, pb)) {
        node.edge_votes[nid] = 0;
        keep.push_back(nid);
      } else {
        node.edge_votes[nid]++;
        if (node.edge_votes[nid] < finalize_thred) {
          keep.push_back(nid);
        }
      }
    }
    node.connect_ids = std::move(keep);
  }
}

void DynamicVisibilityGraph::PruneNodes(double robot_x, double robot_y,
                                      double robot_z) {
  const double range = options_.sensor_range_m();
  const int dumper_thred = std::max(options_.far3d_dumper_thred(), 5);
  for (auto& entry : nodes_) {
    NavNode3& node = entry.second;
    if (node.is_odom) {
      continue;
    }
    if (node.is_static) {
      continue;
    }
    if (std::hypot(node.x - robot_x, node.y - robot_y) > range ||
        std::abs(node.z - robot_z) > options_.far3d_height_margin_m() * 2.0) {
      node.clear_dumper_count++;
    } else {
      node.clear_dumper_count = std::max(0, node.clear_dumper_count - 1);
    }
    if (node.clear_dumper_count > dumper_thred) {
      node.is_merged = true;
    }
  }
}

bool DynamicVisibilityGraph::HasLineOfSight(const PlanningEnv& env,
                                            const NavNode3& a,
                                            const NavNode3& b) const {
  return LineOfSightChecker::HasLineOfSight(env, a.x, a.y, b.x, b.y);
}

automsgs::msgs::visualization_msgs::MarkerArray
DynamicVisibilityGraph::ToMarkerArray(const std::string& frame_id) const {
  using automsgs::msgs::visualization_msgs::Marker;
  automsgs::msgs::visualization_msgs::MarkerArray array;
  Marker edge_marker;
  edge_marker.mutable_header()->set_frame_id(frame_id);
  edge_marker.set_ns("exploration_vg3d_edges");
  edge_marker.set_id(0);
  edge_marker.set_type(Marker::LINE_LIST);
  edge_marker.set_action(Marker::ADD);
  edge_marker.mutable_scale()->set_x(0.04);
  edge_marker.mutable_color()->set_r(0.1f);
  edge_marker.mutable_color()->set_g(0.7f);
  edge_marker.mutable_color()->set_b(1.0f);
  edge_marker.mutable_color()->set_a(0.85f);

  int marker_id = 0;
  for (const auto& entry : nodes_) {
    const NavNode3& node = entry.second;
    if (node.is_merged) {
      continue;
    }
    Marker sphere;
    sphere.mutable_header()->set_frame_id(frame_id);
    sphere.set_ns("exploration_vg3d_nodes");
    sphere.set_id(marker_id++);
    sphere.set_type(Marker::SPHERE);
    sphere.set_action(Marker::ADD);
    sphere.mutable_pose()->mutable_position()->set_x(node.x);
    sphere.mutable_pose()->mutable_position()->set_y(node.y);
    sphere.mutable_pose()->mutable_position()->set_z(node.z);
    sphere.mutable_pose()->mutable_orientation()->set_w(1.0);
    sphere.mutable_scale()->set_x(0.2);
    sphere.mutable_scale()->set_y(0.2);
    sphere.mutable_scale()->set_z(0.2);
    if (node.is_odom) {
      sphere.mutable_color()->set_r(0.1f);
      sphere.mutable_color()->set_g(0.4f);
      sphere.mutable_color()->set_b(1.0f);
    } else if (node.is_frontier) {
      sphere.mutable_color()->set_r(1.0f);
      sphere.mutable_color()->set_g(0.4f);
      sphere.mutable_color()->set_b(0.0f);
    } else {
      sphere.mutable_color()->set_r(0.8f);
      sphere.mutable_color()->set_g(0.8f);
      sphere.mutable_color()->set_b(0.2f);
    }
    sphere.mutable_color()->set_a(0.95f);
    *array.add_markers() = sphere;

    for (std::size_t nid : node.connect_ids) {
      if (entry.first >= nid) {
        continue;
      }
      const auto it = nodes_.find(nid);
      if (it == nodes_.end() || it->second.is_merged) {
        continue;
      }
      auto* pa = edge_marker.add_points();
      pa->set_x(node.x);
      pa->set_y(node.y);
      pa->set_z(node.z);
      auto* pb = edge_marker.add_points();
      pb->set_x(it->second.x);
      pb->set_y(it->second.y);
      pb->set_z(it->second.z);
    }
  }
  if (edge_marker.points_size() > 0) {
    *array.add_markers() = edge_marker;
  }
  return array;
}


namespace {

bool EdgeIsFreeSpace(const PlanningEnv& env, const NavNode3& a,
                     const NavNode3& b) {
  return LineOfSightChecker::HasLineOfSightOnFreeSpace(env, a.x, a.y, b.x, b.y);
}

bool RunSearch(const DynamicVisibilityGraph& graph, const PlanningEnv& env,
               std::size_t from_id, std::size_t to_id, bool free_space_only,
               std::vector<automsgs::msgs::geometry_msgs::Point>* path) {
  if (path == nullptr) {
    return false;
  }
  path->clear();
  const auto& nodes = graph.nodes();
  if (nodes.find(from_id) == nodes.end() || nodes.find(to_id) == nodes.end()) {
    return false;
  }

  struct State {
    std::size_t id{0};
    double cost{0.0};
  };
  struct Cmp {
    bool operator()(const State& a, const State& b) const {
      return a.cost > b.cost;
    }
  };

  std::priority_queue<State, std::vector<State>, Cmp> open;
  std::unordered_map<std::size_t, double> g_score;
  std::unordered_map<std::size_t, std::size_t> parent;
  g_score[from_id] = 0.0;
  open.push({from_id, 0.0});

  const auto& goal = nodes.at(to_id);
  const auto heuristic = [&](std::size_t id) {
    const auto& n = nodes.at(id);
    return std::hypot(n.x - goal.x, n.y - goal.y);
  };

  while (!open.empty()) {
    const State current = open.top();
    open.pop();
    if (current.id == to_id) {
      break;
    }
    if (current.cost > g_score[current.id]) {
      continue;
    }
    const auto& node = nodes.at(current.id);
    for (std::size_t neighbor_id : node.connect_ids) {
      const auto it = nodes.find(neighbor_id);
      if (it == nodes.end() || it->second.is_merged) {
        continue;
      }
      if (free_space_only &&
          !EdgeIsFreeSpace(env, node, it->second)) {
        continue;
      }
      const double edge =
          std::hypot(node.x - it->second.x, node.y - it->second.y);
      const double tentative = current.cost + edge;
      auto score_it = g_score.find(neighbor_id);
      if (score_it != g_score.end() && tentative >= score_it->second) {
        continue;
      }
      g_score[neighbor_id] = tentative;
      parent[neighbor_id] = current.id;
      open.push({neighbor_id, tentative + heuristic(neighbor_id)});
    }
  }

  if (g_score.find(to_id) == g_score.end()) {
    return false;
  }

  std::vector<std::size_t> order;
  for (std::size_t cur = to_id; cur != from_id;) {
    order.push_back(cur);
    auto it = parent.find(cur);
    if (it == parent.end()) {
      return false;
    }
    cur = it->second;
  }
  order.push_back(from_id);
  for (auto rit = order.rbegin(); rit != order.rend(); ++rit) {
    const auto& node = nodes.at(*rit);
    automsgs::msgs::geometry_msgs::Point pt;
    pt.set_x(node.x);
    pt.set_y(node.y);
    pt.set_z(node.z);
    path->push_back(pt);
  }
  return !path->empty();
}

}  // namespace

GraphPlanResult GraphPlanner3::Plan(const DynamicVisibilityGraph& graph,
                                    const PlanningEnv& env, std::size_t from_id,
                                    std::size_t to_id, bool prefer_free_space,
                                    bool auto_switch) {
  GraphPlanResult result;
  if (prefer_free_space &&
      RunSearch(graph, env, from_id, to_id, true, &result.path)) {
    result.mode = NavMode::kFreeSpace;
    return result;
  }
  if (RunSearch(graph, env, from_id, to_id, false, &result.path)) {
    result.mode = NavMode::kAttemptable;
    return result;
  }
  if (auto_switch && prefer_free_space &&
      RunSearch(graph, env, from_id, to_id, false, &result.path)) {
    result.mode = NavMode::kAttemptable;
  }
  return result;
}

bool GraphPlanner3::Plan(const DynamicVisibilityGraph& graph, std::size_t from_id,
                         std::size_t to_id,
                         std::vector<automsgs::msgs::geometry_msgs::Point>* path) {
  if (path == nullptr) {
    return false;
  }
  PlanningEnv dummy{proto::ExplorationOptions()};
  return RunSearch(graph, dummy, from_id, to_id, false, path);
}


namespace {

using automsgs::msgs::sensor_msgs::PointCloud2;
using automsgs::msgs::sensor_msgs::PointField;

struct FieldOffsets {
  int x{-1};
  int y{-1};
  int z{-1};
};

FieldOffsets FindXYZOffsets(const PointCloud2& cloud) {
  FieldOffsets offsets;
  for (int i = 0; i < cloud.fields_size(); ++i) {
    const auto& field = cloud.fields(i);
    if (field.name() == "x") {
      offsets.x = static_cast<int>(field.offset());
    } else if (field.name() == "y") {
      offsets.y = static_cast<int>(field.offset());
    } else if (field.name() == "z") {
      offsets.z = static_cast<int>(field.offset());
    }
  }
  return offsets;
}

}  // namespace

ContourDetector3::ContourDetector3(const proto::ExplorationOptions& options) {
  SetOptions(options);
}

void ContourDetector3::SetOptions(const proto::ExplorationOptions& options) {
  options_ = options;
}

std::vector<ContourPolygon> ContourDetector3::Detect(
    const PointCloud2& cloud, double robot_x, double robot_y,
    double robot_z) const {
  std::vector<ContourPolygon> polygons;
  if (cloud.data().empty() || cloud.width() == 0) {
    return polygons;
  }

  const double voxel =
      options_.far3d_voxel_dim_m() > 0 ? options_.far3d_voxel_dim_m() : 0.2;
  const double range = options_.sensor_range_m() > 0 ? options_.sensor_range_m()
                                                     : 20.0;
  const int mat_size =
      static_cast<int>(std::ceil(2.0 * range / voxel)) + 2;
  const int center = mat_size / 2;
  const double height_margin = options_.far3d_height_margin_m() > 0
                                   ? options_.far3d_height_margin_m()
                                   : 1.5;

  cv::Mat img(mat_size, mat_size, CV_8UC1, cv::Scalar(0));
  const FieldOffsets offsets = FindXYZOffsets(cloud);
  if (offsets.x < 0 || offsets.y < 0 || offsets.z < 0) {
    return polygons;
  }

  const int point_count =
      static_cast<int>(cloud.width() * cloud.height());
  const int stride = std::max(options_.depth_stride(), 1);
  for (int i = 0; i < point_count; i += stride) {
    const int idx = i * static_cast<int>(cloud.point_step());
    if (idx + offsets.z + 8 > cloud.data().size()) {
      continue;
    }
    const float px = automsgs::msgs::sensor_msgs::readPointCloud2BufferValue<float>(
        reinterpret_cast<const unsigned char*>(cloud.data().data()) + idx +
            offsets.x,
        PointField::FLOAT32);
    const float py = automsgs::msgs::sensor_msgs::readPointCloud2BufferValue<float>(
        reinterpret_cast<const unsigned char*>(cloud.data().data()) + idx +
            offsets.y,
        PointField::FLOAT32);
    const float pz = automsgs::msgs::sensor_msgs::readPointCloud2BufferValue<float>(
        reinterpret_cast<const unsigned char*>(cloud.data().data()) + idx +
            offsets.z,
        PointField::FLOAT32);
    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
      continue;
    }
    if (std::abs(static_cast<double>(pz) - robot_z) > height_margin) {
      continue;
    }
    const double dx = static_cast<double>(px) - robot_x;
    const double dy = static_cast<double>(py) - robot_y;
    if (std::hypot(dx, dy) > range) {
      continue;
    }
    const int col = center + static_cast<int>(std::round(dx / voxel));
    const int row = center + static_cast<int>(std::round(dy / voxel));
    if (col < 0 || row < 0 || col >= mat_size || row >= mat_size) {
      continue;
    }
    img.at<unsigned char>(row, col) = 255;
  }

  const int blur = std::max(options_.far3d_contour_blur_size(), 3);
  cv::Mat blurred;
  cv::GaussianBlur(img, blurred, cv::Size(blur | 1, blur | 1), 0.0);
  const int thresh = std::max(options_.far3d_contour_thresh(), 10);
  cv::Mat binary;
  cv::threshold(blurred, binary, thresh, 255, cv::THRESH_BINARY);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto& contour : contours) {
    if (contour.size() < 3) {
      continue;
    }
    std::vector<cv::Point> approx;
    const size_t stride =
        std::max<size_t>(1, contour.size() / 24);
    for (size_t i = 0; i < contour.size(); i += stride) {
      approx.push_back(contour[i]);
    }
    if (approx.size() < 3) {
      approx = contour;
    }
    ContourPolygon polygon;
    double perimeter = 0.0;
    for (size_t i = 0; i < approx.size(); ++i) {
      const auto& p = approx[i];
      const auto& q = approx[(i + 1) % approx.size()];
      perimeter += std::hypot(static_cast<double>(p.x - q.x),
                              static_cast<double>(p.y - q.y));
      automsgs::msgs::geometry_msgs::Point vtx;
      vtx.set_x(robot_x + (static_cast<double>(p.x) - center) * voxel);
      vtx.set_y(robot_y + (static_cast<double>(p.y) - center) * voxel);
      vtx.set_z(robot_z);
      polygon.vertices.push_back(vtx);
    }
    polygon.perimeter = perimeter * voxel;
    polygon.is_pillar = polygon.perimeter < options_.far3d_pillar_perimeter_m();
    polygons.push_back(std::move(polygon));
  }
  return polygons;
}


ContourGraph3::ContourGraph3(const proto::ExplorationOptions& options) {
  SetOptions(options);
}

void ContourGraph3::SetOptions(const proto::ExplorationOptions& options) {
  options_ = options;
}

void ContourGraph3::UpdatePolygons(std::vector<ContourPolygon> polygons) {
  polygons_ = std::move(polygons);
  BuildCtNodes();
}

void ContourGraph3::BuildCtNodes() {
  ct_nodes_.clear();
  for (size_t pid = 0; pid < polygons_.size(); ++pid) {
    const auto& polygon = polygons_[pid];
    if (polygon.vertices.size() < 3) {
      continue;
    }
    for (size_t i = 0; i < polygon.vertices.size(); ++i) {
      const auto& p0 = polygon.vertices[i];
      const auto& p1 = polygon.vertices[(i + 1) % polygon.vertices.size()];
      const auto& p2 = polygon.vertices[(i + 2) % polygon.vertices.size()];
      const double v1x = p1.x() - p0.x();
      const double v1y = p1.y() - p0.y();
      const double v2x = p2.x() - p1.x();
      const double v2y = p2.y() - p1.y();
      const double cross = v1x * v2y - v1y * v2x;
      CtNode node;
      node.position = p1;
      node.polygon_id = static_cast<int>(pid);
      if (polygon.is_pillar) {
        node.convexity = ContourConvexity::kPillar;
      } else if (cross > 0) {
        node.convexity = ContourConvexity::kConvex;
      } else if (cross < 0) {
        node.convexity = ContourConvexity::kConcave;
      }
      const double len = std::hypot(v1x, v1y);
      if (len > 1e-6) {
        node.surface_dir_x = -v1y / len;
        node.surface_dir_y = v1x / len;
      }
      ct_nodes_.push_back(node);
    }
  }
}

bool ContourGraph3::ReprojectPointOutsidePolygons(double* x, double* y,
                                                  double* z) const {
  if (x == nullptr || y == nullptr || z == nullptr) {
    return false;
  }
  const double margin = options_.far3d_position_filter_m() > 0
                            ? options_.far3d_position_filter_m()
                            : 0.5;
  for (const auto& polygon : polygons_) {
    if (polygon.vertices.size() < 3) {
      continue;
    }
    bool inside = false;
    for (size_t i = 0, j = polygon.vertices.size() - 1; i < polygon.vertices.size();
         j = i++) {
      const auto& pi = polygon.vertices[i];
      const auto& pj = polygon.vertices[j];
      const bool intersect =
          ((pi.y() > *y) != (pj.y() > *y)) &&
          (*x < (pj.x() - pi.x()) * (*y - pi.y()) / (pj.y() - pi.y() + 1e-9) +
                     pi.x());
      if (intersect) {
        inside = !inside;
      }
    }
    if (!inside) {
      continue;
    }
    *x += margin;
    return true;
  }
  return false;
}

bool ContourGraph3::SegmentsIntersect2D(double ax, double ay, double bx,
                                        double by, double cx, double cy,
                                        double dx, double dy) {
  const auto orient = [](double x0, double y0, double x1, double y1,
                         double x2, double y2) {
    return (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
  };
  const auto on_segment = [](double x0, double y0, double x1, double y1,
                             double x2, double y2) {
    return std::min(x0, x1) <= x2 && x2 <= std::max(x0, x1) &&
           std::min(y0, y1) <= y2 && y2 <= std::max(y0, y1);
  };
  const double o1 = orient(ax, ay, bx, by, cx, cy);
  const double o2 = orient(ax, ay, bx, by, dx, dy);
  const double o3 = orient(cx, cy, dx, dy, ax, ay);
  const double o4 = orient(cx, cy, dx, dy, bx, by);
  if ((o1 > 0 && o2 < 0 || o1 < 0 && o2 > 0) &&
      (o3 > 0 && o4 < 0 || o3 < 0 && o4 > 0)) {
    return true;
  }
  if (std::abs(o1) < 1e-9 && on_segment(ax, ay, bx, by, cx, cy)) {
    return true;
  }
  if (std::abs(o2) < 1e-9 && on_segment(ax, ay, bx, by, dx, dy)) {
    return true;
  }
  if (std::abs(o3) < 1e-9 && on_segment(cx, cy, dx, dy, ax, ay)) {
    return true;
  }
  if (std::abs(o4) < 1e-9 && on_segment(cx, cy, dx, dy, bx, by)) {
    return true;
  }
  return false;
}

bool ContourGraph3::IsSegmentInFreePolygon(
    const automsgs::msgs::geometry_msgs::Point& a,
    const automsgs::msgs::geometry_msgs::Point& b) const {
  const double margin = options_.far3d_height_margin_m() > 0
                            ? options_.far3d_height_margin_m()
                            : 1.5;
  if (std::abs(a.z() - b.z()) > margin) {
    return false;
  }
  for (const auto& polygon : polygons_) {
    if (polygon.vertices.size() < 2) {
      continue;
    }
    for (size_t i = 0; i < polygon.vertices.size(); ++i) {
      const auto& p0 = polygon.vertices[i];
      const auto& p1 = polygon.vertices[(i + 1) % polygon.vertices.size()];
      if (SegmentsIntersect2D(a.x(), a.y(), b.x(), b.y(), p0.x(), p0.y(),
                              p1.x(), p1.y())) {
        return false;
      }
    }
  }
  return true;
}


namespace {

bool ParseLine(const std::string& line, VghNode* node) {
  if (node == nullptr) {
    return false;
  }
  std::istringstream iss(line);
  std::vector<std::string> parts;
  std::string token;
  while (iss >> token) {
    parts.push_back(token);
  }
  if (parts.size() < 15) {
    return false;
  }
  node->id = static_cast<std::size_t>(std::stoull(parts[0]));
  node->x = std::stod(parts[2]);
  node->y = std::stod(parts[3]);
  node->z = std::stod(parts[4]);
  node->is_boundary = std::stoi(parts[14]) != 0;
  node->is_frontier = std::stoi(parts[12]) != 0;
  node->is_navpoint = std::stoi(parts[13]) != 0;
  node->connect_ids.clear();
  node->contour_ids.clear();

  bool in_connect = true;
  bool in_poly = false;
  bool in_contour = false;
  for (size_t i = 15; i < parts.size(); ++i) {
    if (parts[i] == "|") {
      if (!in_poly && !in_contour) {
        in_connect = false;
        in_poly = true;
      } else if (in_poly && !in_contour) {
        in_poly = false;
        in_contour = true;
      } else {
        in_contour = false;
      }
      continue;
    }
    const std::size_t id = static_cast<std::size_t>(std::stoull(parts[i]));
    if (in_connect) {
      node->connect_ids.push_back(id);
    } else if (in_poly) {
      // poly edges ignored in minimal port
    } else if (in_contour) {
      node->contour_ids.push_back(id);
    }
  }
  return true;
}

}  // namespace

bool VghLoader::LoadFromFile(const std::string& path,
                             std::vector<VghNode>* nodes) {
  if (nodes == nullptr) {
    return false;
  }
  nodes->clear();
  std::ifstream input(path);
  if (!input.is_open()) {
    AWARN << "VghLoader: cannot open " << path;
    return false;
  }
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty()) {
      continue;
    }
    VghNode node;
    if (ParseLine(line, &node)) {
      nodes->push_back(std::move(node));
    }
  }
  AINFO << "VghLoader: loaded " << nodes->size() << " nodes from " << path;
  return !nodes->empty();
}

bool VghLoader::LoadFromConfigPath(
    const std::string& relative_or_absolute_path,
    const std::vector<std::string>& config_directories,
    std::vector<VghNode>* nodes) {
  if (relative_or_absolute_path.empty() || nodes == nullptr) {
    return false;
  }
  if (!relative_or_absolute_path.empty() &&
      relative_or_absolute_path.front() == '/') {
    return LoadFromFile(relative_or_absolute_path, nodes);
  }
  if (config_directories.empty()) {
    return LoadFromFile(relative_or_absolute_path, nodes);
  }
  try {
    ::autonomy::common::ConfigurationFileResolver resolver(config_directories);
    const std::string full_path =
        resolver.GetFullPathOrDie(relative_or_absolute_path);
    return LoadFromFile(full_path, nodes);
  } catch (const std::exception& ex) {
    AWARN << "VghLoader: resolve failed for " << relative_or_absolute_path
          << ": " << ex.what();
    return false;
  }
}

}  // namespace autonomy::perception::exploration::far3d
