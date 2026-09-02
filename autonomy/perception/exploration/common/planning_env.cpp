/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/perception/exploration/common/planning_env.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <queue>

#include "autonomy/map/costmap_2d/cost_values.hpp"

namespace autonomy::perception::exploration {
namespace {

constexpr float kLogOddsHit = 0.85f;
constexpr float kLogOddsMiss = -0.4f;
constexpr float kLogOddsClamp = 5.0f;
constexpr unsigned char kNoInfo = map::costmap_2d::NO_INFORMATION;
constexpr unsigned char kFree = map::costmap_2d::FREE_SPACE;
constexpr unsigned char kLethal = map::costmap_2d::LETHAL_OBSTACLE;

double YawFromOdom(const automsgs::msgs::nav_msgs::Odometry& odom) {
  const auto& q = odom.pose().pose().pose().orientation();
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

float DecodeDepth(const automsgs::msgs::sensor_msgs::Image& depth, int idx,
                  const std::string& encoding) {
  if (encoding == "32FC1" && idx + 3 < depth.data().size()) {
    float value = 0.f;
    std::memcpy(&value, depth.data().data() + idx, sizeof(float));
    return value;
  }
  if ((encoding == "16UC1" || encoding == "mono16") &&
      idx + 1 < depth.data().size()) {
    uint16_t raw = 0;
    std::memcpy(&raw, depth.data().data() + idx, sizeof(uint16_t));
    return static_cast<float>(raw) * 0.001f;
  }
  return std::numeric_limits<float>::quiet_NaN();
}

}  // namespace

PlanningEnv::PlanningEnv(const proto::ExplorationOptions& options) {
  SetOptions(options);
}

void PlanningEnv::SetOptions(const proto::ExplorationOptions& options) {
  options_ = options;
  InitCostmap();
  automsgs::msgs::geometry_msgs::Polygon area;
  const double half = options.default_area_half_extent_m() > 0
                          ? options.default_area_half_extent_m()
                          : 20.0;
  auto* p0 = area.add_points();
  p0->set_x(-half);
  p0->set_y(-half);
  auto* p1 = area.add_points();
  p1->set_x(half);
  p1->set_y(-half);
  auto* p2 = area.add_points();
  p2->set_x(half);
  p2->set_y(half);
  auto* p3 = area.add_points();
  p3->set_x(-half);
  p3->set_y(half);
  SetExplorationArea(area);
  if (options.nogo_half_extent_m() > 0) {
    automsgs::msgs::geometry_msgs::Polygon nogo;
    const double h = options.nogo_half_extent_m();
    auto* n0 = nogo.add_points();
    n0->set_x(-h);
    n0->set_y(-h);
    auto* n1 = nogo.add_points();
    n1->set_x(h);
    n1->set_y(-h);
    auto* n2 = nogo.add_points();
    n2->set_x(h);
    n2->set_y(h);
    auto* n3 = nogo.add_points();
    n3->set_x(-h);
    n3->set_y(h);
    SetNogoArea(nogo);
  }
}

void PlanningEnv::SetExplorationArea(
    const automsgs::msgs::geometry_msgs::Polygon& area) {
  std::vector<::autonomy::common::math::Vec2d> pts;
  pts.reserve(static_cast<size_t>(area.points_size()));
  for (const auto& pt : area.points()) {
    pts.emplace_back(pt.x(), pt.y());
  }
  if (pts.size() >= 3) {
    exploration_polygon_ = ::autonomy::common::math::Polygon2d(pts);
  }
}

void PlanningEnv::SetNogoArea(
    const automsgs::msgs::geometry_msgs::Polygon& area) {
  std::vector<::autonomy::common::math::Vec2d> pts;
  pts.reserve(static_cast<size_t>(area.points_size()));
  for (const auto& pt : area.points()) {
    pts.emplace_back(pt.x(), pt.y());
  }
  if (pts.size() >= 3) {
    nogo_polygon_ = ::autonomy::common::math::Polygon2d(pts);
  }
}

bool PlanningEnv::IsInNogoArea(double x, double y) const {
  if (nogo_polygon_.num_points() < 3) {
    return false;
  }
  return nogo_polygon_.IsPointIn({x, y});
}

void PlanningEnv::UpdatePointCloud(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud) {
  if (!has_odom_ || cloud.data().empty()) {
    return;
  }
  int ox = -1;
  int oy = -1;
  int oz = -1;
  for (int i = 0; i < cloud.fields_size(); ++i) {
    if (cloud.fields(i).name() == "x") {
      ox = static_cast<int>(cloud.fields(i).offset());
    } else if (cloud.fields(i).name() == "y") {
      oy = static_cast<int>(cloud.fields(i).offset());
    } else if (cloud.fields(i).name() == "z") {
      oz = static_cast<int>(cloud.fields(i).offset());
    }
  }
  if (ox < 0 || oy < 0 || oz < 0) {
    return;
  }
  const int count = static_cast<int>(cloud.width() * cloud.height());
  const int stride = std::max(options_.depth_stride(), 1);
  const double range = options_.sensor_range_m();
  const double ground_z = robot_z_ - options_.camera_height_m();
  for (int i = 0; i < count; i += stride) {
    const int idx = i * static_cast<int>(cloud.point_step());
    if (idx + oz + static_cast<int>(sizeof(float)) >
        static_cast<int>(cloud.data().size())) {
      continue;
    }
    float px = 0.f;
    float py = 0.f;
    float pz = 0.f;
    std::memcpy(&px, cloud.data().data() + idx + ox, sizeof(float));
    std::memcpy(&py, cloud.data().data() + idx + oy, sizeof(float));
    std::memcpy(&pz, cloud.data().data() + idx + oz, sizeof(float));
    if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
      continue;
    }
    if (std::hypot(px - robot_x_, py - robot_y_) > range) {
      continue;
    }
    if (!IsInExplorationArea(px, py) || IsInNogoArea(px, py)) {
      continue;
    }
    if (pz < ground_z + options_.ground_height_tol_m()) {
      continue;
    }
    unsigned int cell_x = 0;
    unsigned int cell_y = 0;
    if (costmap_.worldToMap(px, py, cell_x, cell_y)) {
      const size_t index =
          static_cast<size_t>(cell_y) * costmap_.getSizeInCellsX() + cell_x;
      ApplyLogOdds(index, kLogOddsHit);
    }
  }
  SyncCostmapFromLogOdds();
  RebuildInflation();
  ExtractFrontiers();
}

void PlanningEnv::InitCostmap() {
  const int size = std::max(options_.costmap_size_cells(), 100);
  const double res = options_.costmap_resolution_m() > 0
                         ? options_.costmap_resolution_m()
                         : 0.1;
  costmap_.resizeMap(static_cast<unsigned int>(size),
                     static_cast<unsigned int>(size), res, -res * size * 0.5,
                     -res * size * 0.5);
  inflated_ = costmap_;
  log_odds_.assign(static_cast<size_t>(size * size), 0.f);
  covered_.assign(static_cast<size_t>(size * size), 0);
  SyncCostmapFromLogOdds();
  RebuildInflation();
}

void PlanningEnv::UpdateOdometry(
    const automsgs::msgs::nav_msgs::Odometry& odom) {
  robot_x_ = odom.pose().pose().pose().position().x();
  robot_y_ = odom.pose().pose().pose().position().y();
  robot_z_ = odom.pose().pose().pose().position().z();
  robot_yaw_ = YawFromOdom(odom);
  has_odom_ = true;
  RollCostmapIfNeeded(robot_x_, robot_y_);
}

void PlanningEnv::RollCostmapIfNeeded(double x, double y) {
  const double ox = costmap_.getOriginX();
  const double oy = costmap_.getOriginY();
  const double size_x = costmap_.getSizeInMetersX();
  const double size_y = costmap_.getSizeInMetersY();
  const double margin = 0.25 * std::min(size_x, size_y);
  if (x > ox + margin && x < ox + size_x - margin && y > oy + margin &&
      y < oy + size_y - margin) {
    return;
  }
  costmap_.updateOrigin(x - size_x * 0.5, y - size_y * 0.5);
  inflated_ = costmap_;
  PersistLocalToGlobal();
  log_odds_.assign(log_odds_.size(), 0.f);
  covered_.assign(covered_.size(), 0);
  RestoreGlobalToLocal();
  SyncCostmapFromLogOdds();
  RebuildInflation();
}

int64_t PlanningEnv::GlobalKey(int gx, int gy) {
  return (static_cast<int64_t>(gx) << 32) ^
         static_cast<int64_t>(gy & 0xffffffff);
}

void PlanningEnv::WorldToGlobal(double wx, double wy, int* gx, int* gy) const {
  const double res = costmap_.getResolution();
  *gx = static_cast<int>(std::floor(wx / res));
  *gy = static_cast<int>(std::floor(wy / res));
}

void PlanningEnv::WriteGlobalLogOdds(double wx, double wy, float value) {
  int gx = 0;
  int gy = 0;
  WorldToGlobal(wx, wy, &gx, &gy);
  global_log_odds_[GlobalKey(gx, gy)] = value;
}

void PlanningEnv::WriteGlobalCovered(double wx, double wy, uint8_t value) {
  int gx = 0;
  int gy = 0;
  WorldToGlobal(wx, wy, &gx, &gy);
  if (value != 0) {
    global_covered_[GlobalKey(gx, gy)] = 1;
  }
}

bool PlanningEnv::ReadGlobalCovered(double wx, double wy) const {
  int gx = 0;
  int gy = 0;
  WorldToGlobal(wx, wy, &gx, &gy);
  const auto it = global_covered_.find(GlobalKey(gx, gy));
  return it != global_covered_.end() && it->second != 0;
}

void PlanningEnv::PersistLocalToGlobal() {
  const unsigned int w = costmap_.getSizeInCellsX();
  const unsigned int h = costmap_.getSizeInCellsY();
  for (unsigned int y = 0; y < h; ++y) {
    for (unsigned int x = 0; x < w; ++x) {
      const size_t idx = static_cast<size_t>(y) * w + x;
      if (idx >= log_odds_.size()) {
        continue;
      }
      double wx = 0.0;
      double wy = 0.0;
      costmap_.mapToWorld(x, y, wx, wy);
      WriteGlobalLogOdds(wx, wy, log_odds_[idx]);
      if (idx < covered_.size() && covered_[idx] != 0) {
        WriteGlobalCovered(wx, wy, 1);
      }
    }
  }
}

void PlanningEnv::RestoreGlobalToLocal() {
  const unsigned int w = costmap_.getSizeInCellsX();
  const unsigned int h = costmap_.getSizeInCellsY();
  const double res = costmap_.getResolution();
  for (unsigned int y = 0; y < h; ++y) {
    for (unsigned int x = 0; x < w; ++x) {
      double wx = 0.0;
      double wy = 0.0;
      costmap_.mapToWorld(x, y, wx, wy);
      const int gx = static_cast<int>(std::floor(wx / res));
      const int gy = static_cast<int>(std::floor(wy / res));
      const int64_t key = GlobalKey(gx, gy);
      const size_t idx = static_cast<size_t>(y) * w + x;
      const auto odds_it = global_log_odds_.find(key);
      if (odds_it != global_log_odds_.end() && idx < log_odds_.size()) {
        log_odds_[idx] = odds_it->second;
      }
      const auto covered_it = global_covered_.find(key);
      if (covered_it != global_covered_.end() && covered_it->second != 0 &&
          idx < covered_.size()) {
        covered_[idx] = 1;
      }
    }
  }
}

void PlanningEnv::ApplyLogOdds(size_t index, float delta) {
  if (index >= log_odds_.size()) {
    return;
  }
  float value = log_odds_[index] + delta;
  value = std::clamp(value, -kLogOddsClamp, kLogOddsClamp);
  log_odds_[index] = value;
  const unsigned int x =
      static_cast<unsigned int>(index % costmap_.getSizeInCellsX());
  const unsigned int y =
      static_cast<unsigned int>(index / costmap_.getSizeInCellsX());
  double wx = 0.0;
  double wy = 0.0;
  costmap_.mapToWorld(x, y, wx, wy);
  WriteGlobalLogOdds(wx, wy, value);
}

void PlanningEnv::SyncCostmapFromLogOdds() {
  for (size_t i = 0; i < log_odds_.size(); ++i) {
    const float odds = log_odds_[i];
    unsigned char cost = kNoInfo;
    if (odds > 0.5f) {
      cost = kLethal;
    } else if (odds < -0.5f) {
      cost = kFree;
    }
    const unsigned int x = static_cast<unsigned int>(i % costmap_.getSizeInCellsX());
    const unsigned int y =
        static_cast<unsigned int>(i / costmap_.getSizeInCellsX());
    costmap_.setCost(x, y, cost);
  }
}

void PlanningEnv::RebuildInflation() {
  inflated_ = costmap_;
  const double radius = options_.collision_radius_m();
  const int cells = static_cast<int>(
      std::ceil(radius / std::max(costmap_.getResolution(), 1e-3)));
  const unsigned int w = costmap_.getSizeInCellsX();
  const unsigned int h = costmap_.getSizeInCellsY();
  for (unsigned int y = 0; y < h; ++y) {
    for (unsigned int x = 0; x < w; ++x) {
      if (costmap_.getCost(x, y) != kLethal) {
        continue;
      }
      for (int dy = -cells; dy <= cells; ++dy) {
        for (int dx = -cells; dx <= cells; ++dx) {
          const int nx = static_cast<int>(x) + dx;
          const int ny = static_cast<int>(y) + dy;
          if (nx < 0 || ny < 0 || nx >= static_cast<int>(w) ||
              ny >= static_cast<int>(h)) {
            continue;
          }
          inflated_.setCost(static_cast<unsigned int>(nx),
                            static_cast<unsigned int>(ny), kLethal);
        }
      }
    }
  }
}

bool PlanningEnv::IsOccupied(double x, double y) const {
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!inflated_.worldToMap(x, y, mx, my)) {
    return true;
  }
  return inflated_.getCost(mx, my) == kLethal;
}

bool PlanningEnv::IsFree(double x, double y) const {
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap_.worldToMap(x, y, mx, my)) {
    return false;
  }
  return costmap_.getCost(mx, my) == kFree;
}

bool PlanningEnv::IsCovered(double x, double y) const {
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap_.worldToMap(x, y, mx, my)) {
    return ReadGlobalCovered(x, y);
  }
  const size_t idx =
      static_cast<size_t>(my) * costmap_.getSizeInCellsX() + mx;
  if (idx < covered_.size() && covered_[idx] != 0) {
    return true;
  }
  return ReadGlobalCovered(x, y);
}

bool PlanningEnv::IsInExplorationArea(double x, double y) const {
  if (exploration_polygon_.num_points() < 3) {
    return true;
  }
  return exploration_polygon_.IsPointIn({x, y});
}

void PlanningEnv::UpdateDepth(
    const automsgs::msgs::sensor_msgs::Image& depth,
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    const automsgs::msgs::geometry_msgs::Transform& map_t_camera) {
  if (!has_odom_) {
    return;
  }
  FuseDepthFrame(depth, info, map_t_camera);
  MarkCoveredFromExtrinsic(map_t_camera);
  ExtractFrontiers();
}

void PlanningEnv::FuseDepthFrame(
    const automsgs::msgs::sensor_msgs::Image& depth,
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    const automsgs::msgs::geometry_msgs::Transform& map_t_camera) {
  if (info.k_size() < 9 || depth.data().empty()) {
    return;
  }
  const double fx = info.k(0);
  const double fy = info.k(4);
  const double cx = info.k(2);
  const double cy = info.k(5);
  const int stride = std::max(options_.depth_stride(), 1);
  const int width = static_cast<int>(info.width());
  const int height = static_cast<int>(info.height());
  const double min_d = options_.depth_min_m();
  const double max_d = options_.depth_max_m();
  const auto& t = map_t_camera.translation();
  const auto& q = map_t_camera.rotation();
  const double tx = t.x();
  const double ty = t.y();
  const double tz = t.z();
  const double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                                 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  const double cos_y = std::cos(yaw);
  const double sin_y = std::sin(yaw);

  for (int v = 0; v < height; v += stride) {
    for (int u = 0; u < width; u += stride) {
      const int idx = (v * static_cast<int>(depth.step()) +
                       u * static_cast<int>(depth.step() / std::max(width, 1)));
      const float d = DecodeDepth(depth, idx, depth.encoding());
      if (!std::isfinite(d) || d < min_d || d > max_d) {
        continue;
      }
      const double bx = (static_cast<double>(u) - cx) * d / fx;
      const double by = (static_cast<double>(v) - cy) * d / fy;
      const double bz = d;
      const double mx = tx + cos_y * bx - sin_y * by;
      const double my = ty + sin_y * bx + cos_y * by;
      const double mz = tz + bz;
      if (mz > options_.ground_height_tol_m()) {
        unsigned int cell_x = 0;
        unsigned int cell_y = 0;
        if (costmap_.worldToMap(mx, my, cell_x, cell_y)) {
          const size_t index =
              static_cast<size_t>(cell_y) * costmap_.getSizeInCellsX() + cell_x;
          ApplyLogOdds(index, kLogOddsHit);
        }
      }
      // Ray miss along ground plane from robot to endpoint.
      const int steps = static_cast<int>(d / costmap_.getResolution());
      for (int s = 0; s < steps; ++s) {
        const double ratio = static_cast<double>(s) / std::max(steps, 1);
        const double rx = robot_x_ + ratio * (mx - robot_x_);
        const double ry = robot_y_ + ratio * (my - robot_y_);
        unsigned int cell_x = 0;
        unsigned int cell_y = 0;
        if (costmap_.worldToMap(rx, ry, cell_x, cell_y)) {
          const size_t index =
              static_cast<size_t>(cell_y) * costmap_.getSizeInCellsX() + cell_x;
          ApplyLogOdds(index, kLogOddsMiss);
        }
      }
    }
  }
  SyncCostmapFromLogOdds();
  RebuildInflation();
}

void PlanningEnv::MarkCoveredFromExtrinsic(
    const automsgs::msgs::geometry_msgs::Transform& map_t_camera) {
  const double range = options_.sensor_range_m();
  const double fov = M_PI / 2.0;
  const double cx = map_t_camera.translation().x();
  const double cy = map_t_camera.translation().y();
  const auto& q = map_t_camera.rotation();
  const double yaw = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                                 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  const int steps = static_cast<int>(range / costmap_.getResolution());
  for (int i = -steps; i <= steps; ++i) {
    for (int j = 0; j <= steps; ++j) {
      const double angle =
          yaw + (static_cast<double>(i) / std::max(steps, 1)) * fov;
      const double dist =
          range * static_cast<double>(j) / std::max(steps, 1);
      const double wx = cx + dist * std::cos(angle);
      const double wy = cy + dist * std::sin(angle);
      unsigned int mx = 0;
      unsigned int my = 0;
      if (costmap_.worldToMap(wx, wy, mx, my)) {
        const size_t idx =
            static_cast<size_t>(my) * costmap_.getSizeInCellsX() + mx;
        if (idx < covered_.size()) {
          covered_[idx] = 1;
        }
        double wx = 0.0;
        double wy = 0.0;
        costmap_.mapToWorld(mx, my, wx, wy);
        WriteGlobalCovered(wx, wy, 1);
      }
    }
  }
}

void PlanningEnv::ExtractFrontiers() {
  frontiers_.clear();
  frontier_cluster_sizes_.clear();
  const unsigned int w = costmap_.getSizeInCellsX();
  const unsigned int h = costmap_.getSizeInCellsY();
  const int min_cluster = std::max(options_.min_frontier_cluster_size(), 1);

  std::vector<uint8_t> is_frontier(w * h, 0);
  for (unsigned int y = 1; y + 1 < h; ++y) {
    for (unsigned int x = 1; x + 1 < w; ++x) {
      if (costmap_.getCost(x, y) != kFree) {
        continue;
      }
      bool touches_unknown = false;
      for (int dy = -1; dy <= 1 && !touches_unknown; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (costmap_.getCost(x + static_cast<unsigned int>(dx),
                               y + static_cast<unsigned int>(dy)) == kNoInfo) {
            touches_unknown = true;
            break;
          }
        }
      }
      if (touches_unknown) {
        is_frontier[static_cast<size_t>(y) * w + x] = 1;
      }
    }
  }

  std::vector<uint8_t> visited(w * h, 0);
  for (unsigned int y = 1; y + 1 < h; ++y) {
    for (unsigned int x = 1; x + 1 < w; ++x) {
      const size_t start = static_cast<size_t>(y) * w + x;
      if (!is_frontier[start] || visited[start]) {
        continue;
      }

      std::queue<std::pair<unsigned int, unsigned int>> queue;
      queue.push({x, y});
      visited[start] = 1;
      double sum_x = 0.0;
      double sum_y = 0.0;
      int count = 0;

      while (!queue.empty()) {
        const auto [cx, cy] = queue.front();
        queue.pop();
        double wx = 0.0;
        double wy = 0.0;
        costmap_.mapToWorld(cx, cy, wx, wy);
        sum_x += wx;
        sum_y += wy;
        ++count;

        static constexpr int kNeighbors[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        for (const auto& offset : kNeighbors) {
          const int nx = static_cast<int>(cx) + offset[0];
          const int ny = static_cast<int>(cy) + offset[1];
          if (nx < 1 || ny < 1 || nx + 1 >= static_cast<int>(w) ||
              ny + 1 >= static_cast<int>(h)) {
            continue;
          }
          const size_t ni =
              static_cast<size_t>(ny) * w + static_cast<unsigned int>(nx);
          if (!is_frontier[ni] || visited[ni]) {
            continue;
          }
          visited[ni] = 1;
          queue.push({static_cast<unsigned int>(nx),
                      static_cast<unsigned int>(ny)});
        }
      }

      if (count < min_cluster) {
        continue;
      }
      const double centroid_x = sum_x / static_cast<double>(count);
      const double centroid_y = sum_y / static_cast<double>(count);
      if (!IsInExplorationArea(centroid_x, centroid_y)) {
        continue;
      }
      automsgs::msgs::geometry_msgs::Point pt;
      pt.set_x(centroid_x);
      pt.set_y(centroid_y);
      frontiers_.push_back(pt);
      frontier_cluster_sizes_.push_back(count);
    }
  }
}

void PlanningEnv::UpdatePriorMap(
    const automsgs::msgs::map_msgs::OccupancyGrid& map) {
  if (!options_.use_prior_map() || map.data().empty()) {
    return;
  }
  const int threshold = options_.prior_map_lethal_threshold() > 0
                            ? options_.prior_map_lethal_threshold()
                            : 65;
  FuseOccupancyGrid(map, threshold, true);
}

void PlanningEnv::UpdatePlannerCostmap(
    const automsgs::msgs::map_msgs::OccupancyGrid& map) {
  if (!options_.use_planner_costmap() || map.data().empty()) {
    return;
  }
  const int threshold = options_.planner_costmap_lethal_threshold() > 0
                            ? options_.planner_costmap_lethal_threshold()
                            : 50;
  FuseOccupancyGrid(map, threshold, false);
}

void PlanningEnv::FuseOccupancyGrid(
    const automsgs::msgs::map_msgs::OccupancyGrid& map, int lethal_threshold,
    bool mark_free_unknown) {
  const double res =
      map.info().resolution() > 0 ? map.info().resolution() : 0.05;
  const double ox = map.info().origin().position().x();
  const double oy = map.info().origin().position().y();
  const unsigned int width = map.info().width();
  const unsigned int height = map.info().height();
  const double margin = options_.sensor_range_m() * 2.0;
  const double min_x = robot_x_ - margin;
  const double max_x = robot_x_ + margin;
  const double min_y = robot_y_ - margin;
  const double max_y = robot_y_ + margin;

  for (unsigned int y = 0; y < height; ++y) {
    for (unsigned int x = 0; x < width; ++x) {
      const int idx = static_cast<int>(y * width + x);
      if (idx < 0 || idx >= map.data_size()) {
        continue;
      }
      const int value = map.data(idx);
      if (value < 0) {
        continue;
      }
      const double wx = ox + (static_cast<double>(x) + 0.5) * res;
      const double wy = oy + (static_cast<double>(y) + 0.5) * res;
      if (wx < min_x || wx > max_x || wy < min_y || wy > max_y) {
        continue;
      }
      if (!IsInExplorationArea(wx, wy) || IsInNogoArea(wx, wy)) {
        continue;
      }
      if (value >= lethal_threshold) {
        WriteGlobalLogOdds(wx, wy, kLogOddsClamp);
        unsigned int mx = 0;
        unsigned int my = 0;
        if (costmap_.worldToMap(wx, wy, mx, my)) {
          const size_t index =
              static_cast<size_t>(my) * costmap_.getSizeInCellsX() + mx;
          if (index < log_odds_.size()) {
            log_odds_[index] = std::max(log_odds_[index], kLogOddsClamp);
          }
        }
      } else if (value == 0 && mark_free_unknown) {
        unsigned int mx = 0;
        unsigned int my = 0;
        if (costmap_.worldToMap(wx, wy, mx, my)) {
          const size_t index =
              static_cast<size_t>(my) * costmap_.getSizeInCellsX() + mx;
          if (index < log_odds_.size() &&
              std::abs(log_odds_[index]) < 0.1f) {
            log_odds_[index] = kLogOddsMiss;
          }
        }
      }
    }
  }
  SyncCostmapFromLogOdds();
  RebuildInflation();
  ExtractFrontiers();
}

void PlanningEnv::ClearObstacleNear(double x, double y, double radius_m) {
  if (radius_m <= 0.0) {
    return;
  }
  const double res = costmap_.getResolution();
  const int steps = static_cast<int>(std::ceil(radius_m / res));
  for (int dy = -steps; dy <= steps; ++dy) {
    for (int dx = -steps; dx <= steps; ++dx) {
      const double wx = x + static_cast<double>(dx) * res;
      const double wy = y + static_cast<double>(dy) * res;
      if (std::hypot(wx - x, wy - y) > radius_m) {
        continue;
      }
      WriteGlobalLogOdds(wx, wy, kLogOddsMiss);
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap_.worldToMap(wx, wy, mx, my)) {
        continue;
      }
      const size_t index =
          static_cast<size_t>(my) * costmap_.getSizeInCellsX() + mx;
      if (index < log_odds_.size()) {
        log_odds_[index] = kLogOddsMiss;
      }
    }
  }
  SyncCostmapFromLogOdds();
  RebuildInflation();
}

int PlanningEnv::FrontierCellCount() const {
  return static_cast<int>(frontiers_.size());
}

bool PlanningEnv::HasExplorationBoundary() const {
  return exploration_polygon_.num_points() >= 3;
}

automsgs::msgs::geometry_msgs::Polygon PlanningEnv::GetExplorationBoundary()
    const {
  automsgs::msgs::geometry_msgs::Polygon polygon;
  if (!HasExplorationBoundary()) {
    return polygon;
  }
  for (const auto& pt : exploration_polygon_.points()) {
    auto* out = polygon.add_points();
    out->set_x(pt.x());
    out->set_y(pt.y());
    out->set_z(0.0);
  }
  return polygon;
}

automsgs::msgs::map_msgs::OccupancyGrid PlanningEnv::GetOccupancyGrid(
    const std::string& frame_id) const {
  return GetOccupancyGridWithOverlay(frame_id, false);
}

automsgs::msgs::map_msgs::OccupancyGrid PlanningEnv::GetOccupancyGridWithOverlay(
    const std::string& frame_id, bool mark_frontiers) const {
  automsgs::msgs::map_msgs::OccupancyGrid grid;
  grid.mutable_info()->set_width(costmap_.getSizeInCellsX());
  grid.mutable_info()->set_height(costmap_.getSizeInCellsY());
  grid.mutable_info()->set_resolution(
      static_cast<float>(costmap_.getResolution()));
  grid.mutable_info()->mutable_origin()->mutable_position()->set_x(
      costmap_.getOriginX());
  grid.mutable_info()->mutable_origin()->mutable_position()->set_y(
      costmap_.getOriginY());
  grid.mutable_header()->set_frame_id(frame_id);
  const size_t n = static_cast<size_t>(costmap_.getSizeInCellsX() *
                                       costmap_.getSizeInCellsY());
  for (size_t i = 0; i < n; ++i) {
    const unsigned int x =
        static_cast<unsigned int>(i % costmap_.getSizeInCellsX());
    const unsigned int y =
        static_cast<unsigned int>(i / costmap_.getSizeInCellsX());
    const unsigned char c = costmap_.getCost(x, y);
    int value = -1;
    if (c == kFree) {
      value = 0;
    } else if (c == kLethal) {
      value = 100;
    }
    grid.add_data(value);
  }
  if (mark_frontiers) {
    for (const auto& frontier : frontiers_) {
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap_.worldToMap(frontier.x(), frontier.y(), mx, my)) {
        continue;
      }
      const size_t idx =
          static_cast<size_t>(my) * costmap_.getSizeInCellsX() + mx;
      if (idx < grid.data_size()) {
        grid.set_data(static_cast<int>(idx), 50);
      }
    }
  }
  return grid;
}

}  // namespace autonomy::perception::exploration
