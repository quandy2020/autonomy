/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/exploration/planning_env.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

#include "Eigen/Geometry"

#include "autonomy/common/math/vec2d.hpp"
#include "autonomy/common/transform/transform.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/utils/line_iterator.hpp"

namespace autonomy {
namespace exploration {
namespace {

using map::costmap_2d::CostCellToOccupancyEquivalent;
using map::costmap_2d::FREE_SPACE;
using map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using map::costmap_2d::LETHAL_OBSTACLE;
using map::costmap_2d::NO_INFORMATION;
using map::costmap_2d::utils::LineIterator;

double DepthAt(const automsgs::msgs::sensor_msgs::Image& depth, int u, int v)
{
    if (u < 0 || v < 0 || static_cast<uint32_t>(u) >= depth.width() ||
        static_cast<uint32_t>(v) >= depth.height()) {
        return 0.0;
    }
    const size_t offset =
        static_cast<size_t>(v) * depth.step() +
        static_cast<size_t>(u) * (depth.encoding() == "32FC1" ? 4 : 2);
    if (offset >= depth.data().size()) {
        return 0.0;
    }
    if (depth.encoding() == "32FC1") {
        float val = 0.f;
        std::memcpy(&val, &depth.data()[offset], sizeof(float));
        return static_cast<double>(val);
    }
    if (depth.encoding() == "16UC1") {
        uint16_t raw = 0;
        std::memcpy(&raw, &depth.data()[offset], sizeof(uint16_t));
        return static_cast<double>(raw) * 0.001;
    }
    return 0.0;
}

automsgs::msgs::geometry_msgs::Point TransformPoint(
    const automsgs::msgs::geometry_msgs::Transform& t, double x, double y, double z)
{
    const auto& q = t.rotation();
    const double qw = q.w();
    const double qx = q.x();
    const double qy = q.y();
    const double qz = q.z();
    const double tx = t.translation().x();
    const double ty = t.translation().y();
    const double tz = t.translation().z();
    const double ix = qw * x + qy * z - qz * y;
    const double iy = qw * y + qz * x - qx * z;
    const double iz = qw * z + qx * y - qy * x;
    const double iw = -qx * x - qy * y - qz * z;
    automsgs::msgs::geometry_msgs::Point out;
    out.set_x(iw * -qx + ix * qw + iy * -qz - iz * -qy + tx);
    out.set_y(iw * -qy + iy * qw + iz * -qx - ix * -qz + ty);
    out.set_z(iw * -qz + iz * qw + ix * -qy - iy * -qx + tz);
    return out;
}

::autonomy::common::math::Polygon2d MakeSquarePolygon(double half_extent)
{
    std::vector<::autonomy::common::math::Vec2d> corners = {
        {-half_extent, -half_extent},
        {half_extent, -half_extent},
        {half_extent, half_extent},
        {-half_extent, half_extent},
    };
    return ::autonomy::common::math::Polygon2d(corners);
}

::autonomy::common::math::Polygon2d ToPolygon2d(
    const automsgs::msgs::geometry_msgs::Polygon& area)
{
    std::vector<::autonomy::common::math::Vec2d> points;
    points.reserve(area.points_size());
    for (const auto& p : area.points()) {
        points.emplace_back(static_cast<double>(p.x()),
                            static_cast<double>(p.y()));
    }
    if (points.size() < 3) {
        return ::autonomy::common::math::Polygon2d();
    }
    return ::autonomy::common::math::Polygon2d(points);
}

float ClampOdds(float v, float clamp_v)
{
    return std::max(-clamp_v, std::min(clamp_v, v));
}

}  // namespace

PlanningEnv::PlanningEnv(const proto::ExplorationOptions& options)
    : options_(options), camera_(options)
{
    InitCostmap();
    SetDefaultExplorationArea();
}

void PlanningEnv::InitCostmap()
{
    const unsigned int size_x =
        static_cast<unsigned int>(std::max(1, options_.occupancy().size_x()));
    const unsigned int size_y =
        static_cast<unsigned int>(std::max(1, options_.occupancy().size_y()));
    const double resolution = options_.occupancy().resolution();
    const double origin_x = -static_cast<double>(size_x) * resolution * 0.5;
    const double origin_y = -static_cast<double>(size_y) * resolution * 0.5;
    costmap_.setDefaultValue(NO_INFORMATION);
    costmap_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
    inflated_.setDefaultValue(NO_INFORMATION);
    inflated_.resizeMap(size_x, size_y, resolution, origin_x, origin_y);
    const size_t n = static_cast<size_t>(size_x) * size_y;
    log_odds_.assign(n, 0.f);
    covered_.assign(n, 0);
    RestoreGlobalToLocal();
    RebuildInflation();
}

void PlanningEnv::SetDefaultExplorationArea()
{
    exploration_polygon_ =
        MakeSquarePolygon(options_.grid_world().exploration_area_half_extent());
    exploration_area_.clear_points();
    for (const auto& v : exploration_polygon_.points()) {
        automsgs::msgs::geometry_msgs::Point32 p;
        p.set_x(static_cast<float>(v.x()));
        p.set_y(static_cast<float>(v.y()));
        p.set_z(0.f);
        *exploration_area_.add_points() = p;
    }
}

void PlanningEnv::SetOptions(const proto::ExplorationOptions& options)
{
    options_ = options;
    camera_.SetOptions(options);
    InitCostmap();
}

void PlanningEnv::SetExplorationArea(
    const automsgs::msgs::geometry_msgs::Polygon& area)
{
    exploration_area_ = area;
    exploration_polygon_ = ToPolygon2d(area);
}

int64_t PlanningEnv::GlobalKey(int gx, int gy)
{
    return (static_cast<int64_t>(gx) << 32) ^ static_cast<uint32_t>(gy);
}

void PlanningEnv::WorldToGlobal(double wx, double wy, int* gx, int* gy) const
{
    const double res = costmap_.getResolution();
    *gx = static_cast<int>(std::floor(wx / res));
    *gy = static_cast<int>(std::floor(wy / res));
}

void PlanningEnv::ArchiveLocalToGlobal()
{
    const unsigned int size_x = costmap_.getSizeInCellsX();
    const unsigned int size_y = costmap_.getSizeInCellsY();
    for (unsigned int my = 0; my < size_y; ++my) {
        for (unsigned int mx = 0; mx < size_x; ++mx) {
            const size_t idx =
                static_cast<size_t>(my) * size_x + static_cast<size_t>(mx);
            if (log_odds_[idx] == 0.f && covered_[idx] == 0) {
                continue;
            }
            double wx = 0.0;
            double wy = 0.0;
            costmap_.mapToWorld(mx, my, wx, wy);
            int gx = 0;
            int gy = 0;
            WorldToGlobal(wx, wy, &gx, &gy);
            const int64_t key = GlobalKey(gx, gy);
            global_log_odds_[key] = log_odds_[idx];
            global_covered_[key] = covered_[idx];
        }
    }
}

void PlanningEnv::RestoreGlobalToLocal()
{
    const unsigned int size_x = costmap_.getSizeInCellsX();
    const unsigned int size_y = costmap_.getSizeInCellsY();
    for (unsigned int my = 0; my < size_y; ++my) {
        for (unsigned int mx = 0; mx < size_x; ++mx) {
            double wx = 0.0;
            double wy = 0.0;
            costmap_.mapToWorld(mx, my, wx, wy);
            int gx = 0;
            int gy = 0;
            WorldToGlobal(wx, wy, &gx, &gy);
            const int64_t key = GlobalKey(gx, gy);
            const size_t idx =
                static_cast<size_t>(my) * size_x + static_cast<size_t>(mx);
            const auto odds_it = global_log_odds_.find(key);
            if (odds_it != global_log_odds_.end()) {
                log_odds_[idx] = odds_it->second;
            }
            const auto cov_it = global_covered_.find(key);
            if (cov_it != global_covered_.end()) {
                covered_[idx] = cov_it->second;
            }
        }
    }
    SyncCostmapFromLogOdds();
}

void PlanningEnv::RollCostmapIfNeeded(double x, double y)
{
    const double resolution = costmap_.getResolution();
    const unsigned int size_x = costmap_.getSizeInCellsX();
    const unsigned int size_y = costmap_.getSizeInCellsY();
    const double half_x = size_x * resolution * 0.5;
    const double half_y = size_y * resolution * 0.5;
    const double center_x = costmap_.getOriginX() + half_x;
    const double center_y = costmap_.getOriginY() + half_y;
    if (std::abs(x - center_x) < half_x * 0.4 &&
        std::abs(y - center_y) < half_y * 0.4) {
        return;
    }
    ArchiveLocalToGlobal();
    const double new_origin_x = x - half_x;
    const double new_origin_y = y - half_y;
    costmap_.setDefaultValue(NO_INFORMATION);
    costmap_.resizeMap(size_x, size_y, resolution, new_origin_x, new_origin_y);
    inflated_.setDefaultValue(NO_INFORMATION);
    inflated_.resizeMap(size_x, size_y, resolution, new_origin_x, new_origin_y);
    const size_t n = static_cast<size_t>(size_x) * size_y;
    log_odds_.assign(n, 0.f);
    covered_.assign(n, 0);
    RestoreGlobalToLocal();
    RebuildInflation();
}

void PlanningEnv::UpdateOdometry(const automsgs::msgs::planning_msgs::Odometry& odom)
{
    const auto& pose = odom.pose().pose().pose();
    robot_x_ = pose.position().x();
    robot_y_ = pose.position().y();
    robot_z_ = pose.position().z();
    const auto& q = pose.orientation();
    robot_yaw_ = ::autonomy::common::transform::GetYaw(
        Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()));
    has_odom_ = true;
    RollCostmapIfNeeded(robot_x_, robot_y_);
    // Prefer cached extrinsic (from depth). Never per-cell TF lookup on odom.
    if (has_map_t_camera_) {
        UpdateCoverageFromTf();
    }
}

void PlanningEnv::UpdateDepth(
    const automsgs::msgs::sensor_msgs::Image& depth,
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    const automsgs::msgs::geometry_msgs::Transform& map_t_camera)
{
    camera_.SetFromCameraInfo(info);
    last_map_t_camera_ = map_t_camera;
    has_map_t_camera_ = true;
    FuseDepthFrame(depth, info, map_t_camera);
    MarkCoveredFromExtrinsic(map_t_camera);
    RefreshTargets();
}

void PlanningEnv::ApplyLogOdds(size_t index, float delta)
{
    if (index >= log_odds_.size()) {
        return;
    }
    const float clamp_v = options_.occupancy().log_odds_clamp() > 0.0
                              ? static_cast<float>(
                                    options_.occupancy().log_odds_clamp())
                              : 3.5f;
    log_odds_[index] = ClampOdds(log_odds_[index] + delta, clamp_v);
}

void PlanningEnv::SyncCostmapFromLogOdds()
{
    const float occ_thr =
        options_.occupancy().log_odds_occupy_threshold() > 0.0
            ? static_cast<float>(
                  options_.occupancy().log_odds_occupy_threshold())
            : 0.85f;
    const float free_thr =
        options_.occupancy().log_odds_free_threshold() != 0.0
            ? static_cast<float>(
                  options_.occupancy().log_odds_free_threshold())
            : -0.4f;
    for (size_t i = 0; i < log_odds_.size(); ++i) {
        const float lo = log_odds_[i];
        unsigned char cost = NO_INFORMATION;
        if (lo >= occ_thr) {
            cost = LETHAL_OBSTACLE;
        } else if (lo != 0.f && lo <= free_thr) {
            cost = FREE_SPACE;
        } else if (lo != 0.f) {
            cost = FREE_SPACE;
        }
        unsigned int mx = 0;
        unsigned int my = 0;
        costmap_.indexToCells(static_cast<unsigned int>(i), mx, my);
        costmap_.setCost(mx, my, cost);
    }
}

void PlanningEnv::RebuildInflation()
{
    inflated_ = costmap_;
    const double radius = std::max(0.0, options_.collision_radius());
    if (radius <= 1e-6) {
        return;
    }
    const double res = costmap_.getResolution();
    const int r_cells = static_cast<int>(std::ceil(radius / res));
    const unsigned int size_x = costmap_.getSizeInCellsX();
    const unsigned int size_y = costmap_.getSizeInCellsY();
    const int r2 = r_cells * r_cells;
    for (unsigned int my = 0; my < size_y; ++my) {
        for (unsigned int mx = 0; mx < size_x; ++mx) {
            if (costmap_.getCost(mx, my) < LETHAL_OBSTACLE) {
                continue;
            }
            for (int dy = -r_cells; dy <= r_cells; ++dy) {
                for (int dx = -r_cells; dx <= r_cells; ++dx) {
                    if (dx * dx + dy * dy > r2) {
                        continue;
                    }
                    const int nx = static_cast<int>(mx) + dx;
                    const int ny = static_cast<int>(my) + dy;
                    if (nx < 0 || ny < 0 || nx >= static_cast<int>(size_x) ||
                        ny >= static_cast<int>(size_y)) {
                        continue;
                    }
                    if (dx == 0 && dy == 0) {
                        continue;
                    }
                    const unsigned int umx = static_cast<unsigned int>(nx);
                    const unsigned int umy = static_cast<unsigned int>(ny);
                    if (inflated_.getCost(umx, umy) <
                        INSCRIBED_INFLATED_OBSTACLE) {
                        inflated_.setCost(umx, umy, INSCRIBED_INFLATED_OBSTACLE);
                    }
                }
            }
        }
    }
}

void PlanningEnv::UpdateHit(double x, double y)
{
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_.worldToMap(x, y, mx, my)) {
        return;
    }
    const float hit = options_.occupancy().log_odds_hit() > 0.0
                          ? static_cast<float>(options_.occupancy().log_odds_hit())
                          : 0.85f;
    const size_t idx =
        static_cast<size_t>(my) * costmap_.getSizeInCellsX() + mx;
    ApplyLogOdds(idx, hit);
}

void PlanningEnv::UpdateMissRay(double ox, double oy, double tx, double ty)
{
    unsigned int x0 = 0;
    unsigned int y0 = 0;
    unsigned int x1 = 0;
    unsigned int y1 = 0;
    if (!costmap_.worldToMap(ox, oy, x0, y0)) {
        return;
    }
    if (!costmap_.worldToMap(tx, ty, x1, y1)) {
        int mx = 0;
        int my = 0;
        costmap_.worldToMapEnforceBounds(tx, ty, mx, my);
        x1 = static_cast<unsigned int>(mx);
        y1 = static_cast<unsigned int>(my);
    }
    const float miss = options_.occupancy().log_odds_miss() != 0.0
                           ? static_cast<float>(options_.occupancy().log_odds_miss())
                           : -0.4f;
    const unsigned int size_x = costmap_.getSizeInCellsX();
    for (LineIterator line(static_cast<int>(x0), static_cast<int>(y0),
                           static_cast<int>(x1), static_cast<int>(y1));
         line.isValid(); line.advance()) {
        const unsigned int mx = static_cast<unsigned int>(line.getX());
        const unsigned int my = static_cast<unsigned int>(line.getY());
        if (mx >= size_x || my >= costmap_.getSizeInCellsY()) {
            continue;
        }
        if (mx == x1 && my == y1) {
            break;
        }
        const size_t idx = static_cast<size_t>(my) * size_x + mx;
        ApplyLogOdds(idx, miss);
    }
}

bool PlanningEnv::IsOccupied(double x, double y) const
{
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!inflated_.worldToMap(x, y, mx, my)) {
        return false;
    }
    return inflated_.getCost(mx, my) >= INSCRIBED_INFLATED_OBSTACLE;
}

bool PlanningEnv::IsFree(double x, double y) const
{
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_.worldToMap(x, y, mx, my)) {
        return false;
    }
    return costmap_.getCost(mx, my) == FREE_SPACE;
}

bool PlanningEnv::IsCovered(double x, double y) const
{
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_.worldToMap(x, y, mx, my)) {
        return false;
    }
    const size_t idx =
        static_cast<size_t>(my) * costmap_.getSizeInCellsX() + mx;
    return idx < covered_.size() && covered_[idx] != 0;
}

void PlanningEnv::FuseDepthFrame(
    const automsgs::msgs::sensor_msgs::Image& depth,
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    const automsgs::msgs::geometry_msgs::Transform& map_t_camera)
{
    const double fx = info.k_size() >= 1 ? info.k(0) : camera_.hfov_rad();
    const double fy = info.k_size() >= 5 ? info.k(4) : fx;
    const double cx = info.k_size() >= 3 ? info.k(2) : depth.width() * 0.5;
    const double cy = info.k_size() >= 6 ? info.k(5) : depth.height() * 0.5;
    const int step = std::max(1, options_.depth_subsample());
    const double z_min = options_.occupancy().fuse_z_min() != 0.0
                             ? options_.occupancy().fuse_z_min()
                             : 0.05;
    const double z_max = options_.occupancy().fuse_z_max() > 0.0
                             ? options_.occupancy().fuse_z_max()
                             : 1.2;
    const double ox = map_t_camera.translation().x();
    const double oy = map_t_camera.translation().y();

    for (int v = 0; v < static_cast<int>(depth.height()); v += step) {
        for (int u = 0; u < static_cast<int>(depth.width()); u += step) {
            const double d = DepthAt(depth, u, v);
            if (d < options_.camera().z_near() || d > options_.camera().z_far()) {
                continue;
            }
            const double px = (static_cast<double>(u) - cx) * d / fx;
            const double py = (static_cast<double>(v) - cy) * d / fy;
            const auto world = TransformPoint(map_t_camera, px, py, d);
            const double z_rel = world.z() - robot_z_;
            if (z_rel < z_min || z_rel > z_max) {
                continue;
            }
            UpdateMissRay(ox, oy, world.x(), world.y());
            UpdateHit(world.x(), world.y());
        }
    }
    SyncCostmapFromLogOdds();
    RebuildInflation();
}

void PlanningEnv::MarkCoveredFromExtrinsic(
    const automsgs::msgs::geometry_msgs::Transform& map_t_camera)
{
    const unsigned int size_x = costmap_.getSizeInCellsX();
    const unsigned int size_y = costmap_.getSizeInCellsY();
    const float soft_miss =
        options_.occupancy().log_odds_miss() != 0.0
            ? static_cast<float>(options_.occupancy().log_odds_miss()) * 0.5f
            : -0.2f;
    for (unsigned int my = 0; my < size_y; ++my) {
        for (unsigned int mx = 0; mx < size_x; ++mx) {
            double wx = 0.0;
            double wy = 0.0;
            costmap_.mapToWorld(mx, my, wx, wy);
            if (!camera_.IsVisible(map_t_camera, wx, wy, robot_z_, this)) {
                continue;
            }
            const size_t idx =
                static_cast<size_t>(my) * size_x + static_cast<size_t>(mx);
            // Soft-clear unknown in FoV; mark covered only after evidence of
            // free so FoV coverage does not erase free↔unknown frontiers.
            if (log_odds_[idx] == 0.f) {
                ApplyLogOdds(idx, soft_miss);
            }
            const float free_thr =
                options_.occupancy().log_odds_free_threshold() != 0.0
                    ? static_cast<float>(
                          options_.occupancy().log_odds_free_threshold())
                    : -0.4f;
            if (log_odds_[idx] != 0.f && log_odds_[idx] <= free_thr) {
                covered_[idx] = 1;
            }
        }
    }
    SyncCostmapFromLogOdds();
    RebuildInflation();
}

void PlanningEnv::MarkCoveredFromTf()
{
    // Prefer the last extrinsic from UpdateDepth — never look up TF per cell.
    if (!has_map_t_camera_) {
        return;
    }
    MarkCoveredFromExtrinsic(last_map_t_camera_);
}

void PlanningEnv::ExtractFrontiers(
    std::vector<automsgs::msgs::geometry_msgs::Point>* frontiers) const
{
    frontiers->clear();
    const unsigned int size_x = costmap_.getSizeInCellsX();
    const unsigned int size_y = costmap_.getSizeInCellsY();
    static const int kDx[] = {-1, 1, 0, 0};
    static const int kDy[] = {0, 0, -1, 1};

    for (unsigned int my = 1; my + 1 < size_y; ++my) {
        for (unsigned int mx = 1; mx + 1 < size_x; ++mx) {
            if (costmap_.getCost(mx, my) != NO_INFORMATION) {
                continue;
            }
            bool adj_free = false;
            for (int k = 0; k < 4; ++k) {
                const unsigned int nx = static_cast<unsigned int>(
                    static_cast<int>(mx) + kDx[k]);
                const unsigned int ny = static_cast<unsigned int>(
                    static_cast<int>(my) + kDy[k]);
                if (costmap_.getCost(nx, ny) == FREE_SPACE) {
                    adj_free = true;
                    break;
                }
            }
            if (!adj_free) {
                continue;
            }
            automsgs::msgs::geometry_msgs::Point p;
            double wx = 0.0;
            double wy = 0.0;
            costmap_.mapToWorld(mx, my, wx, wy);
            p.set_x(wx);
            p.set_y(wy);
            p.set_z(robot_z_);
            frontiers->push_back(p);
        }
    }
}

void PlanningEnv::ClusterFrontiers(
    const std::vector<automsgs::msgs::geometry_msgs::Point>& frontiers,
    std::vector<automsgs::msgs::geometry_msgs::Point>* targets,
    std::vector<bool>* uncovered) const
{
    targets->clear();
    uncovered->clear();
    const double cluster_dist = options_.frontier_cluster_dist() > 0.0
                                    ? options_.frontier_cluster_dist()
                                    : 1.0;
    const double cluster_dist2 = cluster_dist * cluster_dist;
    std::vector<bool> used(frontiers.size(), false);
    for (size_t i = 0; i < frontiers.size(); ++i) {
        if (used[i]) {
            continue;
        }
        double sx = 0.0;
        double sy = 0.0;
        double sz = 0.0;
        int count = 0;
        std::vector<size_t> stack{i};
        used[i] = true;
        while (!stack.empty()) {
            const size_t cur = stack.back();
            stack.pop_back();
            sx += frontiers[cur].x();
            sy += frontiers[cur].y();
            sz += frontiers[cur].z();
            ++count;
            for (size_t j = 0; j < frontiers.size(); ++j) {
                if (used[j]) {
                    continue;
                }
                const double dx = frontiers[j].x() - frontiers[cur].x();
                const double dy = frontiers[j].y() - frontiers[cur].y();
                if (dx * dx + dy * dy <= cluster_dist2) {
                    used[j] = true;
                    stack.push_back(j);
                }
            }
        }
        if (count <= 0) {
            continue;
        }
        automsgs::msgs::geometry_msgs::Point c;
        c.set_x(sx / count);
        c.set_y(sy / count);
        c.set_z(sz / count);
        targets->push_back(c);
        // Frontier centroids are always exploration goals (uncovered).
        uncovered->push_back(true);
    }
}

void PlanningEnv::ExtractUnknownTargets(
    std::vector<automsgs::msgs::geometry_msgs::Point>* targets,
    std::vector<bool>* uncovered) const
{
    targets->clear();
    uncovered->clear();
    const unsigned int size_x = costmap_.getSizeInCellsX();
    const unsigned int size_y = costmap_.getSizeInCellsY();
    const int stride = 2;
    for (unsigned int my = 0; my < size_y; my += stride) {
        for (unsigned int mx = 0; mx < size_x; mx += stride) {
            if (costmap_.getCost(mx, my) != NO_INFORMATION) {
                continue;
            }
            const size_t idx =
                static_cast<size_t>(my) * size_x + static_cast<size_t>(mx);
            if (idx < covered_.size() && covered_[idx] != 0) {
                continue;
            }
            automsgs::msgs::geometry_msgs::Point p;
            double wx = 0.0;
            double wy = 0.0;
            costmap_.mapToWorld(mx, my, wx, wy);
            p.set_x(wx);
            p.set_y(wy);
            p.set_z(robot_z_);
            targets->push_back(p);
            uncovered->push_back(true);
        }
    }
}

void PlanningEnv::UpdateCoverageFromTf()
{
    MarkCoveredFromTf();
    RefreshTargets();
}

void PlanningEnv::RefreshTargets()
{
    targets_.clear();
    uncovered_.clear();
    frontiers_.clear();

    if (options_.use_frontier()) {
        ExtractFrontiers(&frontiers_);
        ClusterFrontiers(frontiers_, &targets_, &uncovered_);
    } else {
        ExtractUnknownTargets(&targets_, &uncovered_);
    }
}

bool PlanningEnv::IsInExplorationArea(double x, double y) const
{
    if (exploration_polygon_.num_points() < 3) {
        return true;
    }
    return exploration_polygon_.IsPointIn(
        ::autonomy::common::math::Vec2d(x, y));
}

double PlanningEnv::PlanPathAStar(
    double from_x, double from_y, double to_x, double to_y,
    std::vector<automsgs::msgs::geometry_msgs::Point>* path) const
{
    if (path) {
        path->clear();
    }
    unsigned int sx = 0;
    unsigned int sy = 0;
    unsigned int gx = 0;
    unsigned int gy = 0;
    if (!inflated_.worldToMap(from_x, from_y, sx, sy) ||
        !inflated_.worldToMap(to_x, to_y, gx, gy)) {
        return std::numeric_limits<double>::infinity();
    }
    if (inflated_.getCost(sx, sy) >= INSCRIBED_INFLATED_OBSTACLE ||
        inflated_.getCost(gx, gy) >= INSCRIBED_INFLATED_OBSTACLE) {
        return std::numeric_limits<double>::infinity();
    }

    const int size_x = static_cast<int>(inflated_.getSizeInCellsX());
    const int size_y = static_cast<int>(inflated_.getSizeInCellsY());
    const int start = static_cast<int>(sy) * size_x + static_cast<int>(sx);
    const int goal = static_cast<int>(gy) * size_x + static_cast<int>(gx);
    const double res = inflated_.getResolution();

    auto heuristic = [&](int idx) {
        const int x = idx % size_x;
        const int y = idx / size_x;
        return res * std::hypot(static_cast<double>(x - static_cast<int>(gx)),
                                static_cast<double>(y - static_cast<int>(gy)));
    };

    const double inf = std::numeric_limits<double>::infinity();
    std::vector<double> g(static_cast<size_t>(size_x * size_y), inf);
    std::vector<int> parent(static_cast<size_t>(size_x * size_y), -1);
    using QItem = std::pair<double, int>;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;
    g[static_cast<size_t>(start)] = 0.0;
    pq.emplace(heuristic(start), start);

    static const int kDx[] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int kDy[] = {0, 0, 1, -1, 1, -1, 1, -1};
    static const double kCost[] = {1, 1, 1, 1, 1.4142, 1.4142, 1.4142, 1.4142};

    while (!pq.empty()) {
        const auto [f, u] = pq.top();
        pq.pop();
        (void)f;
        if (u == goal) {
            break;
        }
        if (g[static_cast<size_t>(u)] == inf) {
            continue;
        }
        const int ux = u % size_x;
        const int uy = u / size_x;
        for (int k = 0; k < 8; ++k) {
            const int nx = ux + kDx[k];
            const int ny = uy + kDy[k];
            if (nx < 0 || ny < 0 || nx >= size_x || ny >= size_y) {
                continue;
            }
            if (inflated_.getCost(static_cast<unsigned int>(nx),
                                  static_cast<unsigned int>(ny)) >=
                INSCRIBED_INFLATED_OBSTACLE) {
                continue;
            }
            const int v = ny * size_x + nx;
            const double ng = g[static_cast<size_t>(u)] + kCost[k] * res;
            if (ng < g[static_cast<size_t>(v)]) {
                g[static_cast<size_t>(v)] = ng;
                parent[static_cast<size_t>(v)] = u;
                pq.emplace(ng + heuristic(v), v);
            }
        }
    }

    if (g[static_cast<size_t>(goal)] == inf) {
        return inf;
    }
    if (path) {
        std::vector<int> rev;
        for (int cur = goal; cur >= 0; cur = parent[static_cast<size_t>(cur)]) {
            rev.push_back(cur);
            if (cur == start) {
                break;
            }
        }
        std::reverse(rev.begin(), rev.end());
        path->reserve(rev.size());
        for (int idx : rev) {
            const unsigned int mx = static_cast<unsigned int>(idx % size_x);
            const unsigned int my = static_cast<unsigned int>(idx / size_x);
            automsgs::msgs::geometry_msgs::Point p;
            double wx = 0.0;
            double wy = 0.0;
            inflated_.mapToWorld(mx, my, wx, wy);
            p.set_x(wx);
            p.set_y(wy);
            p.set_z(robot_z_);
            path->push_back(p);
        }
    }
    return g[static_cast<size_t>(goal)];
}

automsgs::msgs::map_msgs::OccupancyGrid PlanningEnv::GetOccupancyGrid(
    const std::string& frame_id) const
{
    automsgs::msgs::map_msgs::OccupancyGrid grid;
    grid.mutable_header()->set_frame_id(frame_id);
    grid.mutable_info()->set_resolution(static_cast<float>(costmap_.getResolution()));
    grid.mutable_info()->set_width(costmap_.getSizeInCellsX());
    grid.mutable_info()->set_height(costmap_.getSizeInCellsY());
    grid.mutable_info()->mutable_origin()->mutable_position()->set_x(costmap_.getOriginX());
    grid.mutable_info()->mutable_origin()->mutable_position()->set_y(costmap_.getOriginY());
    grid.mutable_info()->mutable_origin()->mutable_position()->set_z(0.0);
    grid.mutable_info()->mutable_origin()->mutable_orientation()->set_x(0.0);
    grid.mutable_info()->mutable_origin()->mutable_orientation()->set_y(0.0);
    grid.mutable_info()->mutable_origin()->mutable_orientation()->set_z(0.0);
    grid.mutable_info()->mutable_origin()->mutable_orientation()->set_w(1.0);

    const size_t cell_count =
        static_cast<size_t>(grid.info().width()) * grid.info().height();
    grid.mutable_data()->Resize(static_cast<int>(cell_count), 0);
    for (size_t i = 0; i < cell_count; ++i) {
        grid.mutable_data()->Set(
            static_cast<int>(i),
            CostCellToOccupancyEquivalent(
                costmap_.getCost(static_cast<unsigned int>(i))));
    }
    return grid;
}

}  // namespace exploration
}  // namespace autonomy
