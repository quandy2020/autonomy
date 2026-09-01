/*
 * Copyright 2026 The Openbot Authors (duyongquan)
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

#pragma once

#include <cmath>
#include <cstdint>
#include <optional>

#include <algorithm>

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/utils/occ_grid_values.hpp"

namespace autonomy::task::teleop {

/**
 * @brief Default lethal threshold (Nav2 inscribed inflated obstacle)
 */
inline unsigned char DefaultObstacleCost() {
    return map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

/**
 * @brief True when cost exceeds obstacle threshold
 */
inline bool IsPathBlockedCost(unsigned char cost,
                              unsigned char threshold =
                                  DefaultObstacleCost()) {
    return cost >= threshold && cost < map::costmap_2d::NO_INFORMATION;
}

/**
 * @class teleop::PathObstacleGrid
 * @brief Abstract obstacle query interface for path selection and visualization
 */
class PathObstacleGrid
{
public:
    virtual ~PathObstacleGrid() = default;

    // True when world point lies inside the grid.
    virtual bool InBounds(double wx, double wy) const = 0;
    // True when world point is on a lethal cell.
    virtual bool IsLethal(double wx, double wy) const = 0;
    // Raw costmap cost at world point.
    virtual unsigned char CostAt(double wx, double wy) const = 0;
};

/**
 * @brief True when in-bounds cell is lethal
 */
inline bool IsPathBlocked(const PathObstacleGrid& grid, double wx, double wy) {
    return grid.InBounds(wx, wy) && grid.IsLethal(wx, wy);
}

/**
 * @brief True when (wx, wy) lies outside the grid bounds
 */
inline bool IsPathOutOfBounds(const PathObstacleGrid& grid, double wx,
                              double wy) {
    return !grid.InBounds(wx, wy);
}

/**
 * @brief True when cell is lethal or outside the grid (viz clip boundary)
 */
inline bool BlockedOrOob(const PathObstacleGrid& grid, double wx,
                                       double wy) {
    if (!grid.InBounds(wx, wy)) {
        return true;
    }
    return grid.IsLethal(wx, wy);
}

/**
 * @brief Ray-cast in base_link; true if any sample is blocked or OOB
 */
inline bool IsRayBlocked(const PathObstacleGrid& grid, double dir_deg,
                           double start_dist, double end_dist,
                           double step = 0.05) {
    if (end_dist <= start_dist) {
        return false;
    }
    constexpr double kPi = 3.14159265358979323846;
    const double rad = dir_deg * kPi / 180.0;
    const double cos_dir = std::cos(rad);
    const double sin_dir = std::sin(rad);
    const double sample_step = std::max(0.01, step);
    for (double d = start_dist; d <= end_dist + 1e-6; d += sample_step) {
        if (BlockedOrOob(grid, d * cos_dir, d * sin_dir)) {
            return true;
        }
    }
    return false;
}

/**
 * @brief True when any path sample hits a lethal costmap cell
 */
inline bool HasLethalHit(const PathObstacleGrid& grid,
                             const automsgs::msgs::nav_msgs::Path& path,
                             double sample_step = 0.05) {
    if (path.poses_size() == 0) {
        return false;
    }
    const double step = std::max(0.01, sample_step);
    for (int i = 0; i < path.poses_size(); ++i) {
        const double wx = path.poses(i).pose().position().x();
        const double wy = path.poses(i).pose().position().y();

        if (i > 0) {
            const auto& prev = path.poses(i - 1);
            const double px = prev.pose().position().x();
            const double py = prev.pose().position().y();
            const double dx = wx - px;
            const double dy = wy - py;
            const double seg_len = std::hypot(dx, dy);
            if (seg_len > 1e-6) {
                for (double traveled = step; traveled < seg_len;
                     traveled += step) {
                    const double t = traveled / seg_len;
                    if (IsPathBlocked(grid, px + dx * t, py + dy * t)) {
                        return true;
                    }
                }
            }
        }

        if (IsPathBlocked(grid, wx, wy)) {
            return true;
        }
    }
    return false;
}

/**
 * @brief True when entire path is in-bounds and collision-free
 */
inline bool FullyTraversable(const PathObstacleGrid& grid,
                                   const automsgs::msgs::nav_msgs::Path& path,
                                   double sample_step = 0.05) {
    if (path.poses_size() == 0) {
        return false;
    }
    const double step = std::max(0.01, sample_step);
    for (int i = 0; i < path.poses_size(); ++i) {
        const double wx = path.poses(i).pose().position().x();
        const double wy = path.poses(i).pose().position().y();

        if (i > 0) {
            const auto& prev = path.poses(i - 1);
            const double px = prev.pose().position().x();
            const double py = prev.pose().position().y();
            const double dx = wx - px;
            const double dy = wy - py;
            const double seg_len = std::hypot(dx, dy);
            if (seg_len > 1e-6) {
                for (double traveled = step; traveled < seg_len;
                     traveled += step) {
                    const double t = traveled / seg_len;
                    const double sx = px + dx * t;
                    const double sy = py + dy * t;
                    if (!grid.InBounds(sx, sy) ||
                        IsPathBlocked(grid, sx, sy)) {
                        return false;
                    }
                }
            }
        }

        if (!grid.InBounds(wx, wy) || IsPathBlocked(grid, wx, wy)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Sum of segment lengths along path poses
 */
inline double PathArcLength(const automsgs::msgs::nav_msgs::Path& path) {
    double length = 0.0;
    for (int i = 1; i < path.poses_size(); ++i) {
        const auto& prev = path.poses(i - 1).pose().position();
        const auto& curr = path.poses(i).pose().position();
        length += std::hypot(curr.x() - prev.x(), curr.y() - prev.y());
    }
    return length;
}

/**
 * @brief Mean inflation cost along path in [0, 255]
 */
inline double PathMeanCost(const PathObstacleGrid& grid,
                           const automsgs::msgs::nav_msgs::Path& path,
                           double sample_step = 0.05) {
    if (path.poses_size() == 0) {
        return 0.0;
    }
    const double step = std::max(0.01, sample_step);
    double sum = 0.0;
    int count = 0;
    for (int i = 0; i < path.poses_size(); ++i) {
        const double wx = path.poses(i).pose().position().x();
        const double wy = path.poses(i).pose().position().y();
        if (i > 0) {
            const auto& prev = path.poses(i - 1);
            const double px = prev.pose().position().x();
            const double py = prev.pose().position().y();
            const double dx = wx - px;
            const double dy = wy - py;
            const double seg_len = std::hypot(dx, dy);
            if (seg_len > 1e-6) {
                for (double traveled = step; traveled < seg_len;
                     traveled += step) {
                    const double t = traveled / seg_len;
                    const double sx = px + dx * t;
                    const double sy = py + dy * t;
                    if (grid.InBounds(sx, sy)) {
                        sum += static_cast<double>(grid.CostAt(sx, sy));
                        ++count;
                    }
                }
            }
        }
        if (grid.InBounds(wx, wy)) {
            sum += static_cast<double>(grid.CostAt(wx, wy));
            ++count;
        }
    }
    return count > 0 ? sum / static_cast<double>(count) : 0.0;
}

inline double PathClearancePenalty(const PathObstacleGrid& grid,
                                   const automsgs::msgs::nav_msgs::Path& path,
                                   double sample_step = 0.05) {
    constexpr double kMaxCost = 253.0;
    return std::clamp(PathMeanCost(grid, path, sample_step) / kMaxCost, 0.0,
                      1.0);
}

inline double PathSmoothnessPenalty(
    const automsgs::msgs::nav_msgs::Path& path) {
    if (path.poses_size() < 3) {
        return 0.0;
    }
    double max_delta_deg = 0.0;
    double prev_yaw = 0.0;
    bool has_prev = false;
    for (int i = 1; i < path.poses_size(); ++i) {
        const auto& prev = path.poses(i - 1).pose().position();
        const auto& curr = path.poses(i).pose().position();
        const double dx = curr.x() - prev.x();
        const double dy = curr.y() - prev.y();
        if (std::hypot(dx, dy) < 1e-6) {
            continue;
        }
        const double yaw = std::atan2(dy, dx);
        if (has_prev) {
            double delta = std::abs(yaw - prev_yaw) * 180.0 / 3.141592653589793;
            if (delta > 180.0) {
                delta = 360.0 - delta;
            }
            max_delta_deg = std::max(max_delta_deg, delta);
        }
        prev_yaw = yaw;
        has_prev = true;
    }
    return std::clamp(max_delta_deg / 45.0, 0.0, 1.0);
}

inline double PathEfficiencyPenalty(
    const automsgs::msgs::nav_msgs::Path& path, double active_range) {
    if (active_range <= 1e-6) {
        return 0.0;
    }
    return std::clamp(PathArcLength(path) / active_range - 0.5, 0.0, 1.0);
}

inline double VelContinuityPenalty(
    const automsgs::msgs::nav_msgs::Path& path, double robot_vx,
    double robot_wz) {
    if (path.poses_size() < 2) {
        return 0.0;
    }
    const auto& p0 = path.poses(0).pose().position();
    const auto& p1 = path.poses(1).pose().position();
    const double dx = p1.x() - p0.x();
    const double dy = p1.y() - p0.y();
    if (std::hypot(dx, dy) < 1e-6) {
        return 0.0;
    }
    const double path_heading = std::atan2(dy, dx);
    double vel_heading = 0.0;
    if (std::hypot(robot_vx, robot_wz) > 0.02) {
        vel_heading = std::atan2(robot_wz, robot_vx);
    } else {
        return 0.0;
    }
    double diff = std::abs(path_heading - vel_heading) * 180.0 / 3.141592653589793;
    if (diff > 180.0) {
        diff = 360.0 - diff;
    }
    return std::clamp(diff / 90.0, 0.0, 1.0);
}

/**
 * @brief Mean traversability along path in [0, 1]
 */
inline double PathMeanTraversability(const PathObstacleGrid& grid,
                                     const automsgs::msgs::nav_msgs::Path& path,
                                     double sample_step = 0.05) {
    if (path.poses_size() == 0) {
        return 0.0;
    }
    const double step = std::max(0.01, sample_step);
    double sum = 0.0;
    int count = 0;
    for (int i = 0; i < path.poses_size(); ++i) {
        const double wx = path.poses(i).pose().position().x();
        const double wy = path.poses(i).pose().position().y();
        if (i > 0) {
            const auto& prev = path.poses(i - 1);
            const double px = prev.pose().position().x();
            const double py = prev.pose().position().y();
            const double dx = wx - px;
            const double dy = wy - py;
            const double seg_len = std::hypot(dx, dy);
            if (seg_len > 1e-6) {
                for (double traveled = step; traveled < seg_len;
                     traveled += step) {
                    const double t = traveled / seg_len;
                    const double sx = px + dx * t;
                    const double sy = py + dy * t;
                    if (grid.InBounds(sx, sy)) {
                        const unsigned char cost = grid.CostAt(sx, sy);
                        if (IsPathBlockedCost(cost)) {
                            sum += 0.0;
                        } else {
                            sum += 1.0 - static_cast<double>(cost) / 253.0;
                        }
                        ++count;
                    }
                }
            }
        }
        if (grid.InBounds(wx, wy)) {
            const unsigned char cost = grid.CostAt(wx, wy);
            if (IsPathBlockedCost(cost)) {
                sum += 0.0;
            } else {
                sum += 1.0 - static_cast<double>(cost) / 253.0;
            }
            ++count;
        }
    }
    return count > 0 ? sum / static_cast<double>(count) : 0.0;
}

/**
 * @brief True when no path sample hits an obstacle cell
 */
inline bool IsPathCollisionFree(const PathObstacleGrid& grid,
                                const automsgs::msgs::nav_msgs::Path& path,
                                double sample_step = 0.05) {
    if (path.poses_size() == 0) {
        return false;
    }
    const double step = std::max(0.01, sample_step);
    for (int i = 0; i < path.poses_size(); ++i) {
        const auto& pose_stamped = path.poses(i);
        const double wx = pose_stamped.pose().position().x();
        const double wy = pose_stamped.pose().position().y();

        if (i > 0) {
            const auto& prev = path.poses(i - 1);
            const double px = prev.pose().position().x();
            const double py = prev.pose().position().y();
            const double dx = wx - px;
            const double dy = wy - py;
            const double seg_len = std::hypot(dx, dy);
            if (seg_len > 1e-6) {
                for (double traveled = step; traveled < seg_len; traveled += step) {
                    const double t = traveled / seg_len;
                    if (IsPathBlocked(grid, px + dx * t, py + dy * t)) {
                        return false;
                    }
                }
            }
        }

        if (IsPathBlocked(grid, wx, wy)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Clip path at first lethal cell along segments
 */
inline automsgs::msgs::nav_msgs::Path ClipAtObstacle(
    const PathObstacleGrid& grid,
    const automsgs::msgs::nav_msgs::Path& path,
    double sample_step = 0.05) {
    automsgs::msgs::nav_msgs::Path clipped;
    if (path.poses_size() == 0) {
        return clipped;
    }
    *clipped.mutable_header() = path.header();
    const double step = std::max(0.01, sample_step);

    for (int i = 0; i < path.poses_size(); ++i) {
        const auto& pose_stamped = path.poses(i);
        const double wx = pose_stamped.pose().position().x();
        const double wy = pose_stamped.pose().position().y();

        if (i > 0) {
            const auto& prev = path.poses(i - 1);
            const double px = prev.pose().position().x();
            const double py = prev.pose().position().y();
            const double dx = wx - px;
            const double dy = wy - py;
            const double seg_len = std::hypot(dx, dy);
            if (seg_len > 1e-6) {
                for (double traveled = step; traveled < seg_len; traveled += step) {
                    const double t = traveled / seg_len;
                    if (IsPathBlocked(grid, px + dx * t, py + dy * t)) {
                        if (clipped.poses_size() == 0) {
                            *clipped.add_poses() = path.poses(0);
                        }
                        return clipped;
                    }
                }
            }
        }

        if (IsPathBlocked(grid, wx, wy)) {
            break;
        }
        *clipped.add_poses() = pose_stamped;
    }
    if (clipped.poses_size() == 0) {
        *clipped.add_poses() = path.poses(0);
    }
    return clipped;
}

/**
 * @brief Clip path at lethal cells and costmap boundary (viz)
 */
inline automsgs::msgs::nav_msgs::Path ClipForViz(
    const PathObstacleGrid& grid,
    const automsgs::msgs::nav_msgs::Path& path,
    double sample_step = 0.02) {
    automsgs::msgs::nav_msgs::Path clipped;
    if (path.poses_size() == 0) {
        return clipped;
    }
    *clipped.mutable_header() = path.header();
    const double step = std::max(0.01, sample_step);

    for (int i = 0; i < path.poses_size(); ++i) {
        const auto& pose_stamped = path.poses(i);
        const double wx = pose_stamped.pose().position().x();
        const double wy = pose_stamped.pose().position().y();

        if (i > 0) {
            const auto& prev = path.poses(i - 1);
            const double px = prev.pose().position().x();
            const double py = prev.pose().position().y();
            const double dx = wx - px;
            const double dy = wy - py;
            const double seg_len = std::hypot(dx, dy);
            if (seg_len > 1e-6) {
                for (double traveled = step; traveled < seg_len;
                     traveled += step) {
                    const double t = traveled / seg_len;
                    if (BlockedOrOob(grid, px + dx * t,
                                                     py + dy * t)) {
                        if (clipped.poses_size() == 0) {
                            *clipped.add_poses() = path.poses(0);
                        }
                        return clipped;
                    }
                }
            }
        }

        if (BlockedOrOob(grid, wx, wy)) {
            break;
        }
        *clipped.add_poses() = pose_stamped;
    }
    if (clipped.poses_size() == 0 && path.poses_size() > 0) {
        *clipped.add_poses() = path.poses(0);
    }
    return clipped;
}

/**
 * @class teleop::CostmapObstacleGrid
 * @brief PathObstacleGrid adapter for a rolling Costmap2D (base_link frame)
 */
class CostmapObstacleGrid final : public PathObstacleGrid
{
public:
    explicit CostmapObstacleGrid(
        const map::costmap_2d::Costmap2D& costmap,
        unsigned char obstacle_cost_threshold =
            DefaultObstacleCost())
        : costmap_(costmap), threshold_(obstacle_cost_threshold) {}

    bool InBounds(double wx, double wy) const override
    {
        unsigned int mx = 0;
        unsigned int my = 0;
        return costmap_.worldToMap(wx, wy, mx, my);
    }

    bool IsLethal(double wx, double wy) const override
    {
        return IsPathBlockedCost(CostAt(wx, wy), threshold_);
    }

    unsigned char CostAt(double wx, double wy) const override
    {
        unsigned int mx = 0;
        unsigned int my = 0;
        if (!costmap_.worldToMap(wx, wy, mx, my)) {
            return map::costmap_2d::NO_INFORMATION;
        }
        return costmap_.getCost(mx, my);
    }

private:
    const map::costmap_2d::Costmap2D& costmap_;
    unsigned char threshold_;
};

/**
 * @class teleop::RobotFrameCostmapObstacleGrid
 * @brief Queries costmap in map frame while path points stay in robot frame
 */
class RobotFrameCostmapObstacleGrid final : public PathObstacleGrid
{
public:
    RobotFrameCostmapObstacleGrid(const map::costmap_2d::Costmap2D& costmap,
                                    double robot_x, double robot_y,
                                    double robot_yaw)
        : costmap_(costmap),
          robot_x_(robot_x),
          robot_y_(robot_y),
          cos_yaw_(std::cos(robot_yaw)),
          sin_yaw_(std::sin(robot_yaw)) {}

    bool InBounds(double lx, double ly) const override
    {
        double wx = 0.0;
        double wy = 0.0;
        if (!ToMap(lx, ly, &wx, &wy)) {
            return false;
        }
        unsigned int mx = 0;
        unsigned int my = 0;
        return costmap_.worldToMap(wx, wy, mx, my);
    }

    bool IsLethal(double lx, double ly) const override
    {
        return IsPathBlockedCost(CostAt(lx, ly));
    }

    unsigned char CostAt(double lx, double ly) const override
    {
        double wx = 0.0;
        double wy = 0.0;
        if (!ToMap(lx, ly, &wx, &wy)) {
            return map::costmap_2d::NO_INFORMATION;
        }
        unsigned int mx = 0;
        unsigned int my = 0;
        if (!costmap_.worldToMap(wx, wy, mx, my)) {
            return map::costmap_2d::NO_INFORMATION;
        }
        return costmap_.getCost(mx, my);
    }

private:
    bool ToMap(double lx, double ly, double* wx, double* wy) const
    {
        if (!wx || !wy) {
            return false;
        }
        *wx = robot_x_ + cos_yaw_ * lx - sin_yaw_ * ly;
        *wy = robot_y_ + sin_yaw_ * lx + cos_yaw_ * ly;
        return true;
    }

    const map::costmap_2d::Costmap2D& costmap_;
    double robot_x_{0.0};
    double robot_y_{0.0};
    double cos_yaw_{1.0};
    double sin_yaw_{0.0};
};

/**
 * @class teleop::OccupancyGridObstacleGrid
 * @brief PathObstacleGrid adapter for nav_msgs/OccupancyGrid maps
 */
class OccupancyGridObstacleGrid final : public PathObstacleGrid
{
public:
    OccupancyGridObstacleGrid(
        const automsgs::msgs::map_msgs::OccupancyGrid& grid,
        int8_t lethal_threshold =
            map::costmap_2d::utils::OCC_GRID_OCCUPIED / 2)
        : grid_(grid), lethal_threshold_(lethal_threshold) {}

    bool InBounds(double wx, double wy) const override
    {
        return WorldToCell(wx, wy).has_value();
    }

    bool IsLethal(double wx, double wy) const override
    {
        const auto cell = CellAt(wx, wy);
        if (!cell.has_value()) {
            return true;
        }
        if (*cell < 0) {
            return false;
        }
        return *cell >= lethal_threshold_;
    }

    unsigned char CostAt(double wx, double wy) const override
    {
        const auto cell = CellAt(wx, wy);
        if (!cell.has_value()) {
            return map::costmap_2d::NO_INFORMATION;
        }
        if (*cell < 0) {
            return map::costmap_2d::FREE_SPACE;
        }
        if (*cell >= map::costmap_2d::utils::OCC_GRID_OCCUPIED) {
            return map::costmap_2d::LETHAL_OBSTACLE;
        }
        return static_cast<unsigned char>(std::clamp(
            static_cast<int>(*cell) * static_cast<int>(map::costmap_2d::LETHAL_OBSTACLE) /
                static_cast<int>(map::costmap_2d::utils::OCC_GRID_OCCUPIED),
            0, static_cast<int>(map::costmap_2d::MAX_NON_OBSTACLE)));
    }

private:
    struct CellIndex {
        int x;
        int y;
    };

    std::optional<CellIndex> WorldToCell(double wx, double wy) const
    {
        if (grid_.info().resolution() <= 0.0) {
            return std::nullopt;
        }
        const double origin_x = grid_.info().origin().position().x();
        const double origin_y = grid_.info().origin().position().y();
        const int mx = static_cast<int>(std::floor((wx - origin_x) /
                                                   grid_.info().resolution()));
        const int my = static_cast<int>(std::floor((wy - origin_y) /
                                                   grid_.info().resolution()));
        if (mx < 0 || my < 0 ||
            mx >= static_cast<int>(grid_.info().width()) ||
            my >= static_cast<int>(grid_.info().height())) {
            return std::nullopt;
        }
        return CellIndex{mx, my};
    }

    std::optional<int8_t> CellAt(double wx, double wy) const
    {
        const auto index = WorldToCell(wx, wy);
        if (!index.has_value()) {
            return std::nullopt;
        }
        const int linear =
            index->y * static_cast<int>(grid_.info().width()) + index->x;
        if (linear < 0 || linear >= grid_.data_size()) {
            return std::nullopt;
        }
        return static_cast<int8_t>(grid_.data(linear));
    }

    const automsgs::msgs::map_msgs::OccupancyGrid& grid_;
    int8_t lethal_threshold_;
};

}  // namespace autonomy::task::teleop
