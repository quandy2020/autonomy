/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/planning/planner/theta_star/theta_star_planner.hpp"

#include "autolink/plugin_manager/plugin_manager.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"

namespace autonomy {
namespace planning {
namespace planner {
namespace theta_star {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

struct QueueNode {
    double f;
    int index;

    bool operator>(const QueueNode& other) const {
        return f > other.f;
    }
};

inline int ToIndex(unsigned int x, unsigned int y, unsigned int size_x) {
    return static_cast<int>(y * size_x + x);
}

}  // namespace

ThetaStarPlanner::ThetaStarPlanner() = default;

ThetaStarPlanner::~ThetaStarPlanner() {
    Cleanup();
}

bool ThetaStarPlanner::Configure(
    const proto::PlannerOptions& options, const std::string& name,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap) {
    name_ = name;
    costmap_ = costmap;

    const auto& cfg = options.theta_star();
    how_many_corners_ = cfg.how_many_corners() > 0 ? cfg.how_many_corners() : 8;
    if (how_many_corners_ != 4 && how_many_corners_ != 8) {
        AWARN << "ThetaStarPlanner how_many_corners must be 4 or 8, using 8";
        how_many_corners_ = 8;
    }
    allow_unknown_ = cfg.allow_unknown();
    w_euc_cost_ = cfg.w_euc_cost() > 0.0 ? cfg.w_euc_cost() : 2.0;
    w_traversal_cost_ =
        cfg.w_traversal_cost() > 0.0 ? cfg.w_traversal_cost() : 1.0;
    w_heuristic_cost_ = cfg.w_heuristic_cost() > 0.0 ? cfg.w_heuristic_cost()
                                                     : (w_euc_cost_ < 1.0
                                                            ? w_euc_cost_
                                                            : 1.0);
    terminal_checking_interval_ = cfg.terminal_checking_interval() > 0
                                      ? cfg.terminal_checking_interval()
                                      : 5000;
    return true;
}

void ThetaStarPlanner::Cleanup() {
    planning_costmap_copy_.clear();
    costmap_.reset();
}

void ThetaStarPlanner::Activate() {}

void ThetaStarPlanner::Deactivate() {}

uint32 ThetaStarPlanner::CreatePlan(
    const commsgs::geometry_msgs::PoseStamped& start,
    const commsgs::geometry_msgs::PoseStamped& goal,
    commsgs::planning_msgs::Path& plan,
    std::function<bool()> cancel_checker) {
    if (!costmap_) {
        AERROR << "Costmap is not set for planner " << name_;
        return static_cast<uint32>(
            proto::PlannerResultCode::PLANNER_NOT_INITIALIZED);
    }

    if (global_frame_.empty() && !start.header.frame_id.empty()) {
        global_frame_ = start.header.frame_id;
    }

    plan.poses.clear();
    if (makePlan(start.pose, goal.pose, cancel_checker, plan)) {
        return static_cast<uint32>(proto::PlannerResultCode::PLANNER_SUCCESS);
    }
    return static_cast<uint32>(proto::PlannerResultCode::PLANNER_NO_PATH_FOUND);
}

bool ThetaStarPlanner::worldToMap(double wx, double wy, unsigned int& mx,
                                  unsigned int& my) const {
    auto* grid = costmap_->getCostmap();
    if (!grid) {
        return false;
    }
    if (wx < grid->getOriginX() || wy < grid->getOriginY()) {
        return false;
    }
    mx = static_cast<unsigned int>((wx - grid->getOriginX()) /
                                   grid->getResolution());
    my = static_cast<unsigned int>((wy - grid->getOriginY()) /
                                   grid->getResolution());
    return mx < grid->getSizeInCellsX() && my < grid->getSizeInCellsY();
}

void ThetaStarPlanner::mapToWorld(unsigned int mx, unsigned int my, double& wx,
                                  double& wy) const {
    auto* grid = costmap_->getCostmap();
    if (!grid) {
        return;
    }
    wx = grid->getOriginX() + mx * grid->getResolution();
    wy = grid->getOriginY() + my * grid->getResolution();
}

bool ThetaStarPlanner::isCellBlocked(unsigned char cost) const {
    if (cost == map::costmap_2d::LETHAL_OBSTACLE ||
        cost == map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return true;
    }
    if (!allow_unknown_ && cost == map::costmap_2d::NO_INFORMATION) {
        return true;
    }
    return false;
}

bool ThetaStarPlanner::lineOfSight(unsigned int x0, unsigned int y0,
                                   unsigned int x1, unsigned int y1) const {
    int x = static_cast<int>(x0);
    int y = static_cast<int>(y0);
    const int x_end = static_cast<int>(x1);
    const int y_end = static_cast<int>(y1);

    const int dx = std::abs(x_end - x);
    const int dy = std::abs(y_end - y);
    const int sx = x < x_end ? 1 : -1;
    const int sy = y < y_end ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (x < 0 || y < 0 || static_cast<unsigned int>(x) >= size_x_ ||
            static_cast<unsigned int>(y) >= size_y_) {
            return false;
        }
        const size_t idx =
            static_cast<size_t>(y) * size_x_ + static_cast<unsigned int>(x);
        if (isCellBlocked(planning_costmap_copy_[idx])) {
            return false;
        }
        if (x == x_end && y == y_end) {
            break;
        }
        const int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
    return true;
}

double ThetaStarPlanner::euclideanDistance(unsigned int x0, unsigned int y0,
                                            unsigned int x1,
                                            unsigned int y1) const {
    const double dx = static_cast<double>(x1) - static_cast<double>(x0);
    const double dy = static_cast<double>(y1) - static_cast<double>(y0);
    return std::hypot(dx, dy);
}

double ThetaStarPlanner::traversalCost(unsigned int x, unsigned int y) const {
    const size_t idx = static_cast<size_t>(y) * size_x_ + x;
    const unsigned char cost = planning_costmap_copy_[idx];
    if (cost == map::costmap_2d::NO_INFORMATION) {
        return w_traversal_cost_;
    }
    return w_traversal_cost_ * (static_cast<double>(cost) / 252.0);
}

double ThetaStarPlanner::heuristic(unsigned int x, unsigned int y,
                                   unsigned int goal_x,
                                   unsigned int goal_y) const {
    return w_heuristic_cost_ * euclideanDistance(x, y, goal_x, goal_y);
}

bool ThetaStarPlanner::makePlan(
    const commsgs::geometry_msgs::Pose& start,
    const commsgs::geometry_msgs::Pose& goal,
    std::function<bool()> cancel_checker, commsgs::planning_msgs::Path& plan) {
    plan.poses.clear();
    plan.header.frame_id = global_frame_.empty() ? "map" : global_frame_;

    auto* grid = costmap_->getCostmap();
    if (!grid) {
        return false;
    }

    unsigned int start_x = 0;
    unsigned int start_y = 0;
    unsigned int goal_x = 0;
    unsigned int goal_y = 0;
    if (!worldToMap(start.position.x, start.position.y, start_x, start_y) ||
        !worldToMap(goal.position.x, goal.position.y, goal_x, goal_y)) {
        AERROR << "Start or goal is outside map bounds";
        return false;
    }

    size_x_ = grid->getSizeInCellsX();
    size_y_ = grid->getSizeInCellsY();
    const size_t map_size = static_cast<size_t>(size_x_) * size_y_;

    std::unique_lock<map::costmap_2d::Costmap2D::mutex_t> lock(
        *(grid->getMutex()));
    planning_costmap_copy_.resize(map_size);
    std::memcpy(planning_costmap_copy_.data(), grid->getCharMap(), map_size);
    planning_costmap_copy_[static_cast<size_t>(start_y) * size_x_ + start_x] =
        map::costmap_2d::FREE_SPACE;
    lock.unlock();

    if (isCellBlocked(
            planning_costmap_copy_[static_cast<size_t>(goal_y) * size_x_ +
                                   goal_x])) {
        AERROR << "Goal cell is blocked";
        return false;
    }

    std::vector<double> g_score(map_size, kInf);
    std::vector<int> parent(map_size, -1);
    std::vector<uint8_t> closed(map_size, 0);

    const int start_idx = ToIndex(start_x, start_y, size_x_);
    const int goal_idx = ToIndex(goal_x, goal_y, size_x_);
    g_score[static_cast<size_t>(start_idx)] = 0.0;
    parent[static_cast<size_t>(start_idx)] = start_idx;

    std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>>
        open;
    open.push({heuristic(start_x, start_y, goal_x, goal_y), start_idx});

    static const int kOffsets4[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    static const int kOffsets8[8][2] = {{1, 0},  {-1, 0}, {0, 1},  {0, -1},
                                        {1, 1},  {1, -1}, {-1, 1}, {-1, -1}};
    const int (*offsets)[2] =
        how_many_corners_ == 4 ? kOffsets4 : kOffsets8;
    const int num_neighbors = how_many_corners_ == 4 ? 4 : 8;

    int expansions = 0;
    while (!open.empty()) {
        if (cancel_checker && ++expansions % terminal_checking_interval_ == 0 &&
            cancel_checker()) {
            AINFO << "ThetaStarPlanner cancelled";
            return false;
        }

        const QueueNode current = open.top();
        open.pop();
        const int curr_idx = current.index;
        if (closed[static_cast<size_t>(curr_idx)]) {
            continue;
        }
        closed[static_cast<size_t>(curr_idx)] = 1;

        if (curr_idx == goal_idx) {
            break;
        }

        const unsigned int cx =
            static_cast<unsigned int>(curr_idx % static_cast<int>(size_x_));
        const unsigned int cy =
            static_cast<unsigned int>(curr_idx / static_cast<int>(size_x_));
        const int par_idx = parent[static_cast<size_t>(curr_idx)];
        const unsigned int px =
            static_cast<unsigned int>(par_idx % static_cast<int>(size_x_));
        const unsigned int py =
            static_cast<unsigned int>(par_idx / static_cast<int>(size_x_));

        for (int n = 0; n < num_neighbors; ++n) {
            const int nx = static_cast<int>(cx) + offsets[n][0];
            const int ny = static_cast<int>(cy) + offsets[n][1];
            if (nx < 0 || ny < 0 || static_cast<unsigned int>(nx) >= size_x_ ||
                static_cast<unsigned int>(ny) >= size_y_) {
                continue;
            }

            const unsigned int unx = static_cast<unsigned int>(nx);
            const unsigned int uny = static_cast<unsigned int>(ny);
            const size_t n_idx =
                static_cast<size_t>(ToIndex(unx, uny, size_x_));
            if (isCellBlocked(planning_costmap_copy_[n_idx])) {
                continue;
            }

            double tentative_g = kInf;
            int new_parent = curr_idx;

            if (lineOfSight(px, py, unx, uny)) {
                tentative_g =
                    g_score[static_cast<size_t>(par_idx)] +
                    w_euc_cost_ * euclideanDistance(px, py, unx, uny) +
                    traversalCost(unx, uny);
                new_parent = par_idx;
            } else {
                tentative_g =
                    g_score[static_cast<size_t>(curr_idx)] +
                    w_euc_cost_ * euclideanDistance(cx, cy, unx, uny) +
                    traversalCost(unx, uny);
                new_parent = curr_idx;
            }

            if (tentative_g >= g_score[n_idx]) {
                continue;
            }

            g_score[n_idx] = tentative_g;
            parent[n_idx] = new_parent;
            const double f = tentative_g + heuristic(unx, uny, goal_x, goal_y);
            open.push({f, static_cast<int>(n_idx)});
        }
    }

    if (g_score[static_cast<size_t>(goal_idx)] >= kInf) {
        AERROR << "ThetaStarPlanner failed to find a path";
        return false;
    }

    std::vector<std::pair<unsigned int, unsigned int>> cells;
    int idx = goal_idx;
    while (true) {
        const unsigned int x =
            static_cast<unsigned int>(idx % static_cast<int>(size_x_));
        const unsigned int y =
            static_cast<unsigned int>(idx / static_cast<int>(size_x_));
        cells.emplace_back(x, y);
        if (idx == start_idx) {
            break;
        }
        idx = parent[static_cast<size_t>(idx)];
        if (idx < 0) {
            return false;
        }
    }
    std::reverse(cells.begin(), cells.end());

    for (size_t i = 0; i < cells.size(); ++i) {
        double wx = 0.0;
        double wy = 0.0;
        mapToWorld(cells[i].first, cells[i].second, wx, wy);

        commsgs::geometry_msgs::PoseStamped pose;
        pose.header.frame_id = plan.header.frame_id;
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;

        if (i + 1 < cells.size()) {
            double nx = 0.0;
            double ny = 0.0;
            mapToWorld(cells[i + 1].first, cells[i + 1].second, nx, ny);
            const double theta = std::atan2(ny - wy, nx - wx);
            pose.pose.orientation =
                map::costmap_2d::utils::OrientationAroundZAxis(theta);
        } else if (!cells.empty() && i > 0) {
            pose.pose.orientation = plan.poses.back().pose.orientation;
        }

        plan.poses.push_back(pose);
    }

    if (!plan.poses.empty()) {
        plan.poses.back().pose.position = goal.position;
        plan.poses.back().pose.orientation = goal.orientation;
    }

    return !plan.poses.empty();
}

}  // namespace theta_star
}  // namespace planner
}  // namespace planning
}  // namespace autonomy

using autonomy::planning::common::GlobalPlanner;
using autonomy::planning::planner::theta_star::ThetaStarPlanner;

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(ThetaStarPlanner, GlobalPlanner);
