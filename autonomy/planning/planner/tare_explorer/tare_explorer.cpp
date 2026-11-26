/* Copyright 2026 The Openbot Authors. */
#include "tare_explorer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <utility>

#ifndef TARE_EXPLORER_CORE_ONLY
#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#endif

namespace autonomy::planning::planner::tare_explorer {
namespace {

#ifdef TARE_EXPLORER_CORE_ONLY
constexpr unsigned char NO_INFORMATION = 255;
constexpr unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
#else
using map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using map::costmap_2d::NO_INFORMATION;
#endif

constexpr std::array<std::pair<int, int>, 8> kNeighbors{{
    {-1, -1}, {0, -1}, {1, -1}, {-1, 0},
    {1, 0},   {-1, 1}, {0, 1},   {1, 1},
}};
constexpr std::array<std::pair<int, int>, 4> kCardinal{{
    {-1, 0}, {1, 0}, {0, -1}, {0, 1},
}};

std::size_t Index(const GridSnapshot& grid, unsigned x, unsigned y) {
    return static_cast<std::size_t>(y) * grid.width + x;
}

bool InBounds(const GridSnapshot& grid, int x, int y) {
    return x >= 0 && y >= 0 && static_cast<unsigned>(x) < grid.width &&
           static_cast<unsigned>(y) < grid.height;
}

bool ValidGrid(const GridSnapshot& grid) {
    return grid.width > 0 && grid.height > 0 &&
           std::isfinite(grid.resolution) && grid.resolution > 0.0 &&
           std::isfinite(grid.origin_x) && std::isfinite(grid.origin_y) &&
           grid.costs.size() == static_cast<std::size_t>(grid.width) * grid.height;
}

bool IsTraversable(const GridSnapshot& grid, GridCell cell,
                   const TareExplorerSettings& settings) {
    if (cell.x >= grid.width || cell.y >= grid.height) return false;
    const unsigned char cost = grid.costs[Index(grid, cell.x, cell.y)];
    if (cost == NO_INFORMATION) return settings.allow_unknown_transit;
    return cost < INSCRIBED_INFLATED_OBSTACLE;
}

bool IsFrontier(const GridSnapshot& grid, unsigned x, unsigned y) {
    const unsigned char cost = grid.costs[Index(grid, x, y)];
    if (cost == NO_INFORMATION || cost >= INSCRIBED_INFLATED_OBSTACLE) return false;
    for (const auto& [dx, dy] : kCardinal) {
        const int nx = static_cast<int>(x) + dx;
        const int ny = static_cast<int>(y) + dy;
        if (InBounds(grid, nx, ny) &&
            grid.costs[Index(grid, static_cast<unsigned>(nx),
                             static_cast<unsigned>(ny))] == NO_INFORMATION) {
            return true;
        }
    }
    return false;
}

bool HasClearance(const GridSnapshot& grid, GridCell cell, unsigned radius) {
    for (int dy = -static_cast<int>(radius); dy <= static_cast<int>(radius); ++dy) {
        for (int dx = -static_cast<int>(radius); dx <= static_cast<int>(radius); ++dx) {
            const int x = static_cast<int>(cell.x) + dx;
            const int y = static_cast<int>(cell.y) + dy;
            if (!InBounds(grid, x, y)) return false;
            const auto cost = grid.costs[Index(grid, static_cast<unsigned>(x),
                                               static_cast<unsigned>(y))];
            // Unknown is expected next to a frontier; only inflated/occupied
            // cells invalidate a viewpoint.
            if (cost != NO_INFORMATION && cost >= INSCRIBED_INFLATED_OBSTACLE)
                return false;
        }
    }
    return true;
}

struct FrontierCluster {
    std::vector<GridCell> cells;
    GridCell representative;
};

std::vector<FrontierCluster> FindFrontiers(
    const GridSnapshot& grid, const TareExplorerSettings& settings,
    const std::function<bool()>& canceled, bool* was_canceled) {
    const std::size_t count = grid.costs.size();
    std::vector<unsigned char> frontier(count, 0), seen(count, 0);
    for (unsigned y = 0; y < grid.height; ++y) {
        if (canceled && canceled()) { *was_canceled = true; return {}; }
        for (unsigned x = 0; x < grid.width; ++x)
            frontier[Index(grid, x, y)] = IsFrontier(grid, x, y);
    }

    std::vector<FrontierCluster> clusters;
    for (unsigned sy = 0; sy < grid.height; ++sy) {
        for (unsigned sx = 0; sx < grid.width; ++sx) {
            const std::size_t seed = Index(grid, sx, sy);
            if (!frontier[seed] || seen[seed]) continue;
            FrontierCluster cluster;
            std::queue<GridCell> open;
            open.push({sx, sy});
            seen[seed] = 1;
            while (!open.empty()) {
                if (canceled && canceled()) { *was_canceled = true; return {}; }
                const GridCell cell = open.front();
                open.pop();
                cluster.cells.push_back(cell);
                for (const auto& [dx, dy] : kNeighbors) {
                    const int nx = static_cast<int>(cell.x) + dx;
                    const int ny = static_cast<int>(cell.y) + dy;
                    if (!InBounds(grid, nx, ny)) continue;
                    const std::size_t ni = Index(grid, static_cast<unsigned>(nx),
                                                 static_cast<unsigned>(ny));
                    if (frontier[ni] && !seen[ni]) {
                        seen[ni] = 1;
                        open.push({static_cast<unsigned>(nx), static_cast<unsigned>(ny)});
                    }
                }
            }
            if (cluster.cells.size() < settings.min_frontier_cluster_size) continue;
            double cx = 0.0, cy = 0.0;
            for (const auto cell : cluster.cells) { cx += cell.x; cy += cell.y; }
            cx /= cluster.cells.size();
            cy /= cluster.cells.size();
            double best = std::numeric_limits<double>::infinity();
            bool found = false;
            for (const auto cell : cluster.cells) {
                if (!HasClearance(grid, cell, settings.viewpoint_clearance_cells)) continue;
                const double d = std::hypot(cell.x - cx, cell.y - cy);
                if (d < best) { best = d; cluster.representative = cell; found = true; }
            }
            if (found) clusters.push_back(std::move(cluster));
        }
    }
    return clusters;
}

struct SearchNode {
    double f;
    std::size_t index;
    bool operator>(const SearchNode& other) const { return f > other.f; }
};

std::vector<GridCell> AStar(const GridSnapshot& grid, GridCell start, GridCell goal,
                            const TareExplorerSettings& settings,
                            const std::function<bool()>& canceled,
                            bool* was_canceled, bool* search_limit_hit) {
    if (!ValidGrid(grid) || !IsTraversable(grid, start, settings) ||
        !IsTraversable(grid, goal, settings)) return {};
    if (canceled && canceled()) { *was_canceled = true; return {}; }
    const std::size_t n = grid.costs.size();
    const std::size_t start_i = Index(grid, start.x, start.y);
    const std::size_t goal_i = Index(grid, goal.x, goal.y);
    std::vector<double> g(n, std::numeric_limits<double>::infinity());
    std::vector<std::size_t> parent(n, n);
    std::vector<unsigned char> closed(n, 0);
    std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> open;
    g[start_i] = 0.0;
    open.push({std::hypot(static_cast<double>(goal.x) - start.x,
                          static_cast<double>(goal.y) - start.y), start_i});
    std::size_t iterations = 0;
    while (!open.empty() && iterations++ < settings.max_search_iterations) {
        if ((iterations & 0x3ffU) == 0 && canceled && canceled()) {
            *was_canceled = true;
            return {};
        }
        const std::size_t current_i = open.top().index;
        open.pop();
        if (closed[current_i]) continue;
        closed[current_i] = 1;
        if (current_i == goal_i) break;
        const unsigned x = static_cast<unsigned>(current_i % grid.width);
        const unsigned y = static_cast<unsigned>(current_i / grid.width);
        for (const auto& [dx, dy] : kNeighbors) {
            const int nx = static_cast<int>(x) + dx;
            const int ny = static_cast<int>(y) + dy;
            if (!InBounds(grid, nx, ny)) continue;
            GridCell next{static_cast<unsigned>(nx), static_cast<unsigned>(ny)};
            if (!IsTraversable(grid, next, settings)) continue;
            // Do not diagonally squeeze through two blocked cardinal cells.
            if (dx != 0 && dy != 0 &&
                (!IsTraversable(grid, {static_cast<unsigned>(nx), y}, settings) ||
                 !IsTraversable(grid, {x, static_cast<unsigned>(ny)}, settings))) continue;
            const std::size_t ni = Index(grid, next.x, next.y);
            const double step = dx != 0 && dy != 0 ? std::sqrt(2.0) : 1.0;
            const unsigned char cost = grid.costs[ni];
            const double penalty = cost == NO_INFORMATION ? 1.0 : cost / 252.0;
            const double tentative = g[current_i] + step + penalty;
            if (tentative >= g[ni]) continue;
            g[ni] = tentative;
            parent[ni] = current_i;
            const double h = std::hypot(static_cast<double>(goal.x) - next.x,
                                        static_cast<double>(goal.y) - next.y);
            open.push({tentative + h, ni});
        }
    }
    if (!closed[goal_i]) {
        if (!open.empty() && iterations > settings.max_search_iterations)
            *search_limit_hit = true;
        return {};
    }
    std::vector<GridCell> path;
    for (std::size_t at = goal_i;; at = parent[at]) {
        path.push_back({static_cast<unsigned>(at % grid.width),
                        static_cast<unsigned>(at / grid.width)});
        if (at == start_i) break;
        if (parent[at] == n) return {};
    }
    std::reverse(path.begin(), path.end());
    return path;
}

#ifndef TARE_EXPLORER_CORE_ONLY
GridSnapshot Snapshot(const std::shared_ptr<map::costmap_2d::Costmap2DWrapper>& wrapper) {
    GridSnapshot result;
    if (!wrapper) return result;
    auto* grid = wrapper->getCostmap();
    if (!grid) return result;
    std::lock_guard<map::costmap_2d::Costmap2D::mutex_t> lock(*grid->getMutex());
    result.width = grid->getSizeInCellsX();
    result.height = grid->getSizeInCellsY();
    result.resolution = grid->getResolution();
    result.origin_x = grid->getOriginX();
    result.origin_y = grid->getOriginY();
    const std::size_t size = static_cast<std::size_t>(result.width) * result.height;
    const unsigned char* data = grid->getCharMap();
    if (data) result.costs.assign(data, data + size);
    return result;
}

std::optional<GridCell> WorldToCell(const GridSnapshot& grid, double x, double y) {
    if (!ValidGrid(grid) || !std::isfinite(x) || !std::isfinite(y) ||
        x < grid.origin_x || y < grid.origin_y) return std::nullopt;
    const auto mx = static_cast<unsigned>((x - grid.origin_x) / grid.resolution);
    const auto my = static_cast<unsigned>((y - grid.origin_y) / grid.resolution);
    if (mx >= grid.width || my >= grid.height) return std::nullopt;
    return GridCell{mx, my};
}

void FillPath(const GridSnapshot& grid, const std::vector<GridCell>& cells,
              const automsgs::msgs::geometry_msgs::PoseStamped& reference,
              automsgs::msgs::nav_msgs::Path* path) {
    path->Clear();
    *path->mutable_header() = reference.header();
    for (std::size_t i = 0; i < cells.size(); ++i) {
        auto* pose = path->add_poses();
        *pose->mutable_header() = reference.header();
        pose->mutable_pose()->mutable_position()->set_x(
            grid.origin_x + (cells[i].x + 0.5) * grid.resolution);
        pose->mutable_pose()->mutable_position()->set_y(
            grid.origin_y + (cells[i].y + 0.5) * grid.resolution);
        pose->mutable_pose()->mutable_position()->set_z(
            reference.pose().position().z());
        double yaw = 0.0;
        if (i + 1 < cells.size())
            yaw = std::atan2(static_cast<double>(cells[i + 1].y) - cells[i].y,
                             static_cast<double>(cells[i + 1].x) - cells[i].x);
        else if (i > 0)
            yaw = std::atan2(static_cast<double>(cells[i].y) - cells[i - 1].y,
                             static_cast<double>(cells[i].x) - cells[i - 1].x);
        pose->mutable_pose()->mutable_orientation()->set_z(std::sin(yaw * 0.5));
        pose->mutable_pose()->mutable_orientation()->set_w(std::cos(yaw * 0.5));
    }
}
#endif  // TARE_EXPLORER_CORE_ONLY

}  // namespace

TareExplorerCore::TareExplorerCore(TareExplorerSettings settings)
    : settings_(settings) {
    if (settings_.min_frontier_cluster_size == 0 ||
        settings_.max_global_viewpoints == 0 ||
        settings_.max_search_iterations == 0 ||
        !std::isfinite(settings_.information_gain_weight) ||
        settings_.information_gain_weight < 0.0 ||
        !std::isfinite(settings_.travel_cost_weight) ||
        settings_.travel_cost_weight < 0.0) {
        throw std::invalid_argument("invalid TARE explorer settings");
    }
}

TareExplorerResult TareExplorerCore::Plan(
    const GridSnapshot& grid, GridCell robot, const std::optional<GridCell>& home,
    const std::function<bool()>& cancel_checker) const {
    TareExplorerResult result;
    if (!ValidGrid(grid)) { result.status = TareExplorerStatus::kInvalidMap; return result; }
    if (robot.x >= grid.width || robot.y >= grid.height) {
        result.status = TareExplorerStatus::kInvalidStart; return result;
    }
    if (!IsTraversable(grid, robot, settings_)) {
        result.status = TareExplorerStatus::kBlockedStart; return result;
    }
    if (home && (home->x >= grid.width || home->y >= grid.height)) {
        result.status = TareExplorerStatus::kInvalidGoal; return result;
    }
    if (home && !IsTraversable(grid, *home, settings_)) {
        result.status = TareExplorerStatus::kBlockedGoal; return result;
    }
    if (cancel_checker && cancel_checker()) {
        result.status = TareExplorerStatus::kCanceled; result.canceled = true; return result;
    }
    bool canceled = false;
    bool search_limit_hit = false;
    auto clusters = FindFrontiers(grid, settings_, cancel_checker, &canceled);
    if (canceled) {
        result.status = TareExplorerStatus::kCanceled; result.canceled = true; return result;
    }

    GridCell current = robot;
    std::vector<unsigned char> selected(clusters.size(), 0);
    result.path.push_back(robot);
    std::size_t successful = 0;
    for (std::size_t attempted = 0;
         attempted < clusters.size() && successful < settings_.max_global_viewpoints;
         ++attempted) {
        if (cancel_checker && cancel_checker()) {
            result = {}; result.status = TareExplorerStatus::kCanceled;
            result.canceled = true; return result;
        }
        std::size_t best_i = clusters.size();
        double best_score = -std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < clusters.size(); ++i) {
            if (selected[i]) continue;
            const auto candidate = clusters[i].representative;
            const double distance = grid.resolution *
                std::hypot(static_cast<double>(candidate.x) - current.x,
                           static_cast<double>(candidate.y) - current.y);
            const double gain = std::log1p(static_cast<double>(clusters[i].cells.size()));
            const double score = settings_.information_gain_weight * gain -
                                 settings_.travel_cost_weight * distance;
            if (score > best_score || (score == best_score && i < best_i)) {
                best_score = score; best_i = i;
            }
        }
        if (best_i == clusters.size()) break;
        selected[best_i] = 1;
        auto segment = AStar(grid, current, clusters[best_i].representative,
                             settings_, cancel_checker, &canceled, &search_limit_hit);
        if (canceled) {
            result = {}; result.status = TareExplorerStatus::kCanceled;
            result.canceled = true; return result;
        }
        // A one-cell segment makes no progress and would be repeatedly selected
        // by a stateless exploration cycle.
        if (segment.size() < 2) continue;
        result.viewpoints.push_back(clusters[best_i].representative);
        result.path.insert(result.path.end(), std::next(segment.begin()), segment.end());
        current = clusters[best_i].representative;
        ++successful;
    }
    if (!result.viewpoints.empty()) {
        result.status = TareExplorerStatus::kSuccess;
        return result;
    }

    result.path.clear();
    if (search_limit_hit) {
        result.status = TareExplorerStatus::kSearchLimitExceeded;
        return result;
    }
    // No reachable frontier remains. This is the completion condition used by
    // the exploration API; only now may it return to the supplied home pose.
    result.exploration_complete = true;
    result.status = TareExplorerStatus::kExplorationComplete;
    if (home) {
        result.path = AStar(grid, robot, *home, settings_, cancel_checker, &canceled,
                            &search_limit_hit);
        result.canceled = canceled;
        if (canceled) result.status = TareExplorerStatus::kCanceled;
        else if (search_limit_hit) result.status = TareExplorerStatus::kSearchLimitExceeded;
        result.return_home_failed = !canceled && result.path.empty();
    } else {
        result.path.push_back(robot);
    }
    return result;
}

TareExplorerResult TareExplorerCore::PlanRoute(
    const GridSnapshot& grid, GridCell start, GridCell goal,
    const std::function<bool()>& cancel_checker) const {
    TareExplorerResult result;
    if (!ValidGrid(grid)) { result.status = TareExplorerStatus::kInvalidMap; return result; }
    if (start.x >= grid.width || start.y >= grid.height) {
        result.status = TareExplorerStatus::kInvalidStart; return result;
    }
    if (goal.x >= grid.width || goal.y >= grid.height) {
        result.status = TareExplorerStatus::kInvalidGoal; return result;
    }
    if (!IsTraversable(grid, start, settings_)) {
        result.status = TareExplorerStatus::kBlockedStart; return result;
    }
    if (!IsTraversable(grid, goal, settings_)) {
        result.status = TareExplorerStatus::kBlockedGoal; return result;
    }
    bool canceled = false;
    bool search_limit_hit = false;
    result.path = AStar(grid, start, goal, settings_, cancel_checker, &canceled,
                        &search_limit_hit);
    result.canceled = canceled;
    result.status = canceled ? TareExplorerStatus::kCanceled
                  : search_limit_hit ? TareExplorerStatus::kSearchLimitExceeded
                  : result.path.empty() ? TareExplorerStatus::kNoPath
                                        : TareExplorerStatus::kSuccess;
    return result;
}

#ifndef TARE_EXPLORER_CORE_ONLY
TareExplorerSettings ReadTareExplorerSettings(const proto::TareExplorerOptions& options) {
    TareExplorerSettings result;
    if (options.has_min_frontier_cluster_size())
        result.min_frontier_cluster_size = options.min_frontier_cluster_size();
    if (options.has_max_global_viewpoints())
        result.max_global_viewpoints = options.max_global_viewpoints();
    if (options.has_viewpoint_clearance_cells())
        result.viewpoint_clearance_cells = options.viewpoint_clearance_cells();
    if (options.has_information_gain_weight())
        result.information_gain_weight = options.information_gain_weight();
    if (options.has_travel_cost_weight())
        result.travel_cost_weight = options.travel_cost_weight();
    if (options.has_allow_unknown_transit())
        result.allow_unknown_transit = options.allow_unknown_transit();
    if (options.has_max_search_iterations()) {
        if (options.max_search_iterations() >
            static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
            throw std::invalid_argument("TARE max_search_iterations exceeds size_t");
        }
        result.max_search_iterations =
            static_cast<std::size_t>(options.max_search_iterations());
    }
    return result;
}

TareExplorerAdapter::TareExplorerAdapter(
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap,
    TareExplorerSettings settings)
    : costmap_(std::move(costmap)), core_(settings) {}

TareExplorerResult TareExplorerAdapter::PlanExploration(
    const automsgs::msgs::geometry_msgs::PoseStamped& robot,
    const std::optional<automsgs::msgs::geometry_msgs::PoseStamped>& home,
    automsgs::msgs::nav_msgs::Path& path,
    const std::function<bool()>& cancel_checker) const {
    path.Clear();
    const GridSnapshot grid = Snapshot(costmap_);
    const auto robot_cell = WorldToCell(grid, robot.pose().position().x(),
                                        robot.pose().position().y());
    std::optional<GridCell> home_cell;
    if (home) home_cell = WorldToCell(grid, home->pose().position().x(),
                                     home->pose().position().y());
    if (!ValidGrid(grid)) {
        TareExplorerResult result; result.status = TareExplorerStatus::kInvalidMap;
        return result;
    }
    if (!robot_cell) {
        TareExplorerResult result; result.status = TareExplorerStatus::kInvalidStart;
        return result;
    }
    if (home && !home_cell) {
        TareExplorerResult result; result.status = TareExplorerStatus::kInvalidGoal;
        return result;
    }
    auto result = core_.Plan(grid, *robot_cell, home_cell, cancel_checker);
    if (!result.canceled && !result.path.empty()) {
        FillPath(grid, result.path, robot, &path);
        *path.mutable_poses(0)->mutable_pose() = robot.pose();
        if (result.exploration_complete && home) {
            *path.mutable_poses(path.poses_size() - 1)->mutable_pose() = home->pose();
        }
    }
    return result;
}

TareExplorerResult TareExplorerAdapter::PlanRoute(
    const automsgs::msgs::geometry_msgs::PoseStamped& start,
    const automsgs::msgs::geometry_msgs::PoseStamped& goal,
    automsgs::msgs::nav_msgs::Path& path,
    const std::function<bool()>& cancel_checker) const {
    path.Clear();
    const GridSnapshot grid = Snapshot(costmap_);
    const auto start_cell = WorldToCell(grid, start.pose().position().x(),
                                        start.pose().position().y());
    const auto goal_cell = WorldToCell(grid, goal.pose().position().x(),
                                       goal.pose().position().y());
    if (!ValidGrid(grid)) {
        TareExplorerResult result; result.status = TareExplorerStatus::kInvalidMap;
        return result;
    }
    if (!start_cell) {
        TareExplorerResult result; result.status = TareExplorerStatus::kInvalidStart;
        return result;
    }
    if (!goal_cell) {
        TareExplorerResult result; result.status = TareExplorerStatus::kInvalidGoal;
        return result;
    }
    auto result = core_.PlanRoute(grid, *start_cell, *goal_cell, cancel_checker);
    if (!result.canceled && !result.path.empty()) {
        FillPath(grid, result.path, start, &path);
        auto* first = path.mutable_poses(0);
        *first->mutable_pose() = start.pose();
        auto* last = path.mutable_poses(path.poses_size() - 1);
        *last->mutable_pose()->mutable_position() = goal.pose().position();
        *last->mutable_pose()->mutable_orientation() = goal.pose().orientation();
    }
    return result;
}

TareExplorerPlanner::TareExplorerPlanner(
    const proto::PlannerOptions& options, const std::string& name,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap)
    : GlobalPlanner(options, name, costmap),
      explorer_(std::make_shared<TareExplorerAdapter>(
          std::move(costmap), ReadTareExplorerSettings(options.tare_explorer()))) {}

uint32 TareExplorerPlanner::CreatePlan(
    const automsgs::msgs::geometry_msgs::PoseStamped& start,
    const automsgs::msgs::geometry_msgs::PoseStamped& goal,
    automsgs::msgs::nav_msgs::Path& path,
    std::function<bool()> cancel_checker) {
    if (!explorer_) {
        path.Clear();
        return static_cast<uint32>(proto::PlannerResultCode::PLANNER_NOT_INITIALIZED);
    }
    auto result = explorer_->PlanRoute(start, goal, path, cancel_checker);
    if (result.status != TareExplorerStatus::kSuccess) path.Clear();
    switch (result.status) {
        case TareExplorerStatus::kSuccess:
            return static_cast<uint32>(proto::PlannerResultCode::PLANNER_SUCCESS);
        case TareExplorerStatus::kCanceled:
            return static_cast<uint32>(proto::PlannerResultCode::PLANNER_CANCELED);
        case TareExplorerStatus::kInvalidStart:
            return static_cast<uint32>(proto::PlannerResultCode::PLANNER_INVALID_START);
        case TareExplorerStatus::kInvalidGoal:
            return static_cast<uint32>(proto::PlannerResultCode::PLANNER_INVALID_GOAL);
        case TareExplorerStatus::kBlockedStart:
            return static_cast<uint32>(proto::PlannerResultCode::PLANNER_BLOCKED_START);
        case TareExplorerStatus::kBlockedGoal:
            return static_cast<uint32>(proto::PlannerResultCode::PLANNER_BLOCKED_GOAL);
        case TareExplorerStatus::kSearchLimitExceeded:
            return static_cast<uint32>(proto::PlannerResultCode::PLANNER_PAT_EXCEEDED);
        case TareExplorerStatus::kInvalidMap:
            return static_cast<uint32>(proto::PlannerResultCode::PLANNER_INTERNAL_ERROR);
        default:
            return static_cast<uint32>(proto::PlannerResultCode::PLANNER_NO_PATH_FOUND);
    }
}

TareExplorerResult TareExplorerPlanner::CreateExplorationPlan(
    const automsgs::msgs::geometry_msgs::PoseStamped& robot,
    const std::optional<automsgs::msgs::geometry_msgs::PoseStamped>& home,
    automsgs::msgs::nav_msgs::Path& path,
    const std::function<bool()>& cancel_checker) const {
    if (!explorer_) { path.Clear(); return {}; }
    return explorer_->PlanExploration(robot, home, path, cancel_checker);
}
#endif  // TARE_EXPLORER_CORE_ONLY

}  // namespace autonomy::planning::planner::tare_explorer

#ifndef TARE_EXPLORER_CORE_ONLY
using autonomy::planning::common::GlobalPlanner;
using autonomy::planning::planner::tare_explorer::TareExplorerPlanner;
AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(TareExplorerPlanner, GlobalPlanner);
#endif
