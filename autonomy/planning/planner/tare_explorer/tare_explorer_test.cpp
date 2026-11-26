/* Copyright 2026 The Openbot Authors. */
#include <stdexcept>

#include "tare_explorer.hpp"

namespace t = autonomy::planning::planner::tare_explorer;
namespace m {
constexpr unsigned char FREE_SPACE = 0;
constexpr unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
constexpr unsigned char LETHAL_OBSTACLE = 254;
constexpr unsigned char NO_INFORMATION = 255;
}  // namespace m

namespace {
void Require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}

t::GridSnapshot Grid(unsigned width, unsigned height, unsigned char fill) {
    t::GridSnapshot grid;
    grid.width = width;
    grid.height = height;
    grid.resolution = 0.1;
    grid.costs.assign(static_cast<std::size_t>(width) * height, fill);
    return grid;
}

void Set(t::GridSnapshot& grid, unsigned x, unsigned y, unsigned char cost) {
    grid.costs[static_cast<std::size_t>(y) * grid.width + x] = cost;
}

void FrontierSelectionAndLocalRoute() {
    auto grid = Grid(16, 12, m::NO_INFORMATION);
    for (unsigned y = 3; y <= 8; ++y)
        for (unsigned x = 3; x <= 12; ++x) Set(grid, x, y, m::FREE_SPACE);
    t::TareExplorerSettings settings;
    settings.min_frontier_cluster_size = 3;
    settings.viewpoint_clearance_cells = 0;
    settings.max_global_viewpoints = 3;
    t::TareExplorerCore explorer(settings);
    const auto result = explorer.Plan(grid, {7, 5});
    Require(!result.canceled && !result.exploration_complete,
            "a map with reachable frontiers is not complete");
    Require(!result.viewpoints.empty() && result.path.front() == t::GridCell{7, 5},
            "TARE selects a frontier viewpoint and connects it from the robot");
    for (const auto cell : result.path)
        Require(grid.costs[cell.y * grid.width + cell.x] != m::NO_INFORMATION,
                "default local routing must not cross unknown cells");
}

void StrictRouteDoesNotCutCorners() {
    auto grid = Grid(5, 5, m::FREE_SPACE);
    Set(grid, 2, 1, m::LETHAL_OBSTACLE);
    Set(grid, 1, 2, m::LETHAL_OBSTACLE);
    t::TareExplorerCore explorer;
    const auto result = explorer.PlanRoute(grid, {1, 1}, {2, 2});
    Require(!result.path.empty() && result.path.size() > 2,
            "local A* must route around blocked diagonal corners");
}

void CompleteReturnsHome() {
    auto grid = Grid(10, 6, m::FREE_SPACE);
    t::TareExplorerCore explorer;
    const auto result = explorer.Plan(grid, {8, 3}, t::GridCell{1, 3});
    Require(result.exploration_complete && !result.return_home_failed &&
            !result.path.empty() && result.path.back() == t::GridCell{1, 3},
            "a frontier-free map returns home and reports completion");
}

void UnreachableFrontiersComplete() {
    auto grid = Grid(12, 8, m::NO_INFORMATION);
    for (unsigned y = 2; y <= 5; ++y)
        for (unsigned x = 2; x <= 4; ++x) Set(grid, x, y, m::FREE_SPACE);
    // Enclose the robot's known component; the only frontier component is on
    // the far side of an occupied wall and cannot consume a viewpoint quota.
    for (unsigned x = 1; x <= 5; ++x) {
        Set(grid, x, 1, m::LETHAL_OBSTACLE);
        Set(grid, x, 6, m::LETHAL_OBSTACLE);
    }
    for (unsigned y = 1; y <= 6; ++y) {
        Set(grid, 1, y, m::LETHAL_OBSTACLE);
        Set(grid, 5, y, m::LETHAL_OBSTACLE);
    }
    for (unsigned y = 0; y < grid.height; ++y) Set(grid, 6, y, m::LETHAL_OBSTACLE);
    for (unsigned y = 2; y <= 5; ++y)
        for (unsigned x = 8; x <= 9; ++x) Set(grid, x, y, m::FREE_SPACE);
    t::TareExplorerSettings settings;
    settings.min_frontier_cluster_size = 2;
    settings.viewpoint_clearance_cells = 0;
    t::TareExplorerCore explorer(settings);
    const auto result = explorer.Plan(grid, {3, 3}, t::GridCell{2, 3});
    Require(result.exploration_complete && !result.return_home_failed &&
            result.path.back() == t::GridCell{2, 3},
            "no reachable frontier must complete and return home");
}

void CancellationAndValidation() {
    auto grid = Grid(20, 20, m::FREE_SPACE);
    t::TareExplorerCore explorer;
    auto canceled = explorer.PlanRoute(grid, {0, 0}, {19, 19}, [] { return true; });
    Require(canceled.status == t::TareExplorerStatus::kCanceled &&
            canceled.canceled && canceled.path.empty(),
            "cancellation cannot return a successful partial route");
    Require(explorer.PlanRoute(grid, {99, 0}, {1, 1}).status ==
                t::TareExplorerStatus::kInvalidStart,
            "out-of-map starts must be distinguished");
    Set(grid, 19, 19, m::LETHAL_OBSTACLE);
    Require(explorer.PlanRoute(grid, {0, 0}, {19, 19}).status ==
                t::TareExplorerStatus::kBlockedGoal,
            "occupied goals must be distinguished");
    t::TareExplorerSettings bounded;
    bounded.max_search_iterations = 1;
    Require(t::TareExplorerCore(bounded).PlanRoute(grid, {0, 0}, {18, 18}).status ==
                t::TareExplorerStatus::kSearchLimitExceeded,
            "bounded-search exhaustion must be distinguished from no path");
    t::TareExplorerSettings invalid;
    invalid.max_global_viewpoints = 0;
    bool threw = false;
    try { t::TareExplorerCore bad(invalid); } catch (const std::invalid_argument&) { threw = true; }
    Require(threw, "zero work bounds must be rejected");
}
}  // namespace

int main() {
    FrontierSelectionAndLocalRoute();
    StrictRouteDoesNotCutCorners();
    CompleteReturnsHome();
    UnreachableFrontiersComplete();
    CancellationAndValidation();
    return 0;
}
