/* Copyright 2026 The Navigation Authors (navigation) */
#include "navigation/planning/tasks/frontier_detector_task.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>

namespace navigation::planning {
namespace {
using Grid = automsgs::msgs::map_msgs::OccupancyGrid;
constexpr int kOffsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

// Bound work and allocation before converting protobuf dimensions to int.
class Search {
public:
    Search(const Grid& grid, const common::Pose2D& robot, double clearance)
        : grid_(grid) {
        const auto& info = grid.info();
        const std::uint64_t cells = std::uint64_t(info.width()) * info.height();
        resolution_ = info.resolution();
        const auto& origin = info.origin().position();
        const auto& q = info.origin().orientation();
        yaw_ = std::atan2(2 * (q.w() * q.z() + q.x() * q.y()),
                         1 - 2 * (q.y() * q.y() + q.z() * q.z()));
        if (cells == 0 || cells > 1000000 || cells != std::uint64_t(grid.data_size()) ||
            !std::isfinite(resolution_) || resolution_ <= 0 ||
            !std::isfinite(clearance) || clearance < 0 ||
            clearance / resolution_ > 128 || !common::IsFinite(robot) ||
            !std::isfinite(origin.x()) || !std::isfinite(origin.y()) ||
            !std::isfinite(q.x()) || !std::isfinite(q.y()) ||
            !std::isfinite(q.z()) || !std::isfinite(q.w())) return;
        width_ = int(info.width());
        height_ = int(info.height());
        // Chebyshev distance to occupied cells and map edges. This square
        // inflation includes whole occupied cells and is conservative at corners.
        std::vector<int> distance(cells, width_ + height_);
        for (int y = 0; y < height_; ++y) for (int x = 0; x < width_; ++x) {
            const int i = y * width_ + x;
            const int value = grid.data(i);
            distance[i] = value != 0 && value != -1 ? 0 :
                std::min({x + 1, y + 1, width_ - x, height_ - y});
            for (int dy = -1; dy <= 0; ++dy) for (int dx = -1; dx <= 1; ++dx) {
                if (dy == 0 && dx >= 0) continue;
                if (Inside(x + dx, y + dy))
                    distance[i] = std::min(distance[i], distance[(y + dy) * width_ + x + dx] + 1);
            }
        }
        for (int y = height_ - 1; y >= 0; --y) for (int x = width_ - 1; x >= 0; --x) {
            const int i = y * width_ + x;
            for (int dy = 0; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
                if (dy == 0 && dx <= 0) continue;
                if (Inside(x + dx, y + dy))
                    distance[i] = std::min(distance[i], distance[(y + dy) * width_ + x + dx] + 1);
            }
        }
        parent.assign(cells, -1);
        safe_.resize(cells);
        const int inflation = int(std::ceil(clearance / resolution_));
        for (std::size_t i = 0; i < cells; ++i)
            safe_[i] = grid.data(int(i)) == 0 && distance[i] > inflation;
        start_ = Cell(robot);
        if (start_ < 0) return;
        if (!safe_[start_]) {
            // The robot is often closer than the clearance to a wall or a table
            // leg, which puts its own cell inside the inflation. Snap the start
            // to the nearest safe cell reachable through free cells so a short
            // detour stays plannable; an occupied cell is still a hard failure.
            if (grid.data(start_) != 0) return;
            constexpr int kSnapCells = 3;
            struct Node { int cell; int depth; };
            std::vector<Node> pending{{start_, 0}};
            std::vector<int> visited{start_};
            int snapped = -1;
            int snapped_depth = kSnapCells + 1;
            for (std::size_t head = 0; head < pending.size(); ++head) {
                const Node node = pending[head];
                if (safe_[node.cell] && node.depth < snapped_depth) {
                    snapped = node.cell;
                    snapped_depth = node.depth;
                }
                if (node.depth >= kSnapCells) continue;
                const int x = node.cell % width_, y = node.cell / width_;
                for (const auto& d : kOffsets) {
                    const int nx = x + d[0], ny = y + d[1];
                    if (!Inside(nx, ny)) continue;
                    const int n = ny * width_ + nx;
                    if (grid.data(n) != 0) continue;
                    if (std::find(visited.begin(), visited.end(), n) != visited.end()) continue;
                    visited.push_back(n);
                    pending.push_back(Node{n, node.depth + 1});
                }
            }
            if (snapped < 0) return;
            start_ = snapped;
        }
        std::queue<int> queue;
        queue.push(start_);
        parent[start_] = start_;
        while (!queue.empty()) {
            const int i = queue.front(); queue.pop();
            for (const auto& d : kOffsets) {
                const int x = i % width_ + d[0], y = i / width_ + d[1];
                if (!Inside(x, y)) continue;
                const int n = y * width_ + x;
                if (safe_[n] && parent[n] < 0) { parent[n] = i; queue.push(n); }
            }
        }
    }
    bool Inside(int x, int y) const { return x >= 0 && y >= 0 && x < width_ && y < height_; }
    int Cell(const common::Pose2D& pose) const {
        if (width_ == 0 || !common::IsFinite(pose)) return -1;
        const auto& p = grid_.info().origin().position();
        const double dx = pose.x() - p.x(), dy = pose.y() - p.y();
        const double x = (std::cos(yaw_) * dx + std::sin(yaw_) * dy) / resolution_;
        const double y = (-std::sin(yaw_) * dx + std::cos(yaw_) * dy) / resolution_;
        if (x < 0 || y < 0 || x >= width_ || y >= height_) return -1;
        return int(std::floor(y)) * width_ + int(std::floor(x));
    }
    common::Pose2D Pose(int i) const {
        common::Pose2D pose;
        const double x = (i % width_ + 0.5) * resolution_;
        const double y = (i / width_ + 0.5) * resolution_;
        pose.set_x(grid_.info().origin().position().x() + std::cos(yaw_) * x - std::sin(yaw_) * y);
        pose.set_y(grid_.info().origin().position().y() + std::sin(yaw_) * x + std::cos(yaw_) * y);
        return pose;
    }
    bool Frontier(int i) const {
        if (parent.empty() || parent[i] < 0) return false;
        for (const auto& d : kOffsets) {
            const int x = i % width_ + d[0], y = i / width_ + d[1];
            // The outside of a finite map is not unexplored space.
            if (Inside(x, y) && grid_.data(y * width_ + x) == -1) return true;
        }
        return false;
    }
    std::vector<Waypoint> Route(int end) const {
        std::vector<Waypoint> route;
        if (end < 0 || parent.empty() || parent[end] < 0) return route;
        for (int i = end; i != start_; i = parent[i]) route.push_back(Waypoint{Pose(i), 0.15});
        std::reverse(route.begin(), route.end());
        if (route.empty()) route.push_back(Waypoint{Pose(end), 0.15});
        return route;
    }
    int Width() const { return width_; }
    std::vector<int> parent;
private:
    const Grid& grid_;
    int width_{0}, height_{0}, start_{-1};
    double resolution_{0}, yaw_{0};
    std::vector<std::uint8_t> safe_;
};
}  // namespace

std::vector<Waypoint> FrontierDetectorTask::Detect(
    const Grid& grid, const common::Pose2D& robot, double max_range,
    double clearance_m) const {
    Search search(grid, robot, clearance_m);
    std::vector<Waypoint> result;
    if (!std::isfinite(max_range)) return result;
    std::vector<std::uint8_t> visited(search.parent.size(), 0);
    for (int i = 0; i < int(visited.size()); ++i) {
        if (visited[i] || !search.Frontier(i)) continue;
        std::vector<int> cluster;
        std::queue<int> queue;
        queue.push(i); visited[i] = 1;
        double sx = 0, sy = 0;
        while (!queue.empty()) {
            const int cell = queue.front(); queue.pop();
            cluster.push_back(cell);
            const auto pose = search.Pose(cell); sx += pose.x(); sy += pose.y();
            for (const auto& d : kOffsets) {
                const int x = cell % search.Width() + d[0], y = cell / search.Width() + d[1];
                if (!search.Inside(x, y)) continue;
                const int n = y * search.Width() + x;
                if (!visited[n] && search.Frontier(n)) { visited[n] = 1; queue.push(n); }
            }
        }
        common::Pose2D center; center.set_x(sx / cluster.size()); center.set_y(sy / cluster.size());
        const int representative = *std::min_element(cluster.begin(), cluster.end(),
            [&](int a, int b) { return common::Distance(search.Pose(a), center) < common::Distance(search.Pose(b), center); });
        const auto pose = search.Pose(representative);
        if (max_range <= 0 || common::Distance(robot, pose) <= max_range)
            result.push_back(Waypoint{pose, 0.25});
    }
    std::sort(result.begin(), result.end(), [&](const auto& a, const auto& b) {
        return common::Distance(a.pose, robot) < common::Distance(b.pose, robot);
    });
    return result;
}

std::vector<Waypoint> FrontierDetectorTask::PathTo(
    const Grid& grid, const common::Pose2D& robot, const common::Pose2D& target,
    double clearance_m) const {
    Search search(grid, robot, clearance_m);
    return search.Route(search.Cell(target));
}
}  // namespace navigation::planning
