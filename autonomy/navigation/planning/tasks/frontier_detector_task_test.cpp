/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "navigation/planning/tasks/frontier_detector_task.hpp"

using Grid = automsgs::msgs::map_msgs::OccupancyGrid;
using Pose = navigation::common::Pose2D;
void Check(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
Grid Room(int w = 30, int h = 30) {
    Grid grid;
    grid.mutable_info()->set_width(w); grid.mutable_info()->set_height(h);
    grid.mutable_info()->set_resolution(0.2F);
    grid.mutable_info()->mutable_origin()->mutable_orientation()->set_w(1);
    for (int y = 0; y < h; ++y) for (int x = 0; x < w; ++x)
        grid.add_data(x == 0 || y == 0 || x == w - 1 || y == h - 1 ? 100 : 0);
    return grid;
}
Pose At(int x, int y) { Pose p; p.set_x((x + .5) * .2); p.set_y((y + .5) * .2); return p; }
int main() {
    try {
        navigation::planning::FrontierDetectorTask detector;
        auto grid = Room();
        const auto robot = At(7, 7);
        Check(detector.Detect(grid, robot, 100).empty(), "finite room edges became frontier");
        // A disconnected unknown boundary behind a solid wall is unreachable.
        for (int y = 1; y < 29; ++y) grid.set_data(y * 30 + 15, 100);
        for (int y = 5; y < 25; ++y) for (int x = 22; x < 29; ++x) grid.set_data(y * 30 + x, -1);
        Check(detector.Detect(grid, robot, 100).empty(), "selected disconnected frontier");
        Check(detector.PathTo(grid, robot, At(20, 15)).empty(), "path crossed solid wall");
        // Open a safe gap: every path cell remains free and routes around wall.
        for (int y = 17; y <= 24; ++y) grid.set_data(y * 30 + 15, 0);
        const auto frontiers = detector.Detect(grid, robot, 100);
        Check(!frontiers.empty(), "reachable frontier missing");
        const auto route = detector.PathTo(grid, robot, frontiers.front().pose);
        Check(!route.empty(), "frontier path missing");
        bool through_gap = false;
        for (const auto& wp : route) {
            const int x = int(wp.pose.x() / .2), y = int(wp.pose.y() / .2);
            Check(grid.data(y * 30 + x) == 0, "route entered occupied/unknown cell");
            if (x == 15) { Check(y >= 19 && y <= 22, "route violated wall clearance"); through_gap = true; }
        }
        Check(through_gap, "path ignored gap");
        Check(detector.Detect(grid, robot, .2).empty(), "range bound ignored");
        // A robot inside the clearance inflation still gets a path: the start
        // snaps to the nearest connected safe cell instead of failing outright.
        grid = Room();
        grid.set_data(7 * 30 + 8, 100);
        Check(!detector.PathTo(grid, At(7, 7), At(20, 20)).empty(),
              "start cell inflation blocked the whole path");
        grid.set_data(7 * 30 + 7, 100);
        Check(detector.PathTo(grid, At(7, 7), At(20, 20)).empty(),
              "occupied start cell produced a path");
        // Ring cluster's centroid lies in occupied interior; choose a real member.
        grid = Room();
        for (int y = 5; y < 25; ++y) for (int x = 5; x < 25; ++x) {
            if (x < 8 || x > 21 || y < 8 || y > 21) grid.set_data(y * 30 + x, -1);
        }
        for (int y = 13; y < 17; ++y) for (int x = 13; x < 17; ++x) grid.set_data(y * 30 + x, 100);
        const auto ring = detector.Detect(grid, At(10, 10), 100, 0);
        Check(!ring.empty(), "ring frontier missing");
        for (const auto& wp : ring) {
            const int x = int(wp.pose.x() / .2), y = int(wp.pose.y() / .2);
            Check(grid.data(y * 30 + x) == 0, "cluster centroid lies inside obstacle");
        }
        // Origin rotation and translation are applied in both directions.
        const auto plain = detector.Detect(grid, At(10, 10), 100, 0);
        auto* origin = grid.mutable_info()->mutable_origin();
        origin->mutable_position()->set_x(10); origin->mutable_position()->set_y(-4);
        origin->mutable_orientation()->set_z(std::sin(.5)); origin->mutable_orientation()->set_w(std::cos(.5));
        Pose transformed;
        transformed.set_x(10 + std::cos(1.) * 2.1 - std::sin(1.) * 2.1);
        transformed.set_y(-4 + std::sin(1.) * 2.1 + std::cos(1.) * 2.1);
        const auto rotated = detector.Detect(grid, transformed, 100, 0);
        Check(rotated.size() == plain.size(), "rotated origin changed reachability");
        Check(!detector.PathTo(grid, transformed, rotated.front().pose, 0).empty(), "rotated path missing");
        grid.mutable_info()->set_resolution(0);
        Check(detector.Detect(grid, robot, 100).empty(), "zero resolution accepted");
        grid.mutable_info()->set_width(0xffffffff);
        Check(detector.Detect(grid, robot, 100).empty(), "oversize grid accepted");
        std::cout << "PASS reachability, wall inflation, cluster member, origin and bounds\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
