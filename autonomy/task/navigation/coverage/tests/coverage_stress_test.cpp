/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file coverage_stress_test.cpp
 * @brief Map-level regression of the coverage planner (rooms, doors, holes).
 *
 * Every case builds an occupancy grid in memory, plans it and checks three
 * invariants: the request is accepted, the core coverage reaches the expected
 * share of the free space, and no published pose stands on an occupied cell.
 */

#include "autonomy/task/navigation/coverage/coverage_server.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using autonomy::task::coverage::CoverageServer;
namespace proto = autonomy::task::coverage::proto;

// Optional case name from argv: lets a developer rerun one map.
std::string g_case_filter;

constexpr int kCells = 82;
constexpr double kResolution = 0.2;
constexpr double kOrigin = -8.2;

/** @brief Occupancy grid builder used by the cases below. */
struct Grid {
  std::vector<std::int32_t> data;

  Grid() : data(static_cast<std::size_t>(kCells * kCells), 0) {
    // Outer wall, two cells thick.
    for (int i = 0; i < kCells; ++i) {
      for (int t = 0; t < 2; ++t) {
        Set(i, t, 100);
        Set(i, kCells - 1 - t, 100);
        Set(t, i, 100);
        Set(kCells - 1 - t, i, 100);
      }
    }
  }

  void Set(int column, int row, int value) {
    if (column < 0 || row < 0 || column >= kCells || row >= kCells) {
      return;
    }
    data[static_cast<std::size_t>(row) * kCells + column] = value;
  }

  void WallH(int row, int from, int to) {
    for (int c = from; c < to; ++c) {
      Set(c, row, 100);
      Set(c, row + 1, 100);
    }
  }

  void WallV(int column, int from, int to) {
    for (int r = from; r < to; ++r) {
      Set(column, r, 100);
      Set(column + 1, r, 100);
    }
  }

  void DoorH(int row, int column) {
    for (int c = column - 5; c <= column + 4; ++c) {
      Set(c, row, 0);
      Set(c, row + 1, 0);
    }
  }

  void DoorV(int column, int row) {
    for (int r = row - 5; r <= row + 4; ++r) {
      Set(column, r, 0);
      Set(column + 1, r, 0);
    }
  }
};

/** @brief Builds a map request whose work area is the whole grid. */
proto::CoverageRequest MapRequest(const Grid& grid, double start_x,
                                  double start_y, unsigned passes = 0) {
  proto::CoverageRequest request;
  request.set_frame_id("map");
  request.set_coordinates_in_cartesian_frame(true);
  request.set_generate_regions(true);
  request.set_robot_radius(0.4);
  request.set_swath_spacing(0.15);
  request.mutable_headland_mode()->set_mode("CONSTANT");
  request.mutable_headland_mode()->set_width(0.3);
  if (passes > 0) {
    request.set_headland_passes(passes);
  }
  request.mutable_robot_pose()->set_axis1(start_x);
  request.mutable_robot_pose()->set_axis2(start_y);
  auto* ring = request.add_polygons();
  for (auto [x, y] : {std::pair{kOrigin, kOrigin},
                      std::pair{-kOrigin, kOrigin},
                      std::pair{-kOrigin, -kOrigin},
                      std::pair{kOrigin, -kOrigin},
                      std::pair{kOrigin, kOrigin}}) {
    auto* p = ring->add_coordinates();
    p->set_axis1(x);
    p->set_axis2(y);
  }
  auto* map = request.mutable_map();
  map->set_frame_id("map");
  map->set_resolution(kResolution);
  map->set_width(kCells);
  map->set_height(kCells);
  map->set_origin_x(kOrigin);
  map->set_origin_y(kOrigin);
  for (const auto value : grid.data) {
    map->add_data(value);
  }
  return request;
}

/** @brief Fails the case when a published pose stands on an occupied cell. */
int PosesOnObstacles(const proto::OccupancyMap& map,
                     const proto::ComputeCoveragePathAction_Result& result) {
  int bad = 0;
  for (const auto& pose : result.nav_path().poses()) {
    const double x = pose.pose().position().x();
    const double y = pose.pose().position().y();
    const int column = static_cast<int>((x - map.origin_x()) / map.resolution());
    const int row = static_cast<int>((y - map.origin_y()) / map.resolution());
    if (column < 0 || row < 0 || column >= map.width() || row >= map.height()) {
      ++bad;
      continue;
    }
    if (map.data(row * map.width() + column) >= 50) {
      ++bad;
    }
  }
  return bad;
}

/**
 * @brief Dump a case so the local tooling can render it for review.
 * @param[in] name Case name used as the file prefix.
 * @param[in] request Map request that was planned.
 * @param[in] result Planner result carrying the published path.
 */
void DumpCase(const std::string& name, const proto::OccupancyMap& request,
              const proto::ComputeCoveragePathAction_Result& result) {
  const char* directory = std::getenv("COVERAGE_STRESS_DUMP");
  if (directory == nullptr || *directory == '\0') {
    return;
  }
  const std::string prefix = std::string(directory) + "/" + name;
  std::ofstream map_out(prefix + "_map.txt");
  map_out << request.resolution() << ' ' << request.width() << ' '
          << request.height() << ' ' << request.origin_x() << ' '
          << request.origin_y() << '\n';
  for (int row = 0; row < request.height(); ++row) {
    for (int column = 0; column < request.width(); ++column) {
      map_out << static_cast<int>(request.data(row * request.width() + column))
              << (column + 1 == request.width() ? '\n' : ' ');
    }
  }
  std::ofstream path_out(prefix + "_path.txt");
  for (const auto& pose : result.nav_path().poses()) {
    path_out << pose.pose().position().x() << ' ' << pose.pose().position().y()
             << '\n';
  }
}

/**
 * @brief Build the production robot parameters for the in-memory cases.
 *
 * The defaults of Configuration describe a field robot (1.5 m wide), which
 * insets the wall pass by 0.75 m and hides real coverage defects behind the
 * margin. The values below mirror coverage/conf/autosim_coverage.json.
 * @return Configuration matching the Autosim cleaning robot.
 */
autonomy::task::coverage::Configuration ServerConfiguration() {
  autonomy::task::coverage::Configuration config;
  config.robot_width = 0.35;
  config.robot_radius = 0.4;
  config.operation_width = 0.8;
  config.min_turning_radius = 0.4;
  config.headland_width = 0.3;
  config.headland_passes = 1;
  return config;
}

/** @brief Runs one case and checks acceptance, coverage and clearance. */
void RunCase(const std::string& name, const Grid& grid, double minimum,
             double start_x, double start_y, unsigned passes = 0) {
  CoverageServer server(ServerConfiguration());
  const auto request = MapRequest(grid, start_x, start_y, passes);
  const auto result = server.Compute(request);
  if (result.error_code() != proto::NONE) {
    throw std::runtime_error(name + ": " + result.error_msg());
  }
  const double coverage = result.core_coverage();
  const int bad = PosesOnObstacles(request.map(), result);
  // Regions that stay empty were sealed off by the robot clearance; counting
  // them keeps the log explicit when a case reports less than full coverage.
  int sealed = 0;
  for (const auto& region : result.regions()) {
    if (region.coverage() <= 0.0) {
      ++sealed;
    }
  }
  DumpCase(name, request.map(), result);
  if (std::getenv("COVERAGE_STRESS_TRACE") != nullptr) {
    for (const auto& warning : result.warnings()) {
      std::cerr << "  " << name << ": " << warning << std::endl;
    }
  }
  std::cout << name << " coverage=" << coverage << " poses="
            << result.nav_path().poses_size() << " rooms="
            << result.regions_size() << " sealed=" << sealed
            << " on_obstacle=" << bad << std::endl;
  if (coverage < minimum) {
    throw std::runtime_error(name + ": coverage below expectation");
  }
  if (bad != 0) {
    throw std::runtime_error(name + ": path enters an occupied cell");
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc > 1) {
    g_case_filter = argv[1];
  }
  int passed = 0;
  int total = 0;
  const auto run = [&](const std::string& name, const Grid& grid,
                       double minimum, double x = 0.0, double y = 0.0,
                       unsigned passes = 0) {
    if (!g_case_filter.empty() && g_case_filter != name) {
      return;
    }
    ++total;
    const auto started = std::chrono::steady_clock::now();
    try {
      RunCase(name, grid, minimum, x, y, passes);
      ++passed;
    } catch (const std::exception& error) {
      std::cerr << "FAIL " << error.what() << std::endl;
    }
    const auto seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    std::cout << name << " elapsed=" << seconds << "s" << std::endl;
  };

  {
    Grid open;
    run("open", open, 0.99);
    // The panel exposes the wall ring count; two rings must stay accepted and
    // collision free even though the band is twice as wide.
    run("open_two_rings", open, 0.99, 0.0, 0.0, 2);
  }
  {
    Grid rooms;
    for (int k = 1; k <= 3; ++k) {
      const int at = 4 + k * 18;
      rooms.WallH(at, 2, kCells - 2);
      rooms.WallV(at, 2, kCells - 2);
    }
    for (int k = 1; k <= 3; ++k) {
      const int at = 4 + k * 18;
      // One doorway per shared wall segment keeps every room convex enough
      // for a plain boustrophedon sweep.
      for (int s = 0; s <= 3; ++s) {
        rooms.DoorH(at, 13 + s * 18);
        rooms.DoorV(at, 13 + s * 18);
      }
    }
    // 3.6 m rooms behind 2 m doorways: sixteen small rooms are the hardest
    // layout for the wall pass, so the expectation keeps 1.5 points of slack.
    run("rooms_4x4_tight", rooms, 0.98);
  }
  {
    // Three wide rooms: the geometry a household map actually produces.
    Grid rooms;
    rooms.WallH(26, 2, kCells - 2);
    rooms.WallH(54, 2, kCells - 2);
    rooms.DoorH(26, 40);
    rooms.DoorH(54, 40);
    run("rooms_3x1", rooms, 0.99);
  }
  {
    Grid shape;
    shape.WallH(40, 40, kCells - 2);
    shape.WallV(40, 2, 42);
    shape.DoorH(40, 56);
    shape.DoorV(40, 20);
    run("l_shape", shape, 0.99);
  }
  {
    Grid corridors;
    corridors.WallH(26, 2, 60);
    corridors.DoorH(26, 34);
    corridors.WallH(54, 22, kCells - 2);
    corridors.DoorH(54, 46);
    corridors.WallV(60, 2, 28);
    corridors.DoorV(60, 14);
    corridors.WallV(22, 52, kCells - 2);
    corridors.DoorV(22, 66);
    run("corridors", corridors, 0.99);
  }
  {
    Grid pillars;
    for (int r = 14; r < 70; r += 16) {
      for (int c = 14; c < 70; c += 16) {
        for (int dr = 0; dr < 3; ++dr) {
          for (int dc = 0; dc < 3; ++dc) {
            pillars.Set(c + dc, r + dr, 100);
          }
        }
      }
    }
    run("pillars", pillars, 0.985);
  }
  {
    Grid clutter;
    std::uint32_t seed = 7;
    for (int i = 0; i < 40; ++i) {
      seed = seed * 1664525u + 1013904223u;
      const int r = 6 + static_cast<int>(seed % 60);
      seed = seed * 1664525u + 1013904223u;
      const int c = 6 + static_cast<int>(seed % 60);
      for (int dr = 0; dr < 4; ++dr) {
        for (int dc = 0; dc < 4; ++dc) {
          clutter.Set(c + dc, r + dr, 100);
        }
      }
    }
    run("clutter", clutter, 0.985);
  }
  {
    Grid narrow;
    narrow.WallV(40, 2, kCells - 2);
    for (int r = 36; r < 44; ++r) {
      narrow.Set(40, r, 0);
      narrow.Set(41, r, 0);
    }
    run("narrow_door", narrow, 0.99);
  }
  {
    Grid maze;
    for (int k = 0; k < 5; ++k) {
      const int row = 12 + k * 14;
      maze.WallH(row, 2 + (k % 2) * 18, kCells - 2 - ((k + 1) % 2) * 18);
      maze.DoorH(row, 30 + (k % 2) * 12);
    }
    run("maze", maze, 0.985);
  }

  std::cout << passed << '/' << total << " stress cases passed\n";
  return passed == total ? 0 : 1;
}
