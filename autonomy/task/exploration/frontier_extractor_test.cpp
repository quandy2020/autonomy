/* Copyright 2026 The Openbot Authors. */
/** @file frontier_extractor_test.cpp
 * @brief Connected frontier noise-filter regressions.
 */
#include <algorithm>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "autonomy/task/exploration/frontier/frontier_extractor.hpp"
#include "autonomy/task/exploration/polygon_boundary.hpp"

namespace e = autonomy::task::exploration;
namespace n = autonomy::navigation;
namespace p = autonomy::perception;

namespace {
void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

::automsgs::msgs::geometry_msgs::Polygon Polygon(
    std::initializer_list<std::pair<float, float>> vertices) {
    ::automsgs::msgs::geometry_msgs::Polygon polygon;
    for (const auto& [x, y] : vertices) {
        auto* point = polygon.add_points();
        point->set_x(x);
        point->set_y(y);
    }
    return polygon;
}

::automsgs::msgs::geometry_msgs::Polygon ConcaveBoundary() {
    ::automsgs::msgs::geometry_msgs::Polygon polygon;
    for (const auto& [x, y] : {std::pair<float, float>{0, 0},
                               {5, 0},
                               {5, 2},
                               {2, 2},
                               {2, 5},
                               {0, 5}}) {
        auto* point = polygon.add_points();
        point->set_x(x);
        point->set_y(y);
    }
    return polygon;
}

void MinimumClusterFiltersIsolatedBoundaryCells() {
    auto grid = std::make_shared<n::GridMap>(5, 5, 1.0);
    grid->Fill({n::CellState::kUnknown, 0});
    grid->Set(2, 2, {n::CellState::kFree, 0});
    grid->Set(3, 2, {n::CellState::kFree, 0});
    e::MapReadView map;
    map.grid = grid;
    const auto all = e::FrontierExtractor{}.Extract(map, .5, {}, 1);
    const auto clustered = e::FrontierExtractor{}.Extract(map, .5, {}, 2);
    Require(all.size() == 6,
            "all free/unknown boundary cells were not emitted");
    Require(clustered.size() == 4,
            "minimum cluster size did not remove isolated frontier noise");
}

void PolygonValidationAndClosedBoundary() {
    const auto polygon = ConcaveBoundary();
    const auto bounds = e::ValidatePolygonBoundary(polygon);
    Require(bounds && bounds->min_x == 0 && bounds->min_y == 0 &&
                bounds->max_x == 5 && bounds->max_y == 5,
            "concave polygon broad-phase bounds were not computed");
    Require(
        e::PolygonBoundaryWithinBounds(polygon, e::Bounds2d{0, 0, 5, 5}) &&
            !e::PolygonBoundaryWithinBounds(polygon, e::Bounds2d{0, 0, 4, 5}),
        "polygon broad-phase enclosure was not enforced");
    Require(e::PolygonBoundaryContains(polygon, 2, 3),
            "polygon boundary was not included in the task domain");
    Require(!e::PolygonBoundaryContains(polygon, 3, 3),
            "concave polygon cutout was treated as inside");

    ::automsgs::msgs::geometry_msgs::Polygon crossed;
    for (const auto& [x, y] :
         {std::pair<float, float>{0, 0}, {2, 2}, {0, 2}, {2, 0}}) {
        auto* point = crossed.add_points();
        point->set_x(x);
        point->set_y(y);
    }
    Require(!e::ValidatePolygonBoundary(crossed),
            "self-intersecting task polygon was accepted");
}

void ConcaveNoGoAndEdgeExcludeFrontiers() {
    auto grid = std::make_shared<n::GridMap>(5, 5, 1.0);
    grid->Fill({n::CellState::kUnknown, 0});
    grid->Set(1, 2, {n::CellState::kFree, 0});
    e::MapReadView map;
    map.grid = grid;
    const auto no_go = Polygon({{1.5F, 1.5F},
                                {3.5F, 1.5F},
                                {3.5F, 3.5F},
                                {2.5F, 3.5F},
                                {2.5F, 2.5F},
                                {1.5F, 2.5F}});
    Require(e::ValidatePolygonBoundary(no_go).has_value() &&
                !e::PolygonBoundaryOutsideAll({no_go}, 2.5, 2.5) &&
                e::PolygonBoundaryOutsideAll({no_go}, 1.5, 3.5),
            "concave no-go closed-set membership is incorrect");
    const auto frontiers =
        e::FrontierExtractor{}.Extract(map, .5, {}, 1, {}, {no_go});
    Require(frontiers.size() == 2,
            "points on or inside a concave no-go were not excluded");
    for (const auto& frontier : frontiers) {
        Require(
            e::PolygonBoundaryOutsideAll(
                std::vector<::automsgs::msgs::geometry_msgs::Polygon>{no_go},
                frontier.center.x, frontier.center.y),
            "frontier remained on the closed no-go boundary");
    }
}

void ConcavePolygonExcludesBoundingBoxFrontiers() {
    auto grid = std::make_shared<n::GridMap>(5, 5, 1.0);
    grid->Fill({n::CellState::kUnknown, 0});
    grid->Set(1, 2, {n::CellState::kFree, 0});
    e::MapReadView map;
    map.grid = grid;
    const auto polygon = ConcaveBoundary();
    const auto frontiers = e::FrontierExtractor{}.Extract(
        map, .5, e::Bounds2d{0, 0, 5, 5}, 1, polygon);
    Require(frontiers.size() == 3,
            "concave cutout frontier was accepted by broad-phase bounds");
    for (const auto& frontier : frontiers) {
        Require(!(frontier.center.x > 2 && frontier.center.y > 2),
                "frontier escaped the exact concave task polygon");
    }
}

std::shared_ptr<p::ObservedVolumeView> Volume(
    std::vector<p::ObservedVoxel> voxels) {
    auto volume = std::make_shared<p::ObservedVolumeView>();
    volume->map_epoch = 9;
    volume->geometry_revision = 12;
    volume->frame_id = "map";
    volume->origin = {1, -1, 2};
    volume->dimensions = {4, 4, 4};
    volume->resolution = .25;
    volume->mapping_synchronized = true;
    volume->voxels = std::make_shared<const std::vector<p::ObservedVoxel>>(
        std::move(voxels));
    return volume;
}

e::MapReadView VolumeMap(std::vector<p::ObservedVoxel> voxels) {
    e::MapReadView map;
    map.map_epoch = 9;
    map.geometry_revision = 12;
    map.frame_id = "map";
    map.observed_volume = Volume(std::move(voxels));
    return map;
}

void VolumeFrontiersUseSixConnectedMetricClusters() {
    auto map = VolumeMap({{{1, 1, 0}, p::ObservedVoxelState::kFree},
                          {{1, 1, 1}, p::ObservedVoxelState::kFree},
                          {{1, 1, 2}, p::ObservedVoxelState::kFree},
                          {{2, 1, 1}, p::ObservedVoxelState::kOccupied}});
    const auto frontiers = e::FrontierExtractor{}.Extract(map, .5, {}, 3);
    Require(frontiers.size() == 9,
            "3-D frontier clustering did not use vertical 6-connectivity");
    bool found_low = false;
    bool found_high = false;
    for (const auto& frontier : frontiers) {
        Require(frontier.id.kind == e::CoverageKind::kFrontier,
                "volume frontier kind was not preserved");
        Require(!(frontier.center.x == 1.625 && frontier.center.y == -.625 &&
                  frontier.center.z == 2.375),
                "explicitly occupied voxel was treated as unknown frontier");
        if (frontier.center.x == 1.125 && frontier.center.y == -.625) {
            found_low = found_low || frontier.center.z == 2.125;
            found_high = found_high || frontier.center.z == 2.625;
        }
    }
    Require(found_low && found_high,
            "vertical metric frontier centers were not emitted");
    const auto match = std::find_if(
        frontiers.begin(), frontiers.end(), [](const auto& frontier) {
            return frontier.center.x == 1.125 && frontier.center.y == -.625 &&
                   frontier.center.z == 2.125;
        });
    Require(match != frontiers.end() && match->id.x == 4 && match->id.y == -3 &&
                match->id.z == 8,
            "3-D frontier ID did not use the stable world lattice");
}

void VolumeFrontiersFailClosedOnMalformedEvidence() {
    auto fallback_grid = std::make_shared<n::GridMap>(3, 3, 1.0);
    fallback_grid->Fill({n::CellState::kUnknown, 0});
    fallback_grid->Set(1, 1, {n::CellState::kFree, 0});

    auto boundary = VolumeMap({{{0, 0, 0}, p::ObservedVoxelState::kFree},
                               {{1, 0, 0}, p::ObservedVoxelState::kOccupied},
                               {{0, 1, 0}, p::ObservedVoxelState::kOccupied},
                               {{0, 0, 1}, p::ObservedVoxelState::kOccupied}});
    Require(e::FrontierExtractor{}.Extract(boundary).empty(),
            "outside-volume neighbors were treated as unknown frontiers");

    auto duplicate = VolumeMap({{{1, 1, 1}, p::ObservedVoxelState::kFree},
                                {{1, 1, 1}, p::ObservedVoxelState::kFree}});
    duplicate.grid = fallback_grid;
    Require(e::FrontierExtractor{}.Extract(duplicate).empty(),
            "duplicate measured voxel fabricated 3-D frontiers");

    auto outside = VolumeMap({{{4, 1, 1}, p::ObservedVoxelState::kFree}});
    outside.grid = fallback_grid;
    Require(e::FrontierExtractor{}.Extract(outside).empty(),
            "out-of-window measured voxel fabricated 3-D frontiers");

    auto incompatible_volume =
        Volume({{{1, 1, 1}, p::ObservedVoxelState::kFree}});
    incompatible_volume->geometry_revision = 13;
    e::MapReadView incompatible;
    incompatible.map_epoch = 9;
    incompatible.geometry_revision = 12;
    incompatible.frame_id = "map";
    incompatible.grid = fallback_grid;
    incompatible.observed_volume = std::move(incompatible_volume);
    Require(e::FrontierExtractor{}.Extract(incompatible).empty(),
            "incompatible measured volume fell back to planar frontiers");
}
}  // namespace

int main() {
    try {
        MinimumClusterFiltersIsolatedBoundaryCells();
        std::cout << "PASS minimum_frontier_cluster\n";
        PolygonValidationAndClosedBoundary();
        std::cout << "PASS polygon_validation_closed_boundary\n";
        ConcavePolygonExcludesBoundingBoxFrontiers();
        std::cout << "PASS concave_polygon_frontier\n";
        ConcaveNoGoAndEdgeExcludeFrontiers();
        std::cout << "PASS concave_no_go_frontier\n";
        VolumeFrontiersUseSixConnectedMetricClusters();
        std::cout << "PASS metric_3d_frontier_connectivity\n";
        VolumeFrontiersFailClosedOnMalformedEvidence();
        std::cout << "PASS malformed_3d_frontier_volume\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL frontier_extractor: " << error.what() << '\n';
        return 1;
    }
}
