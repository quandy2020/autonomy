/* Copyright 2026 The Openbot Authors. */
/** @file surface_extractor_test.cpp
 * @brief Measured occupied/free surface extraction regressions.
 */
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "autonomy/task/exploration/coverage/surface_extractor.hpp"

namespace e = autonomy::task::exploration;
namespace p = autonomy::perception;

namespace {
void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

std::shared_ptr<p::ObservedVolumeView> Volume(bool include_free) {
    auto volume = std::make_shared<p::ObservedVolumeView>();
    volume->map_epoch = 4;
    volume->geometry_revision = 7;
    volume->frame_id = "map";
    volume->dimensions = {3, 3, 3};
    volume->resolution = .2;
    volume->mapping_synchronized = true;
    std::vector<p::ObservedVoxel> voxels{
        {{1, 1, 1}, p::ObservedVoxelState::kOccupied}};
    if (include_free) {
        voxels.push_back({{2, 1, 1}, p::ObservedVoxelState::kFree});
    }
    volume->voxels = std::make_shared<const std::vector<p::ObservedVoxel>>(
        std::move(voxels));
    return volume;
}

void OnlyMeasuredFreeCreatesSurfaceFace() {
    e::MapReadView map;
    map.map_epoch = 4;
    map.geometry_revision = 7;
    map.frame_id = "map";
    map.observed_volume = Volume(false);
    Require(e::SurfaceExtractor{}.Extract(map).empty(),
            "omitted unknown voxel was interpreted as measured free");
    map.observed_volume = Volume(true);
    const auto surfaces = e::SurfaceExtractor{}.Extract(map);
    Require(surfaces.size() == 1 &&
                surfaces.front().id.kind == e::CoverageKind::kSurface &&
                surfaces.front().normal.x == 1 &&
                surfaces.front().normal.y == 0 &&
                surfaces.front().normal.z == 0,
            "measured occupied/free face was not retained as a surface unit");
}

void ConcavePolygonExcludesBoundingBoxSurfaces() {
    ::automsgs::msgs::geometry_msgs::Polygon polygon;
    for (const auto& [x, y] : {std::pair<float, float>{0, 0},
                               {.6F, 0},
                               {.6F, .25F},
                               {.35F, .25F},
                               {.35F, .6F},
                               {0, .6F}}) {
        auto* point = polygon.add_points();
        point->set_x(x);
        point->set_y(y);
    }
    e::MapReadView map;
    map.map_epoch = 4;
    map.geometry_revision = 7;
    map.frame_id = "map";
    map.observed_volume = Volume(true);
    Require(e::SurfaceExtractor{}
                .Extract(map, e::Bounds2d{0, 0, .6, .6}, polygon)
                .empty(),
            "concave cutout surface was accepted by broad-phase bounds");
}

void ConcaveNoGoExcludesSurface() {
    ::automsgs::msgs::geometry_msgs::Polygon no_go;
    for (const auto& [x, y] : {std::pair<float, float>{.2F, .2F},
                               {.6F, .2F},
                               {.6F, .6F},
                               {.5F, .6F},
                               {.5F, .4F},
                               {.2F, .4F}}) {
        auto* point = no_go.add_points();
        point->set_x(x);
        point->set_y(y);
    }
    e::MapReadView map;
    map.map_epoch = 4;
    map.geometry_revision = 7;
    map.frame_id = "map";
    map.observed_volume = Volume(true);
    Require(e::SurfaceExtractor{}.Extract(map, {}, {}, {no_go}).empty(),
            "surface inside a concave no-go was admitted");
}

void MalformedSparseVolumesFailClosed() {
    e::MapReadView map;
    map.map_epoch = 4;
    map.geometry_revision = 7;
    map.frame_id = "map";
    auto duplicate = Volume(true);
    auto voxels = std::vector<p::ObservedVoxel>(*duplicate->voxels);
    voxels.push_back({{2, 1, 1}, p::ObservedVoxelState::kOccupied});
    duplicate->voxels = std::make_shared<const std::vector<p::ObservedVoxel>>(
        std::move(voxels));
    map.observed_volume = duplicate;
    Require(e::SurfaceExtractor{}.Extract(map).empty(),
            "conflicting duplicate voxel fabricated a measured surface");
    auto identical = Volume(true);
    auto identical_voxels = std::vector<p::ObservedVoxel>(*identical->voxels);
    identical_voxels.push_back(identical_voxels.front());
    identical->voxels = std::make_shared<const std::vector<p::ObservedVoxel>>(
        std::move(identical_voxels));
    map.observed_volume = identical;
    Require(e::SurfaceExtractor{}.Extract(map).empty(),
            "identical duplicate voxel escaped canonical validation");
    auto bad_stamp = Volume(true);
    bad_stamp->source_stamp = std::numeric_limits<double>::quiet_NaN();
    map.observed_volume = bad_stamp;
    Require(e::SurfaceExtractor{}.Extract(map).empty(),
            "nonfinite volume stamp escaped fail-closed extraction");
    auto nonfinite = Volume(true);
    nonfinite->origin.x() = std::numeric_limits<double>::quiet_NaN();
    map.observed_volume = nonfinite;
    Require(e::SurfaceExtractor{}.Extract(map).empty(),
            "nonfinite volume origin escaped fail-closed extraction");
    auto outside = Volume(true);
    outside->voxels = std::make_shared<const std::vector<p::ObservedVoxel>>(
        std::vector<p::ObservedVoxel>{
            {{3, 1, 1}, p::ObservedVoxelState::kOccupied},
            {{2, 1, 1}, p::ObservedVoxelState::kFree}});
    map.observed_volume = outside;
    Require(e::SurfaceExtractor{}.Extract(map).empty(),
            "out-of-range sparse voxel fabricated a measured surface");
}
}  // namespace

int main() {
    try {
        OnlyMeasuredFreeCreatesSurfaceFace();
        std::cout << "PASS measured_surface_face\n";
        ConcavePolygonExcludesBoundingBoxSurfaces();
        std::cout << "PASS concave_polygon_surface\n";
        ConcaveNoGoExcludesSurface();
        std::cout << "PASS concave_no_go_surface\n";
        MalformedSparseVolumesFailClosed();
        std::cout << "PASS malformed_volume_fails_closed\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL surface_extractor: " << error.what() << '\n';
        return 1;
    }
}
