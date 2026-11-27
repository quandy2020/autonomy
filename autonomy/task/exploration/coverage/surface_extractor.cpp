/* Copyright 2026 The Openbot Authors. */
/** @file surface_extractor.cpp
 * @brief Fail-closed measured 3-D surface requirement extraction.
 */
#include "autonomy/task/exploration/coverage/surface_extractor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <tuple>

#include "autonomy/task/exploration/polygon_boundary.hpp"

namespace autonomy::task::exploration {
namespace {
struct FaceDirection {
    Eigen::Vector3i offset;
    int bin;
};
const std::array<FaceDirection, 6> kFaces{{
    {{1, 0, 0}, 0},
    {{-1, 0, 0}, 1},
    {{0, 1, 0}, 2},
    {{0, -1, 0}, 3},
    {{0, 0, 1}, 4},
    {{0, 0, -1}, 5},
}};

using VoxelKey = std::tuple<int, int, int>;
VoxelKey Key(const Eigen::Vector3i& index) {
    return {index.x(), index.y(), index.z()};
}
}  // namespace

std::vector<CoverageUnit> SurfaceExtractor::Extract(
    const MapReadView& map, const std::optional<Bounds2d>& allowed_region,
    const std::optional<::automsgs::msgs::geometry_msgs::Polygon>&
        allowed_polygon,
    const std::vector<::automsgs::msgs::geometry_msgs::Polygon>& no_go_areas)
    const {
    std::vector<CoverageUnit> result;
    const auto& volume = map.observed_volume;
    if ((allowed_polygon &&
         (!allowed_region ||
          !PolygonBoundaryWithinBounds(*allowed_polygon, *allowed_region))) ||
        std::any_of(no_go_areas.begin(), no_go_areas.end(),
                    [](const auto& polygon) {
                        return !ValidatePolygonBoundary(polygon);
                    }) ||
        !volume || !volume->voxels || !volume->mapping_synchronized ||
        volume->map_epoch != map.map_epoch ||
        volume->geometry_revision != map.geometry_revision ||
        volume->frame_id != map.frame_id ||
        !std::isfinite(volume->source_stamp) || volume->source_stamp < 0 ||
        !volume->origin.array().isFinite().all() ||
        !std::isfinite(volume->resolution) || volume->resolution <= 0 ||
        (volume->dimensions.array() <= 0).any()) {
        return result;
    }
    std::map<VoxelKey, perception::ObservedVoxelState> measured;
    for (const auto& voxel : *volume->voxels) {
        if ((voxel.index.array() < 0).any() ||
            (voxel.index.array() >= volume->dimensions.array()).any() ||
            (voxel.state != perception::ObservedVoxelState::kFree &&
             voxel.state != perception::ObservedVoxelState::kOccupied) ||
            !measured.emplace(Key(voxel.index), voxel.state).second) {
            return {};
        }
    }
    for (const auto& entry : measured) {
        if (entry.second != perception::ObservedVoxelState::kOccupied)
            continue;
        const Eigen::Vector3i voxel{std::get<0>(entry.first),
                                    std::get<1>(entry.first),
                                    std::get<2>(entry.first)};
        for (const auto& face : kFaces) {
            const Eigen::Vector3i adjacent = voxel + face.offset;
            if ((adjacent.array() < 0).any() ||
                (adjacent.array() >= volume->dimensions.array()).any() ||
                measured.find(Key(adjacent)) == measured.end() ||
                measured.at(Key(adjacent)) !=
                    perception::ObservedVoxelState::kFree) {
                continue;
            }
            const Eigen::Vector3d center =
                volume->origin +
                (voxel.cast<double>() + Eigen::Vector3d::Constant(.5) +
                 face.offset.cast<double>() * .5) *
                    volume->resolution;
            if (!center.array().isFinite().all())
                return {};
            const Pose2d planar{center.x(), center.y(), 0};
            if ((allowed_region && !allowed_region->Contains(planar)) ||
                (allowed_polygon &&
                 !PolygonBoundaryContains(*allowed_polygon, planar.x,
                                          planar.y)) ||
                !PolygonBoundaryOutsideAll(no_go_areas, planar.x, planar.y)) {
                continue;
            }
            const Eigen::Vector3d world_index =
                (center / volume->resolution).array().floor();
            if (!world_index.array().isFinite().all() ||
                (world_index.array() < std::numeric_limits<int>::min()).any() ||
                (world_index.array() > std::numeric_limits<int>::max()).any())
                return {};
            CoverageUnit unit;
            unit.id = {static_cast<int>(world_index.x()),
                       static_cast<int>(world_index.y()),
                       static_cast<int>(world_index.z()), face.bin,
                       CoverageKind::kSurface};
            unit.center = {center.x(), center.y(), center.z()};
            unit.normal = {static_cast<double>(face.offset.x()),
                           static_cast<double>(face.offset.y()),
                           static_cast<double>(face.offset.z())};
            unit.area = volume->resolution * volume->resolution;
            unit.tolerance = volume->resolution / std::sqrt(2.0);
            result.push_back(unit);
        }
    }
    return result;
}
}  // namespace autonomy::task::exploration
