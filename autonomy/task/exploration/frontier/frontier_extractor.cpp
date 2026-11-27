/* Copyright 2026 The Openbot Authors. */
/** @file frontier_extractor.cpp
 * @brief Connected measured-free/unknown boundary extraction.
 */
#include "autonomy/task/exploration/frontier/frontier_extractor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include "autonomy/task/exploration/polygon_boundary.hpp"

namespace autonomy::task::exploration {
namespace {
constexpr std::array<std::pair<int, int>, 4> kPlanarNeighbors{{
    {1, 0},
    {-1, 0},
    {0, 1},
    {0, -1},
}};
const std::array<Eigen::Vector3i, 6> kVolumeNeighbors{{
    {1, 0, 0},
    {-1, 0, 0},
    {0, 1, 0},
    {0, -1, 0},
    {0, 0, 1},
    {0, 0, -1},
}};

using VoxelKey = std::tuple<int, int, int>;

VoxelKey Key(const Eigen::Vector3i& index) {
    return {index.x(), index.y(), index.z()};
}

Eigen::Vector3i Index(const VoxelKey& key) {
    return {std::get<0>(key), std::get<1>(key), std::get<2>(key)};
}

bool InsideDomain(
    double x, double y, const std::optional<Bounds2d>& allowed_region,
    const std::optional<::automsgs::msgs::geometry_msgs::Polygon>&
        allowed_polygon,
    const std::vector<::automsgs::msgs::geometry_msgs::Polygon>& no_go_areas) {
    const Pose2d planar{x, y, 0};
    return (!allowed_region || allowed_region->Contains(planar)) &&
           (!allowed_polygon ||
            PolygonBoundaryContains(*allowed_polygon, planar.x, planar.y)) &&
           PolygonBoundaryOutsideAll(no_go_areas, planar.x, planar.y);
}
}  // namespace

std::vector<CoverageUnit> FrontierExtractor::Extract(
    const MapReadView& view, double height,
    const std::optional<Bounds2d>& allowed_region,
    std::size_t minimum_cluster_size,
    const std::optional<::automsgs::msgs::geometry_msgs::Polygon>&
        allowed_polygon,
    const std::vector<::automsgs::msgs::geometry_msgs::Polygon>& no_go_areas)
    const {
    std::vector<CoverageUnit> result;
    if (!std::isfinite(height) || !minimum_cluster_size ||
        (allowed_polygon &&
         (!allowed_region ||
          !PolygonBoundaryWithinBounds(*allowed_polygon, *allowed_region))) ||
        std::any_of(no_go_areas.begin(), no_go_areas.end(),
                    [](const auto& polygon) {
                        return !ValidatePolygonBoundary(polygon);
                    })) {
        return result;
    }

    const auto& volume = view.observed_volume;
    if (volume) {
        if (!volume->voxels || !volume->mapping_synchronized ||
            volume->map_epoch != view.map_epoch ||
            volume->geometry_revision != view.geometry_revision ||
            volume->frame_id != view.frame_id ||
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

        std::set<VoxelKey> cells;
        for (const auto& entry : measured) {
            if (entry.second != perception::ObservedVoxelState::kFree)
                continue;
            const Eigen::Vector3i free = Index(entry.first);
            for (const auto& offset : kVolumeNeighbors) {
                const Eigen::Vector3i unknown = free + offset;
                if ((unknown.array() < 0).any() ||
                    (unknown.array() >= volume->dimensions.array()).any() ||
                    measured.find(Key(unknown)) != measured.end()) {
                    continue;
                }
                const Eigen::Vector3d center =
                    volume->origin +
                    (unknown.cast<double>() + Eigen::Vector3d::Constant(.5)) *
                        volume->resolution;
                if (!center.array().isFinite().all())
                    return {};
                if (InsideDomain(center.x(), center.y(), allowed_region,
                                 allowed_polygon, no_go_areas)) {
                    cells.insert(Key(unknown));
                }
            }
        }

        while (!cells.empty()) {
            std::vector<VoxelKey> cluster{*cells.begin()};
            cells.erase(cells.begin());
            for (std::size_t index = 0; index < cluster.size(); ++index) {
                const Eigen::Vector3i cell = Index(cluster[index]);
                for (const auto& offset : kVolumeNeighbors) {
                    const auto found = cells.find(Key(cell + offset));
                    if (found != cells.end()) {
                        cluster.push_back(*found);
                        cells.erase(found);
                    }
                }
            }
            if (cluster.size() < minimum_cluster_size)
                continue;
            for (const auto& key : cluster) {
                const Eigen::Vector3i index = Index(key);
                const Eigen::Vector3d center =
                    volume->origin +
                    (index.cast<double>() + Eigen::Vector3d::Constant(.5)) *
                        volume->resolution;
                const Eigen::Vector3d world_index =
                    (center / volume->resolution).array().floor();
                if (!world_index.array().isFinite().all() ||
                    (world_index.array() < std::numeric_limits<int>::min())
                        .any() ||
                    (world_index.array() > std::numeric_limits<int>::max())
                        .any()) {
                    return {};
                }
                CoverageUnit unit;
                unit.id = {static_cast<int>(world_index.x()),
                           static_cast<int>(world_index.y()),
                           static_cast<int>(world_index.z()), 0,
                           CoverageKind::kFrontier};
                unit.center = {center.x(), center.y(), center.z()};
                unit.area = volume->resolution * volume->resolution;
                unit.tolerance = std::sqrt(3.0) * volume->resolution / 2;
                result.push_back(unit);
            }
        }
        return result;
    }

    if (!view.grid)
        return result;
    const auto& map = *view.grid;
    std::set<std::pair<int, int>> cells;
    for (int y = 0; y < map.height(); ++y) {
        for (int x = 0; x < map.width(); ++x) {
            if (map.At(x, y).state != autonomy::navigation::CellState::kFree) {
                continue;
            }
            for (const auto& offset : kPlanarNeighbors) {
                const int nx = x + offset.first;
                const int ny = y + offset.second;
                if (!map.Contains(nx, ny) ||
                    map.At(nx, ny).state !=
                        autonomy::navigation::CellState::kUnknown) {
                    continue;
                }
                const Pose2d center{
                    map.origin_x() + (nx + .5) * map.resolution(),
                    map.origin_y() + (ny + .5) * map.resolution(), 0};
                if (InsideDomain(center.x, center.y, allowed_region,
                                 allowed_polygon, no_go_areas)) {
                    cells.insert({nx, ny});
                }
            }
        }
    }

    while (!cells.empty()) {
        std::vector<std::pair<int, int>> cluster{*cells.begin()};
        cells.erase(cells.begin());
        for (std::size_t index = 0; index < cluster.size(); ++index) {
            for (const auto& offset : kPlanarNeighbors) {
                const std::pair<int, int> neighbor{
                    cluster[index].first + offset.first,
                    cluster[index].second + offset.second};
                const auto found = cells.find(neighbor);
                if (found != cells.end()) {
                    cluster.push_back(*found);
                    cells.erase(found);
                }
            }
        }
        if (cluster.size() < minimum_cluster_size)
            continue;
        for (const auto& cell : cluster) {
            const Pose2d center{
                map.origin_x() + (cell.first + .5) * map.resolution(),
                map.origin_y() + (cell.second + .5) * map.resolution(), 0};
            CoverageUnit unit;
            // Epoch owns the quantization lattice; IDs survive rolling origin
            // changes.
            unit.id = {
                static_cast<int>(std::floor(center.x / map.resolution())),
                static_cast<int>(std::floor(center.y / map.resolution())), 0, 0,
                CoverageKind::kFrontier};
            unit.center = {center.x, center.y, height};
            unit.area = map.resolution() * map.resolution();
            unit.tolerance = map.resolution() / std::sqrt(2.0);
            result.push_back(unit);
        }
    }
    return result;
}
}  // namespace autonomy::task::exploration
