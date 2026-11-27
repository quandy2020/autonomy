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
const std::array<Eigen::Vector3i, 4> kHorizontalVolumeNeighbors{{
    {1, 0, 0},
    {-1, 0, 0},
    {0, 1, 0},
    {0, -1, 0},
}};

using VoxelKey = std::tuple<int, int, int>;

VoxelKey Key(const Eigen::Vector3i& index) {
    return {index.x(), index.y(), index.z()};
}

Eigen::Vector3i Index(const VoxelKey& key) {
    return {std::get<0>(key), std::get<1>(key), std::get<2>(key)};
}

int MetricBucketWidth(double resolution, double tolerance) {
    const double width = std::ceil(tolerance / resolution);
    if (!std::isfinite(width) || width >= std::numeric_limits<int>::max()) {
        return std::numeric_limits<int>::max();
    }
    return std::max(1, static_cast<int>(width));
}

VoxelKey Bucket(const VoxelKey& key, int width) {
    return {std::get<0>(key) / width, std::get<1>(key) / width,
            std::get<2>(key) / width};
}

std::pair<int, int> Bucket(const std::pair<int, int>& cell, int width) {
    return {cell.first / width, cell.second / width};
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
    const std::vector<::automsgs::msgs::geometry_msgs::Polygon>& no_go_areas,
    double cluster_tolerance_m) const {
    std::vector<CoverageUnit> result;
    if (!std::isfinite(height) || !minimum_cluster_size ||
        !std::isfinite(cluster_tolerance_m) || cluster_tolerance_m <= 0 ||
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
            // Ground exploration retains vertical frontier surfaces only.
            // Missing voxels above or below measured free space are
            // observation-volume caps, not traversable boundary work.
            for (const auto& offset : kHorizontalVolumeNeighbors) {
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

        const int bucket_width =
            MetricBucketWidth(volume->resolution, cluster_tolerance_m);
        std::map<VoxelKey, std::set<VoxelKey>> buckets;
        for (const auto& cell : cells) {
            buckets[Bucket(cell, bucket_width)].insert(cell);
        }

        while (!cells.empty()) {
            std::vector<VoxelKey> cluster{*cells.begin()};
            cells.erase(cells.begin());
            buckets[Bucket(cluster.front(), bucket_width)].erase(
                cluster.front());
            for (std::size_t index = 0; index < cluster.size(); ++index) {
                const Eigen::Vector3i cell = Index(cluster[index]);
                const Eigen::Vector3i bucket =
                    Index(Bucket(cluster[index], bucket_width));
                // A tolerance-sized spatial index limits each metric range
                // query to 27 deterministic neighboring buckets while retaining
                // transitive links.
                for (int dz = -1; dz <= 1; ++dz) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            auto& candidates = buckets[Key(
                                bucket + Eigen::Vector3i{dx, dy, dz})];
                            for (auto candidate = candidates.begin();
                                 candidate != candidates.end();) {
                                const Eigen::Vector3d metric_delta =
                                    (Index(*candidate) - cell).cast<double>() *
                                    volume->resolution;
                                if (metric_delta.squaredNorm() <=
                                    cluster_tolerance_m * cluster_tolerance_m) {
                                    cluster.push_back(*candidate);
                                    cells.erase(*candidate);
                                    candidate = candidates.erase(candidate);
                                } else {
                                    ++candidate;
                                }
                            }
                        }
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

    const int bucket_width =
        MetricBucketWidth(map.resolution(), cluster_tolerance_m);
    std::map<std::pair<int, int>, std::set<std::pair<int, int>>> buckets;
    for (const auto& cell : cells) {
        buckets[Bucket(cell, bucket_width)].insert(cell);
    }

    while (!cells.empty()) {
        std::vector<std::pair<int, int>> cluster{*cells.begin()};
        cells.erase(cells.begin());
        buckets[Bucket(cluster.front(), bucket_width)].erase(cluster.front());
        for (std::size_t index = 0; index < cluster.size(); ++index) {
            const auto bucket = Bucket(cluster[index], bucket_width);
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    auto& candidates =
                        buckets[{bucket.first + dx, bucket.second + dy}];
                    for (auto candidate = candidates.begin();
                         candidate != candidates.end();) {
                        const double dx =
                            (candidate->first - cluster[index].first) *
                            map.resolution();
                        const double dy =
                            (candidate->second - cluster[index].second) *
                            map.resolution();
                        if (dx * dx + dy * dy <=
                            cluster_tolerance_m * cluster_tolerance_m) {
                            cluster.push_back(*candidate);
                            cells.erase(*candidate);
                            candidate = candidates.erase(candidate);
                        } else {
                            ++candidate;
                        }
                    }
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
