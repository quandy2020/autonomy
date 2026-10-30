/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "atlas/map/occupancy_grid_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <utility>

#if defined(__has_include)
#if __has_include(<opencv2/geometry/2d.hpp>)
#include <opencv2/geometry/2d.hpp>
#endif
#endif
#include <opencv2/imgproc.hpp>

namespace autonomy::localization::atlas::map {
namespace {

constexpr std::int8_t kMaximumInflatedCost = 99;

double LogOdds(double probability) {
    return std::log(probability / (1.0 - probability));
}

double Probability(double log_odds) {
    if (log_odds >= 0.0) {
        const double inverse = std::exp(-log_odds);
        return 1.0 / (1.0 + inverse);
    }
    const double value = std::exp(log_odds);
    return value / (1.0 + value);
}

std::uint64_t Mix(std::uint64_t value) {
    value ^= value >> 30U;
    value *= 0xbf58476d1ce4e5b9ULL;
    value ^= value >> 27U;
    value *= 0x94d049bb133111ebULL;
    return value ^ (value >> 31U);
}

bool IsProbability(double value) {
    return std::isfinite(value) && value > 0.0 && value < 1.0;
}

std::uint64_t CeilDivide(std::uint64_t value, std::uint64_t divisor) {
    return value / divisor + (value % divisor == 0U ? 0U : 1U);
}

std::size_t AngleBucket(const Eigen::Vector2d& delta,
                        std::size_t bucket_count) {
    constexpr double kPi = 3.14159265358979323846;
    const double normalized =
        (std::atan2(delta.y(), delta.x()) + kPi) / (2.0 * kPi);
    const double scaled =
        std::floor(normalized * static_cast<double>(bucket_count));
    if (scaled <= 0.0)
        return 0;
    return std::min(bucket_count - 1,
                    static_cast<std::size_t>(scaled));
}

struct ProjectedPoint {
    double u = 0.0;
    double v = 0.0;
    double height = 0.0;
};

ProjectedPoint Project(const Eigen::Vector3f& point,
                       OccupancyGridOptions::ProjectionPlane plane,
                       OccupancyGridOptions::UpAxis up_axis) {
    switch (plane) {
        case OccupancyGridOptions::ProjectionPlane::kXy:
            if (up_axis == OccupancyGridOptions::UpAxis::kPositiveZ)
                return {point.x(), point.y(), point.z()};
            if (up_axis == OccupancyGridOptions::UpAxis::kNegativeZ)
                return {point.x(), -point.y(), -point.z()};
            return {};
        case OccupancyGridOptions::ProjectionPlane::kXz:
            if (up_axis == OccupancyGridOptions::UpAxis::kPositiveY)
                return {point.x(), -point.z(), point.y()};
            if (up_axis == OccupancyGridOptions::UpAxis::kNegativeY)
                return {point.x(), point.z(), -point.y()};
            return {};
        case OccupancyGridOptions::ProjectionPlane::kYz:
            if (up_axis == OccupancyGridOptions::UpAxis::kPositiveX)
                return {point.y(), point.z(), point.x()};
            if (up_axis == OccupancyGridOptions::UpAxis::kNegativeX)
                return {point.y(), -point.z(), -point.x()};
            return {};
    }
    return {};
}

bool ValidProjectionPlane(OccupancyGridOptions::ProjectionPlane plane) {
    switch (plane) {
        case OccupancyGridOptions::ProjectionPlane::kXy:
        case OccupancyGridOptions::ProjectionPlane::kXz:
        case OccupancyGridOptions::ProjectionPlane::kYz:
            return true;
    }
    return false;
}

bool CompatibleProjection(OccupancyGridOptions::ProjectionPlane plane,
                          OccupancyGridOptions::UpAxis up_axis) {
    switch (plane) {
        case OccupancyGridOptions::ProjectionPlane::kXy:
            return up_axis == OccupancyGridOptions::UpAxis::kPositiveZ ||
                   up_axis == OccupancyGridOptions::UpAxis::kNegativeZ;
        case OccupancyGridOptions::ProjectionPlane::kXz:
            return up_axis == OccupancyGridOptions::UpAxis::kPositiveY ||
                   up_axis == OccupancyGridOptions::UpAxis::kNegativeY;
        case OccupancyGridOptions::ProjectionPlane::kYz:
            return up_axis == OccupancyGridOptions::UpAxis::kPositiveX ||
                   up_axis == OccupancyGridOptions::UpAxis::kNegativeX;
    }
    return false;
}

double EffectiveInflationRadius(const OccupancyGridOptions& options) {
    return options.inflation_radius > 0.0 ? options.inflation_radius
                                          : options.robot_radius;
}

bool ValidInflationOptions(const OccupancyGridOptions& options) {
    if (!std::isfinite(options.robot_radius) ||
        !std::isfinite(options.inflation_radius) ||
        !std::isfinite(options.inflation_cost_scaling) ||
        options.robot_radius < 0.0 || options.inflation_radius < 0.0 ||
        options.inflation_cost_scaling < 0.0) {
        return false;
    }
    if (options.inflation_radius == 0.0) {
        if (options.inflation_cost_scaling != 0.0)
            return false;
    } else if (options.inflation_radius < options.robot_radius ||
               options.inflation_cost_scaling <= 0.0) {
        return false;
    }

    return true;
}

}  // namespace

OccupancyGridMap::OccupancyGridMap(OccupancyGridOptions options)
    : options_(std::move(options)) {
    if (!ValidProjectionPlane(options_.projection_plane) ||
        !CompatibleProjection(options_.projection_plane, options_.up_axis) ||
        !std::isfinite(options_.resolution) || options_.resolution <= 0.0 ||
        !ValidInflationOptions(options_) ||
        !std::isfinite(options_.sensor_height) ||
        (options_.use_sensor_height && options_.sensor_height <= 0.0) ||
        !std::isfinite(options_.minimum_obstacle_height) ||
        !std::isfinite(options_.maximum_obstacle_height) ||
        options_.minimum_obstacle_height >
            options_.maximum_obstacle_height ||
        !std::isfinite(options_.minimum_navigation_height) ||
        !std::isfinite(options_.maximum_navigation_height) ||
        options_.minimum_navigation_height >
            options_.maximum_navigation_height ||
        !IsProbability(options_.hit_probability) ||
        !IsProbability(options_.miss_probability) ||
        !IsProbability(options_.minimum_probability) ||
        !IsProbability(options_.maximum_probability) ||
        !IsProbability(options_.occupied_probability) ||
        options_.minimum_probability >= options_.maximum_probability ||
        !std::isfinite(options_.maximum_extent) ||
        options_.maximum_extent <= 0.0 ||
        options_.maximum_rays_per_frame == 0 || options_.maximum_cells == 0 ||
        options_.ray_angle_bins == 0 ||
        options_.ray_angle_bins > options_.maximum_rays_per_frame ||
        options_.maximum_snapshot_cells == 0) {
        throw std::invalid_argument("invalid occupancy-grid options");
    }
    hit_log_odds_ = LogOdds(options_.hit_probability);
    miss_log_odds_ = LogOdds(options_.miss_probability);
    minimum_log_odds_ = LogOdds(options_.minimum_probability);
    maximum_log_odds_ = LogOdds(options_.maximum_probability);
    cells_.reserve(std::min<std::size_t>(options_.maximum_cells, 200000));
}

std::size_t OccupancyGridMap::CellKeyHash::operator()(
    const CellKey& key) const {
    const auto x = static_cast<std::uint32_t>(key.x);
    const auto y = static_cast<std::uint32_t>(key.y);
    return static_cast<std::size_t>(Mix(x) ^ (Mix(y) << 1U));
}

bool OccupancyGridMap::KeyForPoint(double x, double y, CellKey* key) const {
    if (key == nullptr || !std::isfinite(x) || !std::isfinite(y)) {
        return false;
    }
    const double grid_x = std::floor(x / options_.resolution);
    const double grid_y = std::floor(y / options_.resolution);
    constexpr double kMinimum =
        static_cast<double>(std::numeric_limits<std::int32_t>::min());
    constexpr double kMaximum =
        static_cast<double>(std::numeric_limits<std::int32_t>::max());
    if (grid_x < kMinimum || grid_x > kMaximum || grid_y < kMinimum ||
        grid_y > kMaximum) {
        return false;
    }
    key->x = static_cast<std::int32_t>(grid_x);
    key->y = static_cast<std::int32_t>(grid_y);
    return true;
}

void OccupancyGridMap::AddRay(
    const CellKey& begin, const CellKey& end, bool endpoint_is_occupied,
    std::unordered_set<CellKey, CellKeyHash>* free_cells) const {
    if (free_cells == nullptr)
        return;
    std::int64_t x = begin.x;
    std::int64_t y = begin.y;
    const std::int64_t end_x = end.x;
    const std::int64_t end_y = end.y;
    const std::int64_t delta_x = std::llabs(end_x - x);
    const std::int64_t delta_y = std::llabs(end_y - y);
    const std::int64_t step_x = x < end_x ? 1 : -1;
    const std::int64_t step_y = y < end_y ? 1 : -1;
    std::int64_t error = delta_x - delta_y;
    const std::uint64_t maximum_steps =
        static_cast<std::uint64_t>(delta_x + delta_y) + 1U;

    for (std::uint64_t step = 0; step < maximum_steps; ++step) {
        const bool endpoint = x == end_x && y == end_y;
        if (!endpoint || !endpoint_is_occupied) {
            free_cells->insert(CellKey{static_cast<std::int32_t>(x),
                                       static_cast<std::int32_t>(y)});
        }
        if (endpoint)
            break;
        const std::int64_t twice_error = 2 * error;
        if (twice_error > -delta_y) {
            error -= delta_y;
            x += step_x;
        }
        if (twice_error < delta_x) {
            error += delta_x;
            y += step_y;
        }
    }
}

bool OccupancyGridMap::Update(const CellKey& key, double delta) {
    auto iterator = cells_.find(key);
    if (iterator == cells_.end()) {
        if (cells_.size() >= options_.maximum_cells)
            return false;
        iterator = cells_.emplace(key, 0.0F).first;
    }
    iterator->second = static_cast<float>(
        std::clamp(static_cast<double>(iterator->second) + delta,
                   minimum_log_odds_, maximum_log_odds_));
    return true;
}

void OccupancyGridMap::ApplyInflation(
    OccupancyGridSnapshot* snapshot) const {
    if (snapshot == nullptr || snapshot->width == 0U ||
        snapshot->height == 0U || snapshot->data.empty()) {
        return;
    }
    const double maximum_radius = EffectiveInflationRadius(options_);
    if (maximum_radius <= 0.0)
        return;
    if (snapshot->width >
            static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
        snapshot->height >
            static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
        return;
    }

    cv::Mat obstacle_mask(static_cast<int>(snapshot->height),
                          static_cast<int>(snapshot->width), CV_8UC1,
                          cv::Scalar(255));
    bool have_obstacle = false;
    for (std::uint32_t y = 0; y < snapshot->height; ++y) {
        std::uint8_t* mask = obstacle_mask.ptr<std::uint8_t>(
            static_cast<int>(y));
        const std::size_t row = static_cast<std::size_t>(y) *
                                snapshot->width;
        for (std::uint32_t x = 0; x < snapshot->width; ++x) {
            if (snapshot->data[row + x] == 100) {
                mask[x] = 0;
                have_obstacle = true;
            }
        }
    }
    if (!have_obstacle)
        return;

    cv::Mat distance_pixels;
    cv::distanceTransform(obstacle_mask, distance_pixels, cv::DIST_L2,
                          cv::DIST_MASK_PRECISE);
    for (std::uint32_t y = 0; y < snapshot->height; ++y) {
        const float* distances = distance_pixels.ptr<float>(
            static_cast<int>(y));
        const std::size_t row = static_cast<std::size_t>(y) *
                                snapshot->width;
        for (std::uint32_t x = 0; x < snapshot->width; ++x) {
            std::int8_t& cell = snapshot->data[row + x];
            if (cell != 0)
                continue;
            const double distance =
                static_cast<double>(distances[x]) * snapshot->resolution;
            if (!std::isfinite(distance) || distance > maximum_radius)
                continue;

            int cost = kMaximumInflatedCost;
            if (distance > options_.robot_radius) {
                const double scaled =
                    static_cast<double>(kMaximumInflatedCost) *
                    std::exp(-options_.inflation_cost_scaling *
                             (distance - options_.robot_radius));
                cost = std::clamp(static_cast<int>(std::lround(scaled)), 1,
                                  static_cast<int>(kMaximumInflatedCost));
            }
            cell = static_cast<std::int8_t>(cost);
        }
    }
}

std::size_t OccupancyGridMap::Integrate(
    const Eigen::Vector3f& sensor_origin,
    const std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>&
        points) {
    if (!sensor_origin.allFinite())
        return 0;
    const ProjectedPoint projected_origin =
        Project(sensor_origin, options_.projection_plane, options_.up_axis);
    SemanticPointCloud semantic_points;
    semantic_points.reserve(points.size());
    for (const ColorPoint& point : points) {
        if (!point.position.allFinite())
            continue;
        const ProjectedPoint projected_point =
            Project(point.position, options_.projection_plane,
                    options_.up_axis);
        const double relative_sensor_height =
            projected_point.height - projected_origin.height;
        const double obstacle_height =
            options_.use_sensor_height
                ? relative_sensor_height + options_.sensor_height
                : relative_sensor_height;
        const bool obstacle =
            obstacle_height >= options_.minimum_obstacle_height &&
            obstacle_height <= options_.maximum_obstacle_height;
        PointSemantic semantic = PointSemantic::kUnknown;
        if (obstacle) {
            semantic = PointSemantic::kObstacle;
        } else if (obstacle_height < options_.minimum_obstacle_height) {
            semantic = PointSemantic::kGround;
        }
        semantic_points.push_back(SemanticPoint{point.position, semantic});
    }
    return Integrate(sensor_origin, semantic_points);
}

std::size_t OccupancyGridMap::Integrate(
    const Eigen::Vector3f& sensor_origin,
    const SemanticPointCloud& points) {
    if (!sensor_origin.allFinite())
        return 0;
    const ProjectedPoint projected_origin =
        Project(sensor_origin, options_.projection_plane, options_.up_axis);
    CellKey origin;
    if (!KeyForPoint(projected_origin.u, projected_origin.v, &origin))
        return 0;

    struct GroundObservation {
        CellKey endpoint;
        std::size_t angle_bucket = 0;
        double squared_distance = 0.0;
    };
    struct NearestObstacle {
        CellKey endpoint;
        double squared_distance = std::numeric_limits<double>::infinity();
        bool valid = false;
    };

    std::unordered_set<CellKey, CellKeyHash> free_cells;
    std::unordered_set<CellKey, CellKeyHash> occupied_cells;
    free_cells.reserve(points.size() * 4U);
    occupied_cells.reserve(points.size());
    std::vector<GroundObservation> ground_observations;
    ground_observations.reserve(points.size());
    std::vector<NearestObstacle> nearest_obstacles(
        options_.ray_angle_bins);
    const double maximum_squared_extent =
        options_.maximum_extent * options_.maximum_extent;
    for (const SemanticPoint& point : points) {
        if (!point.position.allFinite() ||
            point.semantic == PointSemantic::kUnknown) {
            continue;
        }
        const ProjectedPoint projected_point =
            Project(point.position, options_.projection_plane,
                    options_.up_axis);
        const Eigen::Vector2d delta(projected_point.u - projected_origin.u,
                                    projected_point.v - projected_origin.v);
        const double squared_distance = delta.squaredNorm();
        if (!delta.allFinite() || !std::isfinite(squared_distance) ||
            squared_distance > maximum_squared_extent)
            continue;
        CellKey endpoint;
        if (!KeyForPoint(projected_point.u, projected_point.v, &endpoint))
            continue;
        const bool obstacle = point.semantic == PointSemantic::kObstacle;
        const std::size_t angle_bucket =
            AngleBucket(delta, options_.ray_angle_bins);
        if (obstacle) {
            occupied_cells.insert(endpoint);
            NearestObstacle& nearest = nearest_obstacles[angle_bucket];
            if (!nearest.valid ||
                squared_distance < nearest.squared_distance) {
                nearest.endpoint = endpoint;
                nearest.squared_distance = squared_distance;
                nearest.valid = true;
            }
        } else {
            ground_observations.push_back(
                GroundObservation{endpoint, angle_bucket,
                                  squared_distance});
        }
    }

    std::size_t traced_rays = 0;
    for (const NearestObstacle& nearest : nearest_obstacles) {
        if (!nearest.valid)
            continue;
        AddRay(origin, nearest.endpoint, true, &free_cells);
        ++traced_rays;
    }

    // A ground return cannot observe free space through the closest obstacle
    // in its angular bucket. Obstacle endpoints are all retained as hits, but
    // only the nearest obstacle in each bucket contributes a clearing ray.
    const std::size_t ground_ray_budget =
        options_.maximum_rays_per_frame - traced_rays;
    if (ground_ray_budget > 0U) {
        const std::size_t ground_stride = std::max<std::size_t>(
            1U, (ground_observations.size() + ground_ray_budget - 1U) /
                    ground_ray_budget);
        for (std::size_t index = 0; index < ground_observations.size();
             index += ground_stride) {
            const GroundObservation& observation = ground_observations[index];
            const NearestObstacle& nearest =
                nearest_obstacles[observation.angle_bucket];
            if (nearest.valid &&
                observation.squared_distance >= nearest.squared_distance) {
                AddRay(origin, nearest.endpoint, true, &free_cells);
            } else {
                AddRay(origin, observation.endpoint, false, &free_cells);
            }
        }
    }
    for (const CellKey& key : occupied_cells)
        free_cells.erase(key);

    std::size_t updated = 0;
    for (const CellKey& key : free_cells)
        updated += Update(key, miss_log_odds_) ? 1U : 0U;
    for (const CellKey& key : occupied_cells)
        updated += Update(key, hit_log_odds_) ? 1U : 0U;
    return updated;
}

bool OccupancyGridMap::ProbabilityAt(double x, double y,
                                     double* probability) const {
    if (probability == nullptr)
        return false;
    CellKey key;
    if (!KeyForPoint(x, y, &key))
        return false;
    const auto iterator = cells_.find(key);
    if (iterator == cells_.end())
        return false;
    *probability = Probability(iterator->second);
    return true;
}

OccupancyGridSnapshot OccupancyGridMap::Snapshot(
    double timestamp, std::uint64_t generation) const {
    OccupancyGridSnapshot snapshot;
    snapshot.timestamp = timestamp;
    snapshot.generation = generation;
    snapshot.resolution = options_.resolution;
    if (cells_.empty())
        return snapshot;

    std::int32_t minimum_x = std::numeric_limits<std::int32_t>::max();
    std::int32_t minimum_y = std::numeric_limits<std::int32_t>::max();
    std::int32_t maximum_x = std::numeric_limits<std::int32_t>::min();
    std::int32_t maximum_y = std::numeric_limits<std::int32_t>::min();
    for (const auto& [key, value] : cells_) {
        static_cast<void>(value);
        minimum_x = std::min(minimum_x, key.x);
        minimum_y = std::min(minimum_y, key.y);
        maximum_x = std::max(maximum_x, key.x);
        maximum_y = std::max(maximum_y, key.y);
    }
    const std::uint64_t source_width =
        static_cast<std::uint64_t>(static_cast<std::int64_t>(maximum_x) -
                                   static_cast<std::int64_t>(minimum_x)) +
        1U;
    const std::uint64_t source_height =
        static_cast<std::uint64_t>(static_cast<std::int64_t>(maximum_y) -
                                   static_cast<std::int64_t>(minimum_y)) +
        1U;
    const long double source_area =
        static_cast<long double>(source_width) * source_height;
    std::uint64_t downsample = 1U;
    if (source_area >
        static_cast<long double>(options_.maximum_snapshot_cells)) {
        downsample = static_cast<std::uint64_t>(std::ceil(std::sqrt(
            source_area / options_.maximum_snapshot_cells)));
        downsample = std::max<std::uint64_t>(downsample, 1U);
    }
    std::uint64_t width = CeilDivide(source_width, downsample);
    std::uint64_t height = CeilDivide(source_height, downsample);
    while (height != 0U &&
           width > options_.maximum_snapshot_cells / height) {
        ++downsample;
        width = CeilDivide(source_width, downsample);
        height = CeilDivide(source_height, downsample);
    }
    if (width > std::numeric_limits<std::uint32_t>::max() ||
        height > std::numeric_limits<std::uint32_t>::max() ||
        height == 0U || width > std::numeric_limits<std::size_t>::max() /
                                    height) {
        return snapshot;
    }
    const std::uint64_t area = width * height;

    snapshot.origin_x = static_cast<double>(minimum_x) * options_.resolution;
    snapshot.origin_y = static_cast<double>(minimum_y) * options_.resolution;
    snapshot.resolution = options_.resolution * downsample;
    Eigen::Matrix3d grid_to_world = Eigen::Matrix3d::Identity();
    switch (options_.projection_plane) {
        case OccupancyGridOptions::ProjectionPlane::kXy:
            if (options_.up_axis ==
                OccupancyGridOptions::UpAxis::kPositiveZ) {
                snapshot.origin_translation = Eigen::Vector3d(
                    snapshot.origin_x, snapshot.origin_y, 0.0);
            } else {
                snapshot.origin_translation = Eigen::Vector3d(
                    snapshot.origin_x, -snapshot.origin_y, 0.0);
                grid_to_world.col(1) = -Eigen::Vector3d::UnitY();
                grid_to_world.col(2) = -Eigen::Vector3d::UnitZ();
            }
            break;
        case OccupancyGridOptions::ProjectionPlane::kXz:
            grid_to_world.col(0) = Eigen::Vector3d::UnitX();
            if (options_.up_axis ==
                OccupancyGridOptions::UpAxis::kPositiveY) {
                snapshot.origin_translation = Eigen::Vector3d(
                    snapshot.origin_x, 0.0, -snapshot.origin_y);
                grid_to_world.col(1) = -Eigen::Vector3d::UnitZ();
                grid_to_world.col(2) = Eigen::Vector3d::UnitY();
            } else {
                snapshot.origin_translation = Eigen::Vector3d(
                    snapshot.origin_x, 0.0, snapshot.origin_y);
                grid_to_world.col(1) = Eigen::Vector3d::UnitZ();
                grid_to_world.col(2) = -Eigen::Vector3d::UnitY();
            }
            break;
        case OccupancyGridOptions::ProjectionPlane::kYz:
            grid_to_world.col(0) = Eigen::Vector3d::UnitY();
            if (options_.up_axis ==
                OccupancyGridOptions::UpAxis::kPositiveX) {
                snapshot.origin_translation = Eigen::Vector3d(
                    0.0, snapshot.origin_x, snapshot.origin_y);
                grid_to_world.col(1) = Eigen::Vector3d::UnitZ();
                grid_to_world.col(2) = Eigen::Vector3d::UnitX();
            } else {
                snapshot.origin_translation = Eigen::Vector3d(
                    0.0, snapshot.origin_x, -snapshot.origin_y);
                grid_to_world.col(1) = -Eigen::Vector3d::UnitZ();
                grid_to_world.col(2) = -Eigen::Vector3d::UnitX();
            }
            break;
    }
    snapshot.origin_rotation = Eigen::Quaterniond(grid_to_world);
    snapshot.origin_rotation.normalize();
    snapshot.width = static_cast<std::uint32_t>(width);
    snapshot.height = static_cast<std::uint32_t>(height);
    snapshot.data.assign(static_cast<std::size_t>(area),
                         static_cast<std::int8_t>(-1));
    for (const auto& [key, value] : cells_) {
        const std::uint64_t x = static_cast<std::uint64_t>(
            static_cast<std::int64_t>(key.x) - minimum_x) /
            downsample;
        const std::uint64_t y = static_cast<std::uint64_t>(
            static_cast<std::int64_t>(key.y) - minimum_y) /
            downsample;
        const std::uint64_t index = y * width + x;
        std::int8_t& cell = snapshot.data[static_cast<std::size_t>(index)];
        const bool occupied =
            Probability(value) >= options_.occupied_probability;
        if (occupied || cell < 0)
            cell = occupied ? 100 : 0;
    }
    ApplyInflation(&snapshot);
    return snapshot;
}

void OccupancyGridMap::Reset() {
    cells_.clear();
    cells_.reserve(std::min<std::size_t>(options_.maximum_cells, 200000));
}

}  // namespace autonomy::localization::atlas::map
