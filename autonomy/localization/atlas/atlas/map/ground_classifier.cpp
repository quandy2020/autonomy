/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "atlas/map/ground_classifier.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Eigenvalues>

namespace autonomy::localization::atlas::map {
namespace {

using EstimationPoints =
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

constexpr double kHalfPi = 1.57079632679489661923;

Eigen::Vector3d UpVector(OccupancyGridOptions::UpAxis axis) {
    switch (axis) {
        case OccupancyGridOptions::UpAxis::kPositiveX:
            return Eigen::Vector3d::UnitX();
        case OccupancyGridOptions::UpAxis::kNegativeX:
            return -Eigen::Vector3d::UnitX();
        case OccupancyGridOptions::UpAxis::kPositiveY:
            return Eigen::Vector3d::UnitY();
        case OccupancyGridOptions::UpAxis::kNegativeY:
            return -Eigen::Vector3d::UnitY();
        case OccupancyGridOptions::UpAxis::kPositiveZ:
            return Eigen::Vector3d::UnitZ();
        case OccupancyGridOptions::UpAxis::kNegativeZ:
            return -Eigen::Vector3d::UnitZ();
    }
    return Eigen::Vector3d::Zero();
}

bool ValidOptions(const OccupancyGridOptions& options) {
    return std::isfinite(options.ground_distance_threshold) &&
           options.ground_distance_threshold > 0.0 &&
           std::isfinite(options.minimum_obstacle_height) &&
           std::isfinite(options.maximum_obstacle_height) &&
           options.minimum_obstacle_height <=
               options.maximum_obstacle_height &&
           std::isfinite(options.ground_candidate_height_margin) &&
           options.ground_candidate_height_margin >= 0.0 &&
           std::isfinite(options.maximum_ground_slope_radians) &&
           options.maximum_ground_slope_radians >= 0.0 &&
           options.maximum_ground_slope_radians < kHalfPi &&
           std::isfinite(options.ground_sensor_height_tolerance) &&
           options.ground_sensor_height_tolerance >= 0.0 &&
           std::isfinite(options.maximum_ground_estimation_range) &&
           options.maximum_ground_estimation_range > 0.0 &&
           options.minimum_ground_inliers >= 3U &&
           options.maximum_ground_estimation_points >=
               options.minimum_ground_inliers &&
           options.ground_ransac_iterations > 0U &&
           std::isfinite(options.minimum_navigation_height) &&
           std::isfinite(options.maximum_navigation_height) &&
           options.minimum_navigation_height <=
               options.maximum_navigation_height;
}

std::size_t NextIndex(std::uint64_t* state, std::size_t count) {
    // A deterministic generator keeps tests and replays reproducible.
    *state = *state * 6364136223846793005ULL + 1442695040888963407ULL;
    return static_cast<std::size_t>((*state >> 32U) % count);
}

bool BuildPlane(const Eigen::Vector3d& first,
                const Eigen::Vector3d& second,
                const Eigen::Vector3d& third,
                const Eigen::Vector3d& up,
                GroundPlane* plane) {
    if (plane == nullptr)
        return false;
    Eigen::Vector3d normal =
        (second - first).cross(third - first);
    const double norm = normal.norm();
    if (!std::isfinite(norm) || norm < 1e-8)
        return false;
    normal /= norm;
    if (normal.dot(up) < 0.0)
        normal = -normal;
    plane->normal = normal;
    plane->offset = -normal.dot(first);
    plane->valid = true;
    plane->estimated = true;
    return true;
}

bool PlaneMatchesPrior(const GroundPlane& plane,
                       const Eigen::Vector3d& sensor_origin,
                       const Eigen::Vector3d& up,
                       const OccupancyGridOptions& options) {
    if (!plane.valid || !plane.normal.allFinite() ||
        !std::isfinite(plane.offset)) {
        return false;
    }
    const double cosine_limit =
        std::cos(options.maximum_ground_slope_radians);
    if (plane.normal.dot(up) < cosine_limit)
        return false;
    const double sensor_distance =
        plane.normal.dot(sensor_origin) + plane.offset;
    return sensor_distance > 0.0 &&
           std::abs(sensor_distance - options.sensor_height) <=
               options.ground_sensor_height_tolerance;
}

}  // namespace

GroundClassifier::GroundClassifier(OccupancyGridOptions options)
    : options_(std::move(options)), up_(UpVector(options_.up_axis)) {
    if (!up_.allFinite() || up_.squaredNorm() < 0.5 ||
        !ValidOptions(options_) ||
        (options_.estimate_ground_plane &&
         (!options_.use_sensor_height || options_.sensor_height <= 0.0))) {
        throw std::invalid_argument("invalid ground-classifier options");
    }
}

GroundPlane GroundClassifier::SensorHeightPlane(
    const Eigen::Vector3d& sensor_origin) const {
    GroundPlane plane;
    plane.normal = up_;
    plane.offset = -up_.dot(sensor_origin) + options_.sensor_height;
    plane.valid = sensor_origin.allFinite() &&
                  std::isfinite(options_.sensor_height) &&
                  options_.sensor_height > 0.0;
    plane.estimated = false;
    return plane;
}

GroundPlane GroundClassifier::EstimateGroundPlane(
    const Eigen::Vector3d& sensor_origin,
    const std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>&
        points) const {
    const GroundPlane fallback = SensorHeightPlane(sensor_origin);
    if (!options_.estimate_ground_plane || !fallback.valid ||
        points.size() < options_.minimum_ground_inliers) {
        return fallback;
    }

    EstimationPoints candidates;
    candidates.reserve(std::min(points.size(),
                                options_.maximum_ground_estimation_points));
    const std::size_t stride = std::max<std::size_t>(
        1U, points.size() / options_.maximum_ground_estimation_points);
    const double slope = std::tan(options_.maximum_ground_slope_radians);
    for (std::size_t index = 0;
         index < points.size() &&
         candidates.size() < options_.maximum_ground_estimation_points;
         index += stride) {
        const Eigen::Vector3d point = points[index].position.cast<double>();
        if (!point.allFinite())
            continue;
        const Eigen::Vector3d delta = point - sensor_origin;
        const double vertical = up_.dot(delta);
        const Eigen::Vector3d horizontal = delta - vertical * up_;
        const double range = horizontal.norm();
        if (!std::isfinite(range) ||
            range > options_.maximum_ground_estimation_range) {
            continue;
        }
        const double candidate_limit =
            options_.ground_candidate_height_margin + slope * range;
        if (std::abs(vertical + options_.sensor_height) <= candidate_limit)
            candidates.push_back(point);
    }
    if (candidates.size() < options_.minimum_ground_inliers)
        return fallback;

    GroundPlane best;
    std::size_t best_inliers = 0;
    double best_error = std::numeric_limits<double>::infinity();
    std::uint64_t random_state =
        0x9e3779b97f4a7c15ULL ^
        static_cast<std::uint64_t>(candidates.size());
    for (std::size_t iteration = 0;
         iteration < options_.ground_ransac_iterations; ++iteration) {
        const std::size_t first =
            NextIndex(&random_state, candidates.size());
        std::size_t second = NextIndex(&random_state, candidates.size());
        while (second == first)
            second = NextIndex(&random_state, candidates.size());
        std::size_t third = NextIndex(&random_state, candidates.size());
        while (third == first || third == second)
            third = NextIndex(&random_state, candidates.size());

        GroundPlane hypothesis;
        if (!BuildPlane(candidates[first], candidates[second],
                        candidates[third], up_, &hypothesis) ||
            !PlaneMatchesPrior(hypothesis, sensor_origin, up_, options_)) {
            continue;
        }
        std::size_t inliers = 0;
        double error = 0.0;
        for (const Eigen::Vector3d& point : candidates) {
            const double residual = std::abs(
                hypothesis.normal.dot(point) + hypothesis.offset);
            if (residual <= options_.ground_distance_threshold) {
                ++inliers;
                error += residual;
            }
        }
        if (inliers > best_inliers ||
            (inliers == best_inliers && error < best_error)) {
            best = hypothesis;
            best_inliers = inliers;
            best_error = error;
        }
    }
    if (best_inliers < options_.minimum_ground_inliers)
        return fallback;

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    std::size_t refinement_count = 0;
    for (const Eigen::Vector3d& point : candidates) {
        if (std::abs(best.normal.dot(point) + best.offset) <=
            options_.ground_distance_threshold) {
            centroid += point;
            ++refinement_count;
        }
    }
    if (refinement_count < options_.minimum_ground_inliers)
        return best;
    centroid /= static_cast<double>(refinement_count);

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const Eigen::Vector3d& point : candidates) {
        if (std::abs(best.normal.dot(point) + best.offset) >
            options_.ground_distance_threshold) {
            continue;
        }
        const Eigen::Vector3d centered = point - centroid;
        covariance.noalias() += centered * centered.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success)
        return best;
    GroundPlane refined;
    refined.normal = solver.eigenvectors().col(0).normalized();
    if (refined.normal.dot(up_) < 0.0)
        refined.normal = -refined.normal;
    refined.offset = -refined.normal.dot(centroid);
    refined.valid = true;
    refined.estimated = true;
    return PlaneMatchesPrior(refined, sensor_origin, up_, options_) ? refined
                                                                   : best;
}

bool GroundClassifier::Classify(
    const Eigen::Vector3f& sensor_origin,
    const std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>&
        points,
    SemanticPointCloud* classified_points,
    GroundPlane* ground_plane,
    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>*
        navigation_points) const {
    if (classified_points == nullptr || !sensor_origin.allFinite())
        return false;
    classified_points->clear();
    classified_points->reserve(points.size());
    if (navigation_points != nullptr) {
        navigation_points->clear();
        navigation_points->reserve(points.size());
    }
    const Eigen::Vector3d origin = sensor_origin.cast<double>();

    // Use a conservative sensor-relative fallback when no physical sensor
    // height was configured. High points remain unknown instead of producing
    // free-space rays, while navigation still applies its configured band.
    if (!options_.use_sensor_height) {
        for (const ColorPoint& point : points) {
            if (!point.position.allFinite())
                continue;
            const double relative_height =
                up_.dot(point.position.cast<double>() - origin);
            PointSemantic semantic = PointSemantic::kUnknown;
            if (relative_height >= options_.minimum_obstacle_height &&
                relative_height <= options_.maximum_obstacle_height) {
                semantic = PointSemantic::kObstacle;
            } else if (relative_height <
                       options_.minimum_obstacle_height) {
                semantic = PointSemantic::kGround;
            }
            classified_points->push_back({point.position, semantic});
            // Without a metric ground prior this is a conservative
            // sensor-relative fallback. Production navigation must configure
            // SensorHeight so the band is ground-relative.
            if (navigation_points != nullptr &&
                relative_height >= options_.minimum_navigation_height &&
                relative_height <= options_.maximum_navigation_height) {
                navigation_points->push_back(point);
            }
        }
        if (ground_plane != nullptr)
            *ground_plane = GroundPlane{};
        return !classified_points->empty();
    }

    const GroundPlane plane = EstimateGroundPlane(origin, points);
    if (!plane.valid)
        return false;
    for (const ColorPoint& point : points) {
        if (!point.position.allFinite())
            continue;
        const double height =
            plane.normal.dot(point.position.cast<double>()) + plane.offset;
        PointSemantic semantic = PointSemantic::kUnknown;
        if (std::abs(height) <= options_.ground_distance_threshold) {
            semantic = PointSemantic::kGround;
        } else if (height >= options_.minimum_obstacle_height &&
                   height <= options_.maximum_obstacle_height) {
            semantic = PointSemantic::kObstacle;
        }
        classified_points->push_back({point.position, semantic});
        if (navigation_points != nullptr &&
            height >= options_.minimum_navigation_height &&
            height <= options_.maximum_navigation_height) {
            navigation_points->push_back(point);
        }
    }
    if (ground_plane != nullptr)
        *ground_plane = plane;
    return !classified_points->empty();
}

}  // namespace autonomy::localization::atlas::map
