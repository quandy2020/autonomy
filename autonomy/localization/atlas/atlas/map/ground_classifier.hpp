/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "atlas/map/map_types.hpp"

namespace autonomy::localization::atlas::map {

/**
 * Classifies a frame cloud for the navigation occupancy layer.
 *
 * DenseMap remains independent and keeps every finite ColorPoint. This class
 * only produces the ground/obstacle/unknown view used by OccupancyGridMap.
 */
class GroundClassifier {
public:
    explicit GroundClassifier(OccupancyGridOptions options);

    /**
     * Estimates a gravity-constrained ground plane when enabled, otherwise
     * uses the configured sensor-height plane. Estimation failure always
     * falls back to that deterministic plane.
     */
    bool Classify(
        const Eigen::Vector3f& sensor_origin,
        const std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>&
            points,
        SemanticPointCloud* classified_points,
        GroundPlane* ground_plane = nullptr,
        std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>*
            navigation_points = nullptr) const;

private:
    GroundPlane EstimateGroundPlane(
        const Eigen::Vector3d& sensor_origin,
        const std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>&
            points) const;
    GroundPlane SensorHeightPlane(
        const Eigen::Vector3d& sensor_origin) const;

    OccupancyGridOptions options_;
    Eigen::Vector3d up_ = Eigen::Vector3d::UnitZ();
};

}  // namespace autonomy::localization::atlas::map
