/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#include "autonomy/map/esdf/esdf_map.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

namespace {

using autonomy::map::esdf::EsdfMap;
using autonomy::map::esdf::Options;

Options TinyMap() {
    Options options;
    options.origin = Eigen::Vector3d(-1.0, -1.0, -0.5);
    options.map_size = Eigen::Vector3d(2.0, 2.0, 1.0);
    options.resolution = 0.2;
    options.min_ray_length = 0.0;
    options.max_ray_length = 10.0;
    return options;
}

void Hit(EsdfMap* map, const Eigen::Vector3d& pos, int times) {
    for (int i = 0; i < times; ++i) {
        map->SetOccupancy(pos, true);
        map->UpdateOccupancy(true);
    }
}

}  // namespace

TEST(EsdfMap, OccupiedVoxelHasZeroDistance) {
    EsdfMap map(TinyMap());
    Hit(&map, Eigen::Vector3d(0.0, 0.0, 0.0), 3);
    map.UpdateESDF();
    EXPECT_NEAR(map.GetDistance(Eigen::Vector3d(0.0, 0.0, 0.0)), 0.0, 1e-9);
    EXPECT_EQ(map.GetOccupancy(Eigen::Vector3d(0.0, 0.0, 0.0)), 1);
    EXPECT_TRUE(map.CheckConsistency());
}

TEST(EsdfMap, DistanceGrowsAwayFromObstacle) {
    EsdfMap map(TinyMap());
    Hit(&map, Eigen::Vector3d(0.0, 0.0, 0.0), 3);
    map.UpdateESDF();
    const double near = map.GetDistance(Eigen::Vector3d(0.31, 0.0, 0.0));
    const double far = map.GetDistance(Eigen::Vector3d(0.71, 0.0, 0.0));
    EXPECT_GT(near, 0.0);
    EXPECT_GT(far, near);
}

TEST(EsdfMap, IntegrateRayMarksEndpointAndFreesRay) {
    EsdfMap map(TinyMap());
    const Eigen::Vector3d origin(-0.9, 0.0, 0.0);
    const Eigen::Vector3d hit(0.7, 0.0, 0.0);
    for (int i = 0; i < 3; ++i) {
        map.IntegrateRay(origin, hit);
        map.UpdateOccupancy(true);
    }
    map.UpdateESDF();
    EXPECT_EQ(map.GetOccupancy(hit), 1);
    EXPECT_NEAR(map.GetDistance(hit), 0.0, 1e-6);
    EXPECT_FALSE(map.OccupiedPoints().empty());
}

TEST(EsdfMap, GradientPointsAwayFromObstacle) {
    EsdfMap map(TinyMap());
    Hit(&map, Eigen::Vector3d(0.0, 0.0, 0.0), 4);
    map.UpdateESDF();
    Eigen::Vector3d grad = Eigen::Vector3d::Zero();
    const double dist =
        map.GetDistanceWithGradient(Eigen::Vector3d(0.4, 0.0, 0.0), &grad);
    EXPECT_GT(dist, 0.0);
    EXPECT_GT(grad.x(), 0.0);
}
