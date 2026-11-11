/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file esdf_map_test.cpp
 * @brief Occupancy fusion, distance, ray, delete, and gradient tests.
 */

#include "autonomy/map/esdf/esdf_map.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

namespace {

using autonomy::map::esdf::EsdfMap;
using autonomy::map::esdf::Options;
using autonomy::map::esdf::kInfinity;
using autonomy::map::esdf::kUndefined;

Options TinyMap() {
    Options options;
    options.origin = Eigen::Vector3d(-1.0, -1.0, -0.5);
    options.map_size = Eigen::Vector3d(2.0, 2.0, 1.0);
    options.resolution = 0.2;
    options.min_ray_length = 0.0;
    options.max_ray_length = 10.0;
    return options;
}

Options MediumMap() {
    Options options;
    options.origin = Eigen::Vector3d(-2.0, -2.0, -0.4);
    options.map_size = Eigen::Vector3d(4.0, 4.0, 0.8);
    options.resolution = 0.1;
    options.min_ray_length = 0.0;
    options.max_ray_length = 20.0;
    return options;
}

void Hit(EsdfMap* map, const Eigen::Vector3d& position, int times = 3) {
    for (int i = 0; i < times; ++i) {
        map->SetOccupancy(position, true);
        map->UpdateOccupancy(true);
    }
}

void Miss(EsdfMap* map, const Eigen::Vector3d& position, int times = 6) {
    for (int i = 0; i < times; ++i) {
        map->SetOccupancy(position, false);
        map->UpdateOccupancy(true);
    }
}

void AddWallX(EsdfMap* map, double x, double y_min, double y_max, double z,
              double resolution) {
    for (double y = y_min; y <= y_max + 0.5 * resolution; y += resolution) {
        Hit(map, Eigen::Vector3d(x, y, z));
    }
}

}  // namespace

TEST(EsdfMap, EmptyMapDistanceIsInfinity) {
    EsdfMap map(TinyMap());
    EXPECT_FALSE(map.CheckUpdate());
    EXPECT_NEAR(map.GetDistance(Eigen::Vector3d(0.0, 0.0, 0.0)),
                static_cast<double>(kInfinity), 1e-9);
    EXPECT_EQ(map.GetOccupancy(Eigen::Vector3d(0.0, 0.0, 0.0)), 0);
    EXPECT_TRUE(map.OccupiedPoints().empty());
}

TEST(EsdfMap, OutOfMapQueries) {
    EsdfMap map(TinyMap());
    EXPECT_EQ(map.GetOccupancy(Eigen::Vector3d(10.0, 0.0, 0.0)), kUndefined);
    EXPECT_NEAR(map.GetDistance(Eigen::Vector3d(10.0, 0.0, 0.0)),
                static_cast<double>(kUndefined), 1e-9);
    Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
    EXPECT_NEAR(map.GetDistanceWithGradient(Eigen::Vector3d(10.0, 0.0, 0.0),
                                            &gradient),
                -1.0, 1e-9);
}

TEST(EsdfMap, OneHitDoesNotOccupy) {
    EsdfMap map(TinyMap());
    map.SetOccupancy(Eigen::Vector3d(0.0, 0.0, 0.0), true);
    map.UpdateOccupancy(true);
    EXPECT_EQ(map.GetOccupancy(Eigen::Vector3d(0.0, 0.0, 0.0)), 0);
}

TEST(EsdfMap, TwoHitsOccupy) {
    EsdfMap map(TinyMap());
    Hit(&map, Eigen::Vector3d(0.0, 0.0, 0.0), 2);
    EXPECT_EQ(map.GetOccupancy(Eigen::Vector3d(0.0, 0.0, 0.0)), 1);
}

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

TEST(EsdfMap, WallDistanceMatchesAbsX) {
    EsdfMap map(MediumMap());
    AddWallX(&map, 0.0, -1.2, 1.2, 0.0, map.options().resolution);
    map.UpdateESDF();
    EXPECT_EQ(map.GetOccupancy(Eigen::Vector3d(0.0, 0.0, 0.0)), 1);
    EXPECT_NEAR(map.GetDistance(Eigen::Vector3d(-0.5, 0.0, 0.0)), 0.5, 0.12);
    EXPECT_NEAR(map.GetDistance(Eigen::Vector3d(0.8, 0.0, 0.0)), 0.8, 0.12);
    EXPECT_TRUE(map.CheckConsistency());
}

TEST(EsdfMap, CorridorUsesNearestWall) {
    EsdfMap map(MediumMap());
    AddWallX(&map, -0.8, -1.5, 1.5, 0.0, map.options().resolution);
    AddWallX(&map, 0.8, -1.5, 1.5, 0.0, map.options().resolution);
    map.UpdateESDF();
    EXPECT_NEAR(map.GetDistance(Eigen::Vector3d(0.0, 0.0, 0.0)), 0.8, 0.15);
    EXPECT_LT(map.GetDistance(Eigen::Vector3d(-0.5, 0.0, 0.0)),
              map.GetDistance(Eigen::Vector3d(0.0, 0.0, 0.0)));
}

TEST(EsdfMap, DeleteObstacleClearsOccupancyAndGrowsDistance) {
    EsdfMap map(TinyMap());
    Hit(&map, Eigen::Vector3d(0.0, 0.0, 0.0), 3);
    map.UpdateESDF();
    ASSERT_EQ(map.GetOccupancy(Eigen::Vector3d(0.0, 0.0, 0.0)), 1);
    const double near_before = map.GetDistance(Eigen::Vector3d(0.31, 0.0, 0.0));
    Miss(&map, Eigen::Vector3d(0.0, 0.0, 0.0), 8);
    map.UpdateESDF();
    EXPECT_EQ(map.GetOccupancy(Eigen::Vector3d(0.0, 0.0, 0.0)), 0);
    EXPECT_GT(map.GetDistance(Eigen::Vector3d(0.31, 0.0, 0.0)), near_before);
    EXPECT_TRUE(map.CheckConsistency());
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
    EXPECT_EQ(map.GetOccupancy(Eigen::Vector3d(-0.3, 0.0, 0.0)), 0);
    EXPECT_FALSE(map.OccupiedPoints().empty());
}

TEST(EsdfMap, IntegrateCloudSkipsNan) {
    EsdfMap map(TinyMap());
    const std::vector<Eigen::Vector3d> cloud = {
        Eigen::Vector3d(NAN, 0.0, 0.0),
        Eigen::Vector3d(0.5, 0.0, 0.0),
    };
    for (int i = 0; i < 3; ++i) {
        map.IntegrateCloud(Eigen::Vector3d(-0.9, 0.0, 0.0), cloud);
        map.UpdateOccupancy(true);
    }
    map.UpdateESDF();
    EXPECT_EQ(map.GetOccupancy(Eigen::Vector3d(0.5, 0.0, 0.0)), 1);
}

TEST(EsdfMap, GradientPointsAwayFromObstacle) {
    EsdfMap map(TinyMap());
    Hit(&map, Eigen::Vector3d(0.0, 0.0, 0.0), 4);
    map.UpdateESDF();
    Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
    const double distance =
        map.GetDistanceWithGradient(Eigen::Vector3d(0.4, 0.0, 0.0), &gradient);
    EXPECT_GT(distance, 0.0);
    EXPECT_GT(gradient.x(), 0.0);
}

TEST(EsdfMap, GradientOnWallPointsAlongX) {
    EsdfMap map(MediumMap());
    AddWallX(&map, 0.0, -1.0, 1.0, 0.0, map.options().resolution);
    map.UpdateESDF();
    Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
    map.GetDistanceWithGradient(Eigen::Vector3d(0.45, 0.0, 0.0), &gradient);
    EXPECT_GT(gradient.x(), 0.0);
    EXPECT_LT(std::abs(gradient.y()), std::abs(gradient.x()));
}
