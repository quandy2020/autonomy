/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "atlas/map/occupancy_grid_map.hpp"

#include <limits>

#include <gtest/gtest.h>

namespace autonomy::localization::atlas::map {
namespace {

TEST(OccupancyGridMapTest, ClearsRayAndMarksEndpointOccupied) {
    OccupancyGridOptions options;
    options.resolution = 1.0;
    options.minimum_obstacle_height = -1.0;
    options.maximum_obstacle_height = 1.0;
    options.maximum_extent = 10.0;
    options.maximum_cells = 100;
    options.maximum_snapshot_cells = 100;
    OccupancyGridMap map(options);

    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> points(1);
    points[0].position = Eigen::Vector3f(3.1F, 0.1F, 0.0F);
    EXPECT_EQ(map.Integrate(Eigen::Vector3f::Zero(), points), 4U);

    double free_probability = 0.0;
    double hit_probability = 0.0;
    ASSERT_TRUE(map.ProbabilityAt(1.1, 0.1, &free_probability));
    ASSERT_TRUE(map.ProbabilityAt(3.1, 0.1, &hit_probability));
    EXPECT_NEAR(free_probability, options.miss_probability, 1e-5);
    EXPECT_NEAR(hit_probability, options.hit_probability, 1e-5);

    const OccupancyGridSnapshot snapshot = map.Snapshot(1.0, 2);
    EXPECT_EQ(snapshot.width, 4U);
    EXPECT_EQ(snapshot.height, 1U);
    ASSERT_EQ(snapshot.data.size(), 4U);
    EXPECT_EQ(snapshot.data[0], 0);
    EXPECT_EQ(snapshot.data[1], 0);
    EXPECT_EQ(snapshot.data[2], 0);
    EXPECT_EQ(snapshot.data[3], 100);
}

TEST(OccupancyGridMapTest, OccupiedCellWinsOverCrossingFreeRay) {
    OccupancyGridOptions options;
    options.resolution = 1.0;
    options.minimum_obstacle_height = -1.0;
    options.maximum_obstacle_height = 1.0;
    options.maximum_extent = 10.0;
    options.maximum_cells = 100;
    options.maximum_snapshot_cells = 100;
    OccupancyGridMap map(options);

    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> points(2);
    points[0].position = Eigen::Vector3f(2.1F, 0.1F, 0.0F);
    points[1].position = Eigen::Vector3f(4.1F, 0.1F, -2.0F);
    map.Integrate(Eigen::Vector3f::Zero(), points);
    double probability = 0.0;
    ASSERT_TRUE(map.ProbabilityAt(2.1, 0.1, &probability));
    EXPECT_NEAR(probability, options.hit_probability, 1e-5);
}

TEST(OccupancyGridMapTest, ProjectsXzPlaneWithNegativeYUp) {
    OccupancyGridOptions options;
    options.projection_plane =
        OccupancyGridOptions::ProjectionPlane::kXz;
    options.up_axis = OccupancyGridOptions::UpAxis::kNegativeY;
    options.resolution = 1.0;
    options.minimum_obstacle_height = -1.0;
    options.maximum_obstacle_height = 1.0;
    options.maximum_extent = 10.0;
    options.maximum_cells = 100;
    options.maximum_snapshot_cells = 100;
    OccupancyGridMap map(options);

    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> points(1);
    points[0].position = Eigen::Vector3f(2.1F, 0.0F, 3.1F);
    EXPECT_GT(map.Integrate(Eigen::Vector3f::Zero(), points), 0U);
    double probability = 0.0;
    ASSERT_TRUE(map.ProbabilityAt(2.1, 3.1, &probability));
    EXPECT_NEAR(probability, options.hit_probability, 1e-5);

    const OccupancyGridSnapshot snapshot = map.Snapshot(1.0, 1);
    EXPECT_TRUE(snapshot.origin_translation.isApprox(
        Eigen::Vector3d(snapshot.origin_x, 0.0, snapshot.origin_y)));
    EXPECT_TRUE((snapshot.origin_rotation * Eigen::Vector3d::UnitX())
                    .isApprox(Eigen::Vector3d::UnitX()));
    EXPECT_TRUE((snapshot.origin_rotation * Eigen::Vector3d::UnitY())
                    .isApprox(Eigen::Vector3d::UnitZ()));
    EXPECT_TRUE((snapshot.origin_rotation * Eigen::Vector3d::UnitZ())
                    .isApprox(-Eigen::Vector3d::UnitY()));
}

TEST(OccupancyGridMapTest, SensorHeightKeepsGroundFreeAndObstacleOccupied) {
    OccupancyGridOptions options;
    options.projection_plane =
        OccupancyGridOptions::ProjectionPlane::kXz;
    options.up_axis = OccupancyGridOptions::UpAxis::kNegativeY;
    options.resolution = 1.0;
    options.sensor_height = 1.0;
    options.use_sensor_height = true;
    options.minimum_obstacle_height = 0.1;
    options.maximum_obstacle_height = 1.0;
    options.maximum_extent = 10.0;
    options.maximum_cells = 100;
    options.maximum_snapshot_cells = 100;
    OccupancyGridMap map(options);

    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> points(2);
    // Negative-Y is up. y=1 is one metre below the camera: ground height 0.
    points[0].position = Eigen::Vector3f(2.1F, 1.0F, 0.1F);
    // y=0.5 is 0.5 m above the inferred ground plane: obstacle.
    points[1].position = Eigen::Vector3f(3.1F, 0.5F, 1.1F);
    EXPECT_GT(map.Integrate(Eigen::Vector3f::Zero(), points), 0U);

    double ground_probability = 0.0;
    double obstacle_probability = 0.0;
    ASSERT_TRUE(map.ProbabilityAt(2.1, 0.1, &ground_probability));
    ASSERT_TRUE(map.ProbabilityAt(3.1, 1.1, &obstacle_probability));
    EXPECT_NEAR(ground_probability, options.miss_probability, 1e-5);
    EXPECT_NEAR(obstacle_probability, options.hit_probability, 1e-5);
}

TEST(OccupancyGridMapTest, ExplicitUnknownDoesNotModifyMap) {
    OccupancyGridOptions options;
    options.resolution = 1.0;
    options.maximum_extent = 10.0;
    options.maximum_cells = 100;
    options.maximum_snapshot_cells = 100;
    OccupancyGridMap map(options);

    SemanticPointCloud points(3);
    points[0].position = Eigen::Vector3f(2.1F, 0.1F, 0.0F);
    points[0].semantic = PointSemantic::kGround;
    points[1].position = Eigen::Vector3f(3.1F, 1.1F, 0.0F);
    points[1].semantic = PointSemantic::kObstacle;
    points[2].position = Eigen::Vector3f(2.1F, 3.1F, 0.0F);
    points[2].semantic = PointSemantic::kUnknown;
    EXPECT_GT(map.Integrate(Eigen::Vector3f::Zero(), points), 0U);

    double probability = 0.0;
    ASSERT_TRUE(map.ProbabilityAt(2.1, 0.1, &probability));
    EXPECT_NEAR(probability, options.miss_probability, 1e-5);
    ASSERT_TRUE(map.ProbabilityAt(3.1, 1.1, &probability));
    EXPECT_NEAR(probability, options.hit_probability, 1e-5);
    EXPECT_FALSE(map.ProbabilityAt(2.1, 3.1, &probability));
}

TEST(OccupancyGridMapTest, SemanticGroundRayStopsAtNearestObstacle) {
    OccupancyGridOptions options;
    options.resolution = 1.0;
    options.maximum_extent = 10.0;
    options.maximum_cells = 100;
    options.maximum_snapshot_cells = 100;
    options.ray_angle_bins = 360;
    OccupancyGridMap map(options);

    SemanticPointCloud points(2);
    points[0].position = Eigen::Vector3f(2.1F, 0.0F, 0.0F);
    points[0].semantic = PointSemantic::kObstacle;
    points[1].position = Eigen::Vector3f(5.1F, 0.0F, 0.0F);
    points[1].semantic = PointSemantic::kGround;
    map.Integrate(Eigen::Vector3f::Zero(), points);

    double probability = 0.0;
    ASSERT_TRUE(map.ProbabilityAt(1.1, 0.0, &probability));
    EXPECT_NEAR(probability, options.miss_probability, 1e-5);
    ASSERT_TRUE(map.ProbabilityAt(2.1, 0.0, &probability));
    EXPECT_NEAR(probability, options.hit_probability, 1e-5);
    EXPECT_FALSE(map.ProbabilityAt(3.1, 0.0, &probability));
    EXPECT_FALSE(map.ProbabilityAt(5.1, 0.0, &probability));
}

TEST(OccupancyGridMapTest, SemanticFarObstacleKeepsOccludedCellsUnknown) {
    OccupancyGridOptions options;
    options.resolution = 1.0;
    options.maximum_extent = 10.0;
    options.maximum_cells = 100;
    options.maximum_snapshot_cells = 100;
    options.ray_angle_bins = 360;
    OccupancyGridMap map(options);

    SemanticPointCloud points(2);
    points[0].position = Eigen::Vector3f(2.1F, 0.0F, 0.0F);
    points[0].semantic = PointSemantic::kObstacle;
    points[1].position = Eigen::Vector3f(5.1F, 0.0F, 0.0F);
    points[1].semantic = PointSemantic::kObstacle;
    map.Integrate(Eigen::Vector3f::Zero(), points);

    double probability = 0.0;
    ASSERT_TRUE(map.ProbabilityAt(2.1, 0.0, &probability));
    EXPECT_NEAR(probability, options.hit_probability, 1e-5);
    EXPECT_FALSE(map.ProbabilityAt(3.1, 0.0, &probability));
    ASSERT_TRUE(map.ProbabilityAt(5.1, 0.0, &probability));
    EXPECT_NEAR(probability, options.hit_probability, 1e-5);
}

TEST(OccupancyGridMapTest, SemanticRayTraversalRespectsConfiguredBudget) {
    OccupancyGridOptions options;
    options.resolution = 1.0;
    options.maximum_extent = 10.0;
    options.maximum_rays_per_frame = 1;
    options.ray_angle_bins = 1;
    options.maximum_cells = 100;
    options.maximum_snapshot_cells = 100;
    OccupancyGridMap map(options);

    SemanticPointCloud points(2);
    points[0].position = Eigen::Vector3f(3.1F, 0.0F, 0.0F);
    points[0].semantic = PointSemantic::kObstacle;
    points[1].position = Eigen::Vector3f(0.0F, 3.1F, 0.0F);
    points[1].semantic = PointSemantic::kGround;
    map.Integrate(Eigen::Vector3f::Zero(), points);

    double probability = 0.0;
    ASSERT_TRUE(map.ProbabilityAt(3.1, 0.0, &probability));
    EXPECT_NEAR(probability, options.hit_probability, 1e-5);
    EXPECT_FALSE(map.ProbabilityAt(0.0, 1.1, &probability));
}

TEST(OccupancyGridMapTest, ClampsRepeatedHitAndMissUpdates) {
    OccupancyGridOptions options;
    options.resolution = 1.0;
    options.minimum_obstacle_height = -1.0;
    options.maximum_obstacle_height = 1.0;
    options.maximum_extent = 10.0;
    options.maximum_cells = 100;
    options.maximum_snapshot_cells = 100;
    OccupancyGridMap map(options);

    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> hit(1);
    hit[0].position = Eigen::Vector3f(3.1F, 0.1F, 0.0F);
    for (int iteration = 0; iteration < 100; ++iteration)
        map.Integrate(Eigen::Vector3f::Zero(), hit);
    double probability = 0.0;
    ASSERT_TRUE(map.ProbabilityAt(3.1, 0.1, &probability));
    EXPECT_NEAR(probability, options.maximum_probability, 1e-5);

    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> miss(1);
    miss[0].position = Eigen::Vector3f(3.1F, 0.1F, -2.0F);
    for (int iteration = 0; iteration < 100; ++iteration)
        map.Integrate(Eigen::Vector3f::Zero(), miss);
    ASSERT_TRUE(map.ProbabilityAt(1.1, 0.1, &probability));
    EXPECT_NEAR(probability, options.minimum_probability, 1e-5);
}

TEST(OccupancyGridMapTest, ExpandsAcrossNegativeCoordinates) {
    OccupancyGridOptions options;
    options.resolution = 1.0;
    options.minimum_obstacle_height = -1.0;
    options.maximum_obstacle_height = 1.0;
    options.maximum_extent = 10.0;
    options.maximum_cells = 100;
    options.maximum_snapshot_cells = 100;
    OccupancyGridMap map(options);

    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> points(2);
    points[0].position = Eigen::Vector3f(-2.1F, 0.1F, 0.0F);
    points[1].position = Eigen::Vector3f(2.1F, 0.1F, 0.0F);
    map.Integrate(Eigen::Vector3f::Zero(), points);
    const OccupancyGridSnapshot snapshot = map.Snapshot(1.0, 1);
    EXPECT_DOUBLE_EQ(snapshot.origin_x, -3.0);
    EXPECT_EQ(snapshot.width, 6U);
    EXPECT_EQ(snapshot.height, 1U);
    EXPECT_EQ(snapshot.data.size(), 6U);
    EXPECT_EQ(snapshot.data.front(), 100);
    EXPECT_EQ(snapshot.data.back(), 100);
}

TEST(OccupancyGridMapTest, DownsamplesOversizedContiguousSnapshot) {
    OccupancyGridOptions options;
    options.resolution = 1.0;
    options.minimum_obstacle_height = -1.0;
    options.maximum_obstacle_height = 1.0;
    options.maximum_extent = 10.0;
    options.maximum_cells = 100;
    options.maximum_snapshot_cells = 4;
    OccupancyGridMap map(options);

    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> points(2);
    points[0].position = Eigen::Vector3f(-2.1F, 0.1F, 0.0F);
    points[1].position = Eigen::Vector3f(2.1F, 0.1F, 0.0F);
    map.Integrate(Eigen::Vector3f::Zero(), points);
    const OccupancyGridSnapshot snapshot = map.Snapshot(1.0, 1);
    EXPECT_DOUBLE_EQ(snapshot.resolution, 2.0);
    EXPECT_EQ(snapshot.width, 3U);
    EXPECT_EQ(snapshot.height, 1U);
    ASSERT_EQ(snapshot.data.size(), 3U);
    EXPECT_EQ(snapshot.data.front(), 100);
    EXPECT_EQ(snapshot.data.back(), 100);
}

TEST(OccupancyGridMapTest,
     RelativeHeightFallbackDoesNotClearSpaceForHighPoints) {
    OccupancyGridOptions options;
    options.resolution = 1.0;
    options.minimum_obstacle_height = -0.5;
    options.maximum_obstacle_height = 1.0;
    options.maximum_extent = 10.0;
    options.maximum_cells = 100;
    options.maximum_snapshot_cells = 100;
    OccupancyGridMap map(options);

    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> points(2);
    points[0].position = Eigen::Vector3f(0.1F, 3.1F, 2.0F);
    points[1].position = Eigen::Vector3f(3.1F, 0.1F, -1.0F);
    EXPECT_GT(map.Integrate(Eigen::Vector3f::Zero(), points), 0U);

    double probability = 0.0;
    EXPECT_FALSE(map.ProbabilityAt(0.1, 3.1, &probability));
    ASSERT_TRUE(map.ProbabilityAt(3.1, 0.1, &probability));
    EXPECT_NEAR(probability, options.miss_probability, 1e-5);
}

TEST(OccupancyGridMapTest, MaximumExtentIsRelativeToSensorOrigin) {
    OccupancyGridOptions options;
    options.resolution = 1.0;
    options.minimum_obstacle_height = -1.0;
    options.maximum_obstacle_height = 1.0;
    options.maximum_extent = 10.0;
    options.maximum_cells = 100;
    options.maximum_snapshot_cells = 100;
    OccupancyGridMap map(options);

    const Eigen::Vector3f origin(100.1F, 0.1F, 0.0F);
    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> nearby(1);
    nearby[0].position = Eigen::Vector3f(105.1F, 0.1F, 0.0F);
    EXPECT_EQ(map.Integrate(origin, nearby), 6U);
    double probability = 0.0;
    ASSERT_TRUE(map.ProbabilityAt(105.1, 0.1, &probability));
    EXPECT_NEAR(probability, options.hit_probability, 1e-5);

    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> distant(1);
    distant[0].position = Eigen::Vector3f(111.1F, 0.1F, 0.0F);
    EXPECT_EQ(map.Integrate(origin, distant), 0U);
    EXPECT_FALSE(map.ProbabilityAt(111.1, 0.1, &probability));
}

TEST(OccupancyGridMapTest,
     InflatesKnownFreeCellsAtBoundaryWithoutChangingRawOddsOrUnknown) {
    OccupancyGridOptions options;
    options.resolution = 1.0;
    options.maximum_extent = 10.0;
    options.maximum_cells = 100;
    options.maximum_snapshot_cells = 100;
    options.robot_radius = 1.0;
    options.inflation_radius = 3.0;
    options.inflation_cost_scaling = 1.0;
    OccupancyGridMap map(options);

    SemanticPointCloud points(2);
    points[0].position = Eigen::Vector3f(0.1F, 0.1F, 0.0F);
    points[0].semantic = PointSemantic::kObstacle;
    points[1].position = Eigen::Vector3f(-4.1F, 2.1F, 0.0F);
    points[1].semantic = PointSemantic::kGround;
    ASSERT_GT(map.Integrate(Eigen::Vector3f(-4.1F, 0.1F, 0.0F), points),
              0U);

    double probability = 0.0;
    ASSERT_TRUE(map.ProbabilityAt(-0.9, 0.1, &probability));
    EXPECT_NEAR(probability, options.miss_probability, 1e-5);

    const OccupancyGridSnapshot snapshot = map.Snapshot(1.0, 1);
    ASSERT_EQ(snapshot.width, 6U);
    ASSERT_EQ(snapshot.height, 3U);
    ASSERT_EQ(snapshot.data.size(), 18U);
    EXPECT_EQ(snapshot.data[5], 100);
    EXPECT_EQ(snapshot.data[4], 99);
    EXPECT_EQ(snapshot.data[3], 36);
    EXPECT_EQ(snapshot.data[2], 13);
    EXPECT_EQ(snapshot.data[1], 0);
    // The cell directly above the inflated band was never observed.
    EXPECT_EQ(snapshot.data[10], -1);

    ASSERT_TRUE(map.ProbabilityAt(-0.9, 0.1, &probability));
    EXPECT_NEAR(probability, options.miss_probability, 1e-5);
}

TEST(OccupancyGridMapTest, InflationUsesDownsampledSnapshotResolution) {
    const auto make_snapshot = [](double robot_radius) {
        OccupancyGridOptions options;
        options.resolution = 1.0;
        options.maximum_extent = 10.0;
        options.maximum_cells = 100;
        options.maximum_snapshot_cells = 16;
        options.robot_radius = robot_radius;
        OccupancyGridMap map(options);

        SemanticPointCloud points(2);
        points[0].position = Eigen::Vector3f(0.1F, 0.1F, 0.0F);
        points[0].semantic = PointSemantic::kObstacle;
        points[1].position = Eigen::Vector3f(-4.1F, 4.1F, 0.0F);
        points[1].semantic = PointSemantic::kGround;
        EXPECT_GT(map.Integrate(Eigen::Vector3f(-4.1F, 0.1F, 0.0F), points),
                  0U);
        return map.Snapshot(1.0, 1);
    };

    const OccupancyGridSnapshot narrow = make_snapshot(1.1);
    ASSERT_DOUBLE_EQ(narrow.resolution, 2.0);
    ASSERT_EQ(narrow.width, 3U);
    ASSERT_EQ(narrow.height, 3U);
    ASSERT_EQ(narrow.data.size(), 9U);
    EXPECT_EQ(narrow.data[2], 100);
    EXPECT_EQ(narrow.data[1], 0);

    const OccupancyGridSnapshot wide = make_snapshot(2.1);
    ASSERT_DOUBLE_EQ(wide.resolution, 2.0);
    ASSERT_EQ(wide.data.size(), 9U);
    EXPECT_EQ(wide.data[2], 100);
    EXPECT_EQ(wide.data[1], 99);
}

TEST(OccupancyGridMapTest, ValidatesInflationOptions) {
    OccupancyGridOptions options;
    options.resolution = 1.0;

    options.robot_radius = -0.1;
    EXPECT_THROW({ OccupancyGridMap{options}; }, std::invalid_argument);

    options = OccupancyGridOptions{};
    options.resolution = 1.0;
    options.inflation_radius = 1.0;
    EXPECT_THROW({ OccupancyGridMap{options}; }, std::invalid_argument);

    options.inflation_cost_scaling = 1.0;
    options.robot_radius = 2.0;
    EXPECT_THROW({ OccupancyGridMap{options}; }, std::invalid_argument);

    options = OccupancyGridOptions{};
    options.resolution = 1.0;
    options.inflation_cost_scaling = 1.0;
    EXPECT_THROW({ OccupancyGridMap{options}; }, std::invalid_argument);

    options = OccupancyGridOptions{};
    options.resolution = 1.0;
    options.robot_radius = 513.0;
    EXPECT_NO_THROW({
        OccupancyGridMap large_radius_map(options);
        SemanticPointCloud points(1);
        points[0].position = Eigen::Vector3f::Zero();
        points[0].semantic = PointSemantic::kObstacle;
        large_radius_map.Integrate(Eigen::Vector3f::Zero(), points);
        static_cast<void>(large_radius_map.Snapshot(1.0, 1));
    });

    options = OccupancyGridOptions{};
    options.resolution = 1.0;
    options.inflation_cost_scaling =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW({ OccupancyGridMap{options}; }, std::invalid_argument);

    options = OccupancyGridOptions{};
    options.resolution = 1.0;
    options.robot_radius = 1.0;
    options.inflation_radius = 3.0;
    options.inflation_cost_scaling = 1.0;
    EXPECT_NO_THROW({ OccupancyGridMap{options}; });
}

TEST(OccupancyGridMapTest, RejectsInvalidHeightBands) {
    OccupancyGridOptions options;
    options.minimum_obstacle_height = 2.0;
    options.maximum_obstacle_height = 1.0;
    EXPECT_THROW({ OccupancyGridMap{options}; }, std::invalid_argument);

    options = OccupancyGridOptions{};
    options.minimum_navigation_height = 2.0;
    options.maximum_navigation_height = 1.0;
    EXPECT_THROW({ OccupancyGridMap{options}; }, std::invalid_argument);
}

}  // namespace
}  // namespace autonomy::localization::atlas::map
