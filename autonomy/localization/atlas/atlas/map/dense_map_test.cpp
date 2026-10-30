/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "atlas/map/dense_map.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <vector>

#include <gtest/gtest.h>

namespace autonomy::localization::atlas::map {
namespace {

using PointVector =
    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>;

std::set<int> SnapshotXCoordinates(const DenseMap& map) {
    std::set<int> coordinates;
    for (const ColorPoint& point : map.Snapshot(0.0, 0).points)
        coordinates.insert(static_cast<int>(std::lround(point.position.x())));
    return coordinates;
}

PointVector PointsOnXAxis(int begin, int end) {
    PointVector points;
    for (int x = begin; x < end; ++x) {
        ColorPoint point;
        point.position =
            Eigen::Vector3f(static_cast<float>(x), 0.0F, 0.0F);
        points.push_back(point);
    }
    return points;
}

TEST(DenseMapTest, FusesPointsAndColorsWithinAVoxel) {
    DenseMapOptions options;
    options.voxel_size = 1.0;
    options.maximum_voxels = 10;
    options.reserve_voxels = 10;
    DenseMap map(options);

    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> points(2);
    points[0].position = Eigen::Vector3f(0.1F, 0.2F, 0.3F);
    points[0].red = 10;
    points[0].green = 20;
    points[0].blue = 30;
    points[1].position = Eigen::Vector3f(0.3F, 0.4F, 0.5F);
    points[1].red = 30;
    points[1].green = 40;
    points[1].blue = 50;

    EXPECT_EQ(map.Integrate(points), 2U);
    const DenseMapSnapshot snapshot = map.Snapshot(4.0, 7);
    ASSERT_EQ(snapshot.points.size(), 1U);
    EXPECT_TRUE(snapshot.points[0].position.isApprox(
        Eigen::Vector3f(0.2F, 0.3F, 0.4F), 1e-5F));
    EXPECT_EQ(snapshot.points[0].red, 20);
    EXPECT_EQ(snapshot.points[0].green, 30);
    EXPECT_EQ(snapshot.points[0].blue, 40);
    EXPECT_EQ(snapshot.generation, 7U);
}

TEST(DenseMapTest, EnforcesVoxelCapacityAndReset) {
    DenseMapOptions options;
    options.voxel_size = 0.1;
    options.maximum_voxels = 1;
    options.reserve_voxels = 1;
    DenseMap map(options);

    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> points(2);
    points[0].position = Eigen::Vector3f::Zero();
    points[1].position = Eigen::Vector3f::Ones();
    EXPECT_GE(map.Integrate(points), 1U);
    EXPECT_EQ(map.size(), 1U);
    map.Reset();
    EXPECT_EQ(map.size(), 0U);
}

TEST(DenseMapTest, CapacitySampleIsIndependentOfInsertionOrder) {
    DenseMapOptions options;
    options.voxel_size = 1.0;
    options.maximum_voxels = 11;
    options.reserve_voxels = 11;
    DenseMap forward(options);
    DenseMap reverse(options);

    PointVector points = PointsOnXAxis(-64, 64);
    PointVector reversed = points;
    std::reverse(reversed.begin(), reversed.end());
    EXPECT_GE(forward.Integrate(points), options.maximum_voxels);
    EXPECT_GE(reverse.Integrate(reversed), options.maximum_voxels);

    EXPECT_EQ(forward.size(), options.maximum_voxels);
    EXPECT_EQ(reverse.size(), options.maximum_voxels);
    EXPECT_EQ(SnapshotXCoordinates(forward), SnapshotXCoordinates(reverse));
}

TEST(DenseMapTest, BetterLateVoxelReplacesWorstRetainedVoxel) {
    DenseMapOptions options;
    options.voxel_size = 1.0;
    options.maximum_voxels = 1;
    options.reserve_voxels = 1;
    const PointVector points = PointsOnXAxis(0, 64);

    DenseMap reference(options);
    reference.Integrate(points);
    const std::set<int> reference_coordinates =
        SnapshotXCoordinates(reference);
    ASSERT_EQ(reference_coordinates.size(), 1U);
    const int selected = *reference_coordinates.begin();

    PointVector earlier;
    ColorPoint selected_point;
    bool found_selected = false;
    for (const ColorPoint& point : points) {
        if (static_cast<int>(std::lround(point.position.x())) == selected) {
            selected_point = point;
            found_selected = true;
        } else {
            earlier.push_back(point);
        }
    }
    ASSERT_TRUE(found_selected);

    DenseMap late(options);
    late.Integrate(earlier);
    EXPECT_NE(SnapshotXCoordinates(late), reference_coordinates);
    PointVector final_point;
    final_point.push_back(selected_point);
    EXPECT_EQ(late.Integrate(final_point), 1U);
    EXPECT_EQ(late.size(), 1U);
    EXPECT_EQ(SnapshotXCoordinates(late), reference_coordinates);
}

TEST(DenseMapTest, ResetClearsCapacityPriorityIndex) {
    DenseMapOptions options;
    options.voxel_size = 1.0;
    options.maximum_voxels = 1;
    options.reserve_voxels = 1;
    DenseMap map(options);
    map.Integrate(PointsOnXAxis(0, 64));
    ASSERT_EQ(map.size(), 1U);

    map.Reset();
    PointVector point = PointsOnXAxis(1000, 1001);
    ASSERT_EQ(map.Integrate(point), 1U);
    EXPECT_EQ(map.size(), 1U);
    EXPECT_EQ(SnapshotXCoordinates(map), std::set<int>({1000}));
}

TEST(DenseMapTest, RadiusRemovalRebuildsCapacityPriorityIndex) {
    DenseMapOptions options;
    options.voxel_size = 1.0;
    options.maximum_voxels = 3;
    options.reserve_voxels = 3;
    DenseMap map(options);

    PointVector initial = PointsOnXAxis(0, 2);
    PointVector far = PointsOnXAxis(100, 102);
    initial.insert(initial.end(), far.begin(), far.end());
    map.Integrate(initial);
    ASSERT_EQ(map.size(), 3U);
    EXPECT_EQ(map.RemoveOutsideRadius(Eigen::Vector3f::Zero(), 2.0), 1U);
    EXPECT_EQ(SnapshotXCoordinates(map), std::set<int>({0, 1}));

    PointVector later = PointsOnXAxis(2, 64);
    map.Integrate(later);
    DenseMap reference(options);
    reference.Integrate(PointsOnXAxis(0, 64));
    EXPECT_EQ(map.size(), options.maximum_voxels);
    EXPECT_EQ(SnapshotXCoordinates(map), SnapshotXCoordinates(reference));
}

}  // namespace
}  // namespace autonomy::localization::atlas::map
