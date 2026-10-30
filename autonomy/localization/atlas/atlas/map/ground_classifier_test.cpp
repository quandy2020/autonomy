/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "atlas/map/ground_classifier.hpp"

#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

namespace autonomy::localization::atlas::map {
namespace {

ColorPoint Point(float x, float y, float z) {
    ColorPoint point;
    point.position = Eigen::Vector3f(x, y, z);
    return point;
}

OccupancyGridOptions NavigationOptions() {
    OccupancyGridOptions options;
    options.projection_plane = OccupancyGridOptions::ProjectionPlane::kXy;
    options.up_axis = OccupancyGridOptions::UpAxis::kPositiveZ;
    options.use_sensor_height = true;
    options.sensor_height = 1.0;
    options.estimate_ground_plane = true;
    options.ground_distance_threshold = 0.10;
    options.ground_candidate_height_margin = 0.25;
    options.maximum_ground_slope_radians = 0.35;
    options.ground_sensor_height_tolerance = 0.40;
    options.maximum_ground_estimation_range = 12.0;
    options.minimum_ground_inliers = 20;
    options.maximum_ground_estimation_points = 1024;
    options.ground_ransac_iterations = 96;
    options.minimum_obstacle_height = 0.10;
    options.maximum_obstacle_height = 1.0;
    options.minimum_navigation_height = -1.0;
    options.maximum_navigation_height = 1.0;
    return options;
}

TEST(GroundClassifierTest, EstimatesTiltedGroundAndClassifiesHeightBand) {
    const OccupancyGridOptions options = NavigationOptions();
    GroundClassifier classifier(options);
    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> points;
    for (int x = -4; x <= 4; ++x) {
        for (int y = -4; y <= 4; ++y) {
            const float ground_z = 0.05F * static_cast<float>(x);
            points.push_back(Point(static_cast<float>(x),
                                   static_cast<float>(y), ground_z));
        }
    }
    // One navigable obstacle and two points outside the occupancy height band.
    points.push_back(Point(2.0F, 0.0F, 0.60F));  // 0.50 m above ground.
    points.push_back(Point(0.0F, 1.0F, 1.30F));  // Above 1.0 m.
    points.push_back(Point(0.0F, 2.0F, -0.30F));  // Below ground tolerance.

    SemanticPointCloud classified;
    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> navigation;
    GroundPlane plane;
    ASSERT_TRUE(classifier.Classify(Eigen::Vector3f(0.0F, 0.0F, 1.0F),
                                    points, &classified, &plane,
                                    &navigation));
    ASSERT_TRUE(plane.valid);
    EXPECT_TRUE(plane.estimated);
    EXPECT_GT(plane.normal.dot(Eigen::Vector3d::UnitZ()), 0.99);

    std::size_t ground = 0;
    std::size_t obstacle = 0;
    std::size_t unknown = 0;
    for (const SemanticPoint& point : classified) {
        switch (point.semantic) {
            case PointSemantic::kGround:
                ++ground;
                break;
            case PointSemantic::kObstacle:
                ++obstacle;
                break;
            case PointSemantic::kUnknown:
                ++unknown;
                break;
        }
    }
    EXPECT_GE(ground, 80U);
    EXPECT_EQ(obstacle, 1U);
    EXPECT_EQ(unknown, 2U);
    // The [-1, 1] m navigation cloud retains the below-ground sample but not
    // the sample 1.3 m above the plane. DenseMap still receives all 84 points.
    EXPECT_EQ(navigation.size(), 83U);
}

TEST(GroundClassifierTest, FallsBackToSensorHeightPlane) {
    OccupancyGridOptions options = NavigationOptions();
    options.up_axis = OccupancyGridOptions::UpAxis::kNegativeY;
    options.projection_plane = OccupancyGridOptions::ProjectionPlane::kXz;
    options.minimum_ground_inliers = 20;
    GroundClassifier classifier(options);

    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> points;
    points.push_back(Point(2.0F, 1.0F, 0.0F));   // Ground, one metre down.
    points.push_back(Point(3.0F, 0.5F, 0.0F));   // 0.5 m obstacle.
    points.push_back(Point(4.0F, -0.5F, 0.0F));  // 1.5 m, unknown.
    points.push_back(Point(5.0F, 1.3F, 0.0F));   // Below ground, unknown.

    SemanticPointCloud classified;
    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> navigation;
    GroundPlane plane;
    ASSERT_TRUE(classifier.Classify(Eigen::Vector3f::Zero(), points,
                                    &classified, &plane, &navigation));
    ASSERT_EQ(classified.size(), 4U);
    EXPECT_TRUE(plane.valid);
    EXPECT_FALSE(plane.estimated);
    EXPECT_EQ(classified[0].semantic, PointSemantic::kGround);
    EXPECT_EQ(classified[1].semantic, PointSemantic::kObstacle);
    EXPECT_EQ(classified[2].semantic, PointSemantic::kUnknown);
    EXPECT_EQ(classified[3].semantic, PointSemantic::kUnknown);
    EXPECT_EQ(navigation.size(), 3U);
}

TEST(GroundClassifierTest,
     UsesConservativeRelativeHeightWithoutSensorHeight) {
    OccupancyGridOptions options = NavigationOptions();
    options.use_sensor_height = false;
    options.estimate_ground_plane = false;
    options.minimum_obstacle_height = -0.5;
    options.maximum_obstacle_height = 2.0;
    GroundClassifier classifier(options);

    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> points;
    points.push_back(Point(0.0F, 0.0F, 0.5F));
    points.push_back(Point(0.0F, 0.0F, 3.0F));
    points.push_back(Point(0.0F, 0.0F, -0.75F));
    points.push_back(Point(0.0F, 0.0F, -2.0F));
    SemanticPointCloud classified;
    std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>> navigation;
    ASSERT_TRUE(classifier.Classify(Eigen::Vector3f::Zero(), points,
                                    &classified, nullptr, &navigation));
    ASSERT_EQ(classified.size(), 4U);
    EXPECT_EQ(classified[0].semantic, PointSemantic::kObstacle);
    EXPECT_EQ(classified[1].semantic, PointSemantic::kUnknown);
    EXPECT_EQ(classified[2].semantic, PointSemantic::kGround);
    EXPECT_EQ(classified[3].semantic, PointSemantic::kGround);
    ASSERT_EQ(navigation.size(), 2U);
    EXPECT_TRUE(navigation[0].position.isApprox(points[0].position));
    EXPECT_TRUE(navigation[1].position.isApprox(points[2].position));
}

TEST(GroundClassifierTest, RejectsInvalidHeightBands) {
    OccupancyGridOptions options = NavigationOptions();
    options.minimum_obstacle_height = 2.0;
    options.maximum_obstacle_height = 1.0;
    EXPECT_THROW({ GroundClassifier classifier(options); },
                 std::invalid_argument);

    options = NavigationOptions();
    options.minimum_navigation_height = 2.0;
    options.maximum_navigation_height = 1.0;
    EXPECT_THROW({ GroundClassifier classifier(options); },
                 std::invalid_argument);

}

}  // namespace
}  // namespace autonomy::localization::atlas::map
