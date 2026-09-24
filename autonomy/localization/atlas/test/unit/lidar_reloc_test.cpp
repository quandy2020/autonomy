/*
 * Copyright 2026 The Openbot Authors
 *
 * Yaw-grid relocalization followed by Ceres point-to-plane.
 */

#include "autonomy/localization/atlas/frontend/lidar/lidar_odometry.hpp"
#include "autonomy/localization/atlas/io/point_cloud_io.hpp"

#include <cmath>

#include "Eigen/Geometry"
#include "gtest/gtest.h"

namespace autonomy {
namespace localization {
namespace atlas {
namespace {

PointCloud MakeWall() {
    PointCloud cloud;
    for (int y = -4; y <= 4; ++y) {
        for (int z = 0; z <= 4; ++z) {
            PointXYZI point;
            point.x = 3.f;
            point.y = 0.25f * static_cast<float>(y);
            point.z = 0.25f * static_cast<float>(z);
            cloud.push_back(point);
        }
    }
    return cloud;
}

TEST(LidarRelocTest, YawSearchThenCeres) {
    AtlasConfig config;
    config.mode = FrontendMode::kLio;
    config.mission = Mission::kRelocalization;
    config.voxel_size = 0.3;
    config.ceres_pose_weight = 1.0;
    config.ceres_max_iterations = 15;
    config.max_local_map_points = 100000;
    config.reloc_xy_radius = 2.0;
    config.reloc_xy_step = 2.0;

    LidarOdometry lidar;
    ASSERT_TRUE(lidar.Init(config));
    const PointCloud wall = MakeWall();
    lidar.LoadMap(wall);

    const double yaw = 0.35;
    const Mat33 rotation =
        Eigen::AngleAxisd(yaw, Vec3::UnitZ()).toRotationMatrix();
    PointCloud body;
    for (const auto& point : wall) {
        const Vec3 world(point.x, point.y, point.z);
        const Vec3 local = rotation.transpose() * world;
        PointXYZI sample;
        sample.x = static_cast<float>(local.x());
        sample.y = static_cast<float>(local.y());
        sample.z = static_cast<float>(local.z());
        body.push_back(sample);
    }
    SensorData data;
    EncodePointCloud2(body, &data.lidar, "lidar");
    data.has_lidar = true;

    ASSERT_TRUE(lidar.Process(data));
    OdometryResult result;
    ASSERT_TRUE(lidar.GetResult(&result));
    const double estimated =
        std::atan2(result.pose_world_body.rotation()(1, 0),
                   result.pose_world_body.rotation()(0, 0));
    EXPECT_NEAR(estimated, yaw, 0.2);

    PointCloud2 dense;
    EXPECT_TRUE(lidar.FillDenseCloud(&dense));
    EXPECT_GT(dense.width(), 0u);

    const std::string directory = "/tmp/atlas_lidar_tiles";
    ASSERT_TRUE(lidar.SaveMap(directory));
    LidarOdometry loaded;
    ASSERT_TRUE(loaded.Init(config));
    ASSERT_TRUE(loaded.LoadMapDirectory(directory));
    ASSERT_TRUE(loaded.Process(data));
    OdometryResult loaded_result;
    ASSERT_TRUE(loaded.GetResult(&loaded_result));
    const double loaded_yaw =
        std::atan2(loaded_result.pose_world_body.rotation()(1, 0),
                   loaded_result.pose_world_body.rotation()(0, 0));
    EXPECT_NEAR(loaded_yaw, yaw, 0.2);
}

}  // namespace
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
