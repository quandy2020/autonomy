/*
 * Copyright 2026 Autodriver contributors
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

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <memory>
#include <string>

#include <automsgs/msgs/nav_msgs/odometry.pb.h>

#include "autodriver/bridge/pose_feeder.hpp"
#include "autodriver/config.hpp"
#include "autodriver/sensor_manager.hpp"

namespace {

std::shared_ptr<automsgs::msgs::nav_msgs::Odometry> MakeOdom(
    double x, double y, double z, std::int32_t sec, std::uint32_t nanosec) {
    auto msg = std::make_shared<automsgs::msgs::nav_msgs::Odometry>();
    msg->mutable_header()->mutable_stamp()->set_sec(sec);
    msg->mutable_header()->mutable_stamp()->set_nanosec(nanosec);
    // automsgs: PoseWithCovariance.pose is PoseStamped (not bare Pose).
    auto* pose = msg->mutable_pose()->mutable_pose()->mutable_pose();
    pose->mutable_position()->set_x(x);
    pose->mutable_position()->set_y(y);
    pose->mutable_position()->set_z(z);
    pose->mutable_orientation()->set_w(1.0);
    return msg;
}

}  // namespace

TEST(PoseFeeder, Affine3dFromPoseAndStamp) {
    automsgs::msgs::geometry_msgs::Pose pose;
    pose.mutable_position()->set_x(1.0);
    pose.mutable_position()->set_y(2.0);
    pose.mutable_position()->set_z(3.0);
    pose.mutable_orientation()->set_w(1.0);
    const Eigen::Affine3d T = autodriver::bridge::Affine3dFromPose(pose);
    EXPECT_NEAR(T.translation().x(), 1.0, 1e-9);
    EXPECT_NEAR(T.translation().y(), 2.0, 1e-9);
    EXPECT_NEAR(T.translation().z(), 3.0, 1e-9);

    automsgs::msgs::builtin_interfaces::Time stamp;
    stamp.set_sec(2);
    stamp.set_nanosec(500);
    EXPECT_EQ(autodriver::bridge::StampToNanoseconds(stamp),
              2'000'000'000ULL + 500ULL);
}

TEST(PoseFeeder, BuildTargetsUsesGlobalAndPerSensorChannel) {
    autodriver::Config config;
    config.compensator.pose_channel = "/localization/odom";

    autodriver::Config::Sensor a;
    a.id = "lidar/vlp16";
    a.params["enable_compensator"] = "true";
    config.sensors.push_back(a);

    autodriver::Config::Sensor b;
    b.id = "lidar/hesai";
    b.params["enable_compensator"] = "true";
    b.params["pose_channel"] = "/alt/odom";
    config.sensors.push_back(b);

    autodriver::Config::Sensor c;
    c.id = "lidar/off";
    c.params["enable_compensator"] = "false";
    config.sensors.push_back(c);

    const auto targets = autodriver::bridge::BuildPoseFeedTargets(config);
    ASSERT_EQ(targets.size(), 2u);
    ASSERT_EQ(targets.at("/localization/odom").size(), 1u);
    EXPECT_EQ(targets.at("/localization/odom")[0].id, "lidar/vlp16");
    ASSERT_EQ(targets.at("/alt/odom").size(), 1u);
    EXPECT_EQ(targets.at("/alt/odom")[0].id, "lidar/hesai");
}

TEST(PoseFeeder, BuildTargetsLoadsExtrinsic) {
    const std::string path = "/tmp/autodriver_pose_feeder_ext.yaml";
    {
        std::ofstream ofs(path);
        ASSERT_TRUE(ofs.is_open());
        ofs << "header:\n  frame_id: world\n"
               "child_frame_id: lidar\n"
               "transform:\n"
               "  translation: {x: 0.5, y: 0.0, z: 0.0}\n"
               "  rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}\n";
    }
    autodriver::Config config;
    config.compensator.pose_channel = "/odom";
    autodriver::Config::Sensor sensor;
    sensor.id = "lidar/vlp16";
    sensor.params["enable_compensator"] = "true";
    sensor.params["extrinsic_path"] = path;
    config.sensors.push_back(sensor);

    const auto targets = autodriver::bridge::BuildPoseFeedTargets(config);
    ASSERT_EQ(targets.at("/odom").size(), 1u);
    EXPECT_NEAR(targets.at("/odom")[0].base_T_lidar.translation().x(), 0.5,
                1e-9);
    std::remove(path.c_str());
}

TEST(PoseFeeder, FeedOdometryPushesThroughManager) {
    autodriver::Config config;
    config.compensator.pose_channel = "/odom";
    autodriver::Config::Sensor sensor;
    sensor.id = "lidar/vlp16";
    sensor.module = "Lidar3dModule";
    sensor.backend = "velodyne";
    sensor.autostart = true;
    sensor.params["enable_compensator"] = "true";
    sensor.params["source_type"] = "raw_packet";
    sensor.params["packets_per_scan"] = "1";
    config.sensors.push_back(sensor);

    autodriver::SensorManager manager(config);
    ASSERT_TRUE(manager.Initialize());
    ASSERT_TRUE(manager.Start());
    EXPECT_EQ(manager.AttachedCount(), 1u);

    autodriver::bridge::PoseFeeder feeder;
    ASSERT_TRUE(feeder.Start(nullptr, &manager, config));
    feeder.FeedOdometry("/odom", MakeOdom(1.0, 2.0, 3.0, 1, 0));

    Eigen::Affine3d probe = Eigen::Affine3d::Identity();
    probe.translation() = Eigen::Vector3d(9, 9, 9);
    EXPECT_TRUE(manager.PushLidarPose("lidar/vlp16", 42ULL, probe));

    manager.Stop();
}
