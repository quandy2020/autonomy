// Copyright 2026 The Openbot Authors
#include <atomic>
#include <chrono>
#include <filesystem>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <gtest/gtest.h>
#include "autonomy/localization/fusion/odometry_fusion_node.hpp"
#include "autonomy/transform/buffer_utils.hpp"

namespace autonomy::localization::fusion {
TEST(OdometryFusionNodeTest, LoadsShippedRobotConfiguration) {
    const auto path = std::filesystem::path(__FILE__).parent_path().parent_path() /
                      "conf/cartographer/wheel_imu_ekf.yaml";
    const auto config = LoadOdometryFusionConfig(path.string());
    EXPECT_NO_THROW(OdometryFusionNode(config, nullptr));
}

TEST(OdometryFusionNodeTest, RejectsFeedbackAndInvalidNoiseConfiguration) {
    OdometryFusionConfig c;
    c.output_topic = c.wheel_topic;
    EXPECT_THROW(OdometryFusionNode(c, nullptr), std::invalid_argument);
    c = {}; c.gyro_variance = -1;
    EXPECT_THROW(OdometryFusionNode(c, nullptr), std::invalid_argument);
    c = {}; c.reorder_delay = 1;
    EXPECT_THROW(OdometryFusionNode(c, nullptr), std::invalid_argument);
}

TEST(OdometryFusionNodeTest, RealTransportRotatedImuReorderingCovarianceAndTf) {
    ASSERT_TRUE(autolink::Init("odometry_fusion_test"));
    const std::string prefix = "/codex_ekf_test_" + std::to_string(getpid());
    auto node = autolink::CreateNode("ekf_test_" + std::to_string(getpid()));
    OdometryFusionConfig c;
    c.wheel_topic = prefix + "/wheel"; c.imu_topic = prefix + "/imu";
    c.output_topic = prefix + "/odometry"; c.tf_topic = prefix + "/tf";
    c.base_frame = "ekf_test_base_" + std::to_string(getpid());
    c.odom_frame = "ekf_test_odom_" + std::to_string(getpid());
    const std::string imu_frame = "ekf_test_imu_" + std::to_string(getpid());
    auto* buffer = transform::Buffer::Instance();
    automsgs::msgs::geometry_msgs::TransformStamped mount;
    mount.mutable_header()->set_frame_id(c.base_frame); mount.set_child_frame_id(imu_frame);
    mount.mutable_transform()->mutable_rotation()->set_x(std::sqrt(.5));
    mount.mutable_transform()->mutable_rotation()->set_w(std::sqrt(.5));
    transform::ApplyTransformStampedToBuffer(buffer, mount, "ekf_test", true);
    auto fusion = std::make_shared<OdometryFusionNode>(c, buffer);
    std::mutex mutex;
    automsgs::msgs::nav_msgs::Odometry result;
    automsgs::msgs::tf2_msgs::TFMessage tf;
    auto reader = node->CreateReader<OdometryFusionNode::Odometry>(c.output_topic, [&](const auto& msg) {
        std::lock_guard<std::mutex> lock(mutex); result = *msg;
    });
    auto tf_reader = node->CreateReader<automsgs::msgs::tf2_msgs::TFMessage>(c.tf_topic, [&](const auto& msg) {
        std::lock_guard<std::mutex> lock(mutex); tf = *msg;
    });
    ASSERT_TRUE(fusion->Start(node));
    auto wheel = node->CreateWriter<OdometryFusionNode::Odometry>(c.wheel_topic);
    auto imu = node->CreateWriter<OdometryFusionNode::Imu>(c.imu_topic);
    for (int i = 0; i < 100 && (!wheel->HasReader() || !imu->HasReader()); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_TRUE(wheel->HasReader() && imu->HasReader());
    auto send_wheel = [&](int ms) {
        OdometryFusionNode::Odometry msg;
        msg.mutable_header()->set_frame_id(c.odom_frame); msg.set_child_frame_id(c.base_frame);
        msg.mutable_header()->mutable_stamp()->set_sec(100);
        msg.mutable_header()->mutable_stamp()->set_nanosec(ms * 1000000);
        msg.mutable_twist()->mutable_twist()->mutable_linear()->set_x(1.);
        msg.mutable_twist()->mutable_twist()->mutable_angular()->set_z(-.5);
        for (int i = 0; i < 36; ++i) msg.mutable_twist()->add_covariance(0.);
        msg.mutable_twist()->set_covariance(35, 10.);
        msg.mutable_pose()->mutable_pose()->mutable_pose()->mutable_position()->set_x(9999.);
        wheel->Write(msg);
    };
    auto send_imu = [&](int ms) {
        OdometryFusionNode::Imu msg;
        msg.mutable_header()->set_frame_id(imu_frame);
        msg.mutable_header()->mutable_stamp()->set_sec(100);
        msg.mutable_header()->mutable_stamp()->set_nanosec(ms * 1000000);
        msg.mutable_angular_velocity()->set_y(.4);
        for (int i = 0; i < 9; ++i) msg.add_angular_velocity_covariance(i % 4 == 0 ? 1e-6 : 0.);
        imu->Write(msg);
    };
    // Arrival order differs from sensor time. The queue must restore chronology.
    send_wheel(0); send_imu(100); send_wheel(50); send_imu(25);
    send_wheel(150); send_imu(200); send_wheel(100); send_imu(125);
    for (int i = 0; i < 200; ++i) {
        { std::lock_guard<std::mutex> lock(mutex);
          if (result.header().stamp().nanosec() == 200000000 && !tf.transforms().empty() &&
              tf.transforms(0).header().stamp().nanosec() == 200000000) break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    {
        std::lock_guard<std::mutex> lock(mutex);
        EXPECT_EQ(result.header().stamp().sec(), 100);
        EXPECT_EQ(result.header().stamp().nanosec(), 200000000);
        EXPECT_EQ(result.header().frame_id(), c.odom_frame);
        EXPECT_EQ(result.child_frame_id(), c.base_frame);
        EXPECT_NEAR(result.twist().twist().angular().z(), .4, .01);
        EXPECT_GT(result.pose().pose().pose().position().x(), .1);
        EXPECT_LT(result.pose().pose().pose().position().x(), .3);
        EXPECT_EQ(result.pose().covariance_size(), 36);
        EXPECT_GT(result.pose().covariance(0), 0.);
        ASSERT_EQ(tf.transforms_size(), 1);
        EXPECT_EQ(tf.transforms(0).header().stamp().nanosec(), result.header().stamp().nanosec());
        EXPECT_EQ(tf.transforms(0).child_frame_id(), c.base_frame);
        EXPECT_DOUBLE_EQ(tf.transforms(0).transform().translation().x(), result.pose().pose().pose().position().x());
    }
    fusion->Stop(); fusion.reset();
    reader->Shutdown(); tf_reader->Shutdown();
    reader.reset(); tf_reader.reset(); wheel.reset(); imu.reset(); node.reset();
    autolink::Clear();
}
}  // namespace autonomy::localization::fusion
