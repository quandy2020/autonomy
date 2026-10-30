/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "atlas/atlas_node.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/buffer_utils.hpp"

namespace autonomy::localization::atlas {
namespace {

constexpr double kHalfSqrtTwo = 0.70710678118654752440;

void SetPose(const Eigen::Vector3d& translation,
             const Eigen::Quaterniond& rotation,
             ::automsgs::msgs::geometry_msgs::Pose* pose) {
    pose->mutable_position()->set_x(translation.x());
    pose->mutable_position()->set_y(translation.y());
    pose->mutable_position()->set_z(translation.z());
    pose->mutable_orientation()->set_x(rotation.x());
    pose->mutable_orientation()->set_y(rotation.y());
    pose->mutable_orientation()->set_z(rotation.z());
    pose->mutable_orientation()->set_w(rotation.w());
}

void SetTransform(const Eigen::Vector3d& translation,
                  const Eigen::Quaterniond& rotation,
                  ::automsgs::msgs::geometry_msgs::Transform* transform) {
    transform->mutable_translation()->set_x(translation.x());
    transform->mutable_translation()->set_y(translation.y());
    transform->mutable_translation()->set_z(translation.z());
    transform->mutable_rotation()->set_x(rotation.x());
    transform->mutable_rotation()->set_y(rotation.y());
    transform->mutable_rotation()->set_z(rotation.z());
    transform->mutable_rotation()->set_w(rotation.w());
}

}  // namespace

class AtlasNodeTfTest : public ::testing::Test
{
protected:
    static bool ComputeMapToOdom(
        const AtlasNode& node,
        const ::automsgs::msgs::nav_msgs::Odometry& odometry,
        const ::automsgs::msgs::geometry_msgs::TransformStamped&
            odom_from_camera,
        ::automsgs::msgs::geometry_msgs::TransformStamped* map_from_odom) {
        return node.ComputeMapToOdom(odometry, odom_from_camera, map_from_odom);
    }

    static void IngestTf(
        AtlasNode* node, ::autonomy::transform::Buffer* buffer,
        const std::shared_ptr<::automsgs::msgs::tf2_msgs::TFMessage>& message,
        bool is_static) {
        node->tf_buffer_ = buffer;
        node->running_ = true;
        node->OnTfMessage(message, is_static);
        node->running_ = false;
        node->tf_buffer_ = nullptr;
    }
};

TEST(AtlasNodeOptionsTest, PublishesMapToOdomTfByDefault) {
    const AtlasNode::Options options;
    EXPECT_TRUE(options.publish_tf);
    EXPECT_EQ(options.tf_topic, "/tf");
    EXPECT_EQ(options.tf_static_topic, "/tf_static");
    EXPECT_EQ(options.map_frame, "map");
    EXPECT_EQ(options.odom_frame, "odom");
    EXPECT_EQ(options.base_frame, "base_link");
    EXPECT_EQ(options.camera_frame, "camera_link");
    EXPECT_EQ(options.maximum_dense_map_points, 1500000U);
}

TEST(MessageAdapterOptionsTest, UsesCameraLinkByDefault) {
    const MessageAdapter::Options options;
    EXPECT_EQ(options.map_frame, "map");
    EXPECT_EQ(options.camera_frame, "camera_link");
    EXPECT_EQ(options.maximum_dense_map_points, 1500000U);
}

TEST(MessageAdapterMapSnapshotTest, EncodesOneCallerPinnedGeneration) {
    auto snapshot = std::make_shared<map::MapSnapshot>();
    snapshot->navigation_map.timestamp = 42.25;
    snapshot->navigation_map.generation = 17;
    map::ColorPoint point;
    point.position = Eigen::Vector3f(1.0F, 2.0F, 3.0F);
    point.red = 11;
    point.green = 22;
    point.blue = 33;
    snapshot->navigation_map.points.push_back(point);

    snapshot->occupancy_grid.timestamp = 42.25;
    snapshot->occupancy_grid.generation = 17;
    snapshot->occupancy_grid.resolution = 0.1;
    snapshot->occupancy_grid.width = 2;
    snapshot->occupancy_grid.height = 1;
    snapshot->occupancy_grid.data = {0, 100};

    MessageAdapter adapter(nullptr);
    ::automsgs::msgs::sensor_msgs::PointCloud2 cloud;
    ::automsgs::msgs::map_msgs::OccupancyGrid grid;
    ASSERT_TRUE(adapter.GetDenseMap(snapshot, &cloud));
    ASSERT_TRUE(adapter.GetOccupancyGrid(snapshot, &grid));
    EXPECT_EQ(cloud.header().stamp().sec(), 42);
    EXPECT_EQ(cloud.header().stamp().nanosec(), 250000000U);
    EXPECT_EQ(grid.header().stamp().sec(),
              cloud.header().stamp().sec());
    EXPECT_EQ(grid.header().stamp().nanosec(),
              cloud.header().stamp().nanosec());
    EXPECT_EQ(cloud.header().frame_id(), "map");
    EXPECT_EQ(grid.header().frame_id(), "map");
    EXPECT_EQ(cloud.width(), 1U);
    EXPECT_EQ(grid.info().width(), 2U);
    EXPECT_EQ(grid.info().height(), 1U);
    EXPECT_EQ(grid.data_size(), 2);
}

TEST(MessageAdapterMapSnapshotTest, RejectsMissingCallerSnapshot) {
    MessageAdapter adapter(nullptr);
    const std::shared_ptr<const map::MapSnapshot> snapshot;
    ::automsgs::msgs::sensor_msgs::PointCloud2 cloud;
    ::automsgs::msgs::map_msgs::OccupancyGrid grid;
    EXPECT_FALSE(adapter.GetDenseMap(snapshot, &cloud));
    EXPECT_FALSE(adapter.GetOccupancyGrid(snapshot, &grid));
}

TEST_F(AtlasNodeTfTest, ComputesMapToOdomWithOdometryStamp) {
    const AtlasNode node{AtlasNode::Options()};
    const Eigen::Quaterniond quarter_turn(kHalfSqrtTwo, 0.0, 0.0, kHalfSqrtTwo);

    ::automsgs::msgs::nav_msgs::Odometry odometry;
    odometry.mutable_header()->set_frame_id("map");
    odometry.mutable_header()->mutable_stamp()->set_sec(123);
    odometry.mutable_header()->mutable_stamp()->set_nanosec(456789123);
    odometry.set_child_frame_id("camera_link");
    SetPose(Eigen::Vector3d(10.0, 5.0, 1.0), quarter_turn,
            odometry.mutable_pose()->mutable_pose()->mutable_pose());

    ::automsgs::msgs::geometry_msgs::TransformStamped odom_from_camera;
    odom_from_camera.mutable_header()->set_frame_id("odom");
    *odom_from_camera.mutable_header()->mutable_stamp() =
        odometry.header().stamp();
    odom_from_camera.set_child_frame_id("camera_link");
    SetTransform(Eigen::Vector3d(3.0, 2.0, 1.0), quarter_turn,
                 odom_from_camera.mutable_transform());

    ::automsgs::msgs::geometry_msgs::TransformStamped map_from_odom;
    ASSERT_TRUE(
        ComputeMapToOdom(node, odometry, odom_from_camera, &map_from_odom));
    EXPECT_EQ(map_from_odom.header().frame_id(), "map");
    EXPECT_EQ(map_from_odom.child_frame_id(), "odom");
    EXPECT_EQ(map_from_odom.header().stamp().sec(), 123);
    EXPECT_EQ(map_from_odom.header().stamp().nanosec(), 456789123);
    EXPECT_NEAR(map_from_odom.transform().translation().x(), 7.0, 1e-12);
    EXPECT_NEAR(map_from_odom.transform().translation().y(), 3.0, 1e-12);
    EXPECT_NEAR(map_from_odom.transform().translation().z(), 0.0, 1e-12);
    EXPECT_NEAR(map_from_odom.transform().rotation().x(), 0.0, 1e-12);
    EXPECT_NEAR(map_from_odom.transform().rotation().y(), 0.0, 1e-12);
    EXPECT_NEAR(map_from_odom.transform().rotation().z(), 0.0, 1e-12);
    EXPECT_NEAR(std::abs(map_from_odom.transform().rotation().w()), 1.0, 1e-12);
}

TEST_F(AtlasNodeTfTest, FiltersMapParentAndBuildsOdomCameraChain) {
    auto* buffer = ::autonomy::transform::Buffer::Instance();
    ASSERT_NE(buffer, nullptr);
    buffer->clear();

    AtlasNode node{AtlasNode::Options()};
    auto dynamic_message =
        std::make_shared<::automsgs::msgs::tf2_msgs::TFMessage>();
    auto add_identity =
        [](const std::shared_ptr<::automsgs::msgs::tf2_msgs::TFMessage>&
               message,
           const std::string& parent, const std::string& child, int seconds,
           std::uint32_t nanoseconds) {
            auto* transform = message->add_transforms();
            transform->mutable_header()->set_frame_id(parent);
            transform->mutable_header()->mutable_stamp()->set_sec(seconds);
            transform->mutable_header()->mutable_stamp()->set_nanosec(
                nanoseconds);
            transform->set_child_frame_id(child);
            transform->mutable_transform()->mutable_rotation()->set_w(1.0);
        };
    add_identity(dynamic_message, "map", "odom", 42, 123);
    add_identity(dynamic_message, "odom", "base_link", 42, 123);
    add_identity(dynamic_message, "base_link", "camera_link", 42, 123);
    IngestTf(&node, buffer, dynamic_message, false);

    ::automsgs::msgs::builtin_interfaces::Time query_time;
    query_time.set_sec(42);
    query_time.set_nanosec(123);
    std::string error;
    EXPECT_TRUE(
        buffer->canTransform("odom", "camera_link", query_time, 0.0f, &error))
        << error;
    error.clear();

    // A mount edge mixed into /tf must still be cached as static.
    constexpr std::uint64_t kFarFutureNanoseconds = 142000000123ULL;
    EXPECT_TRUE(
        static_cast<const ::autonomy::transform::tf2::BufferCore*>(buffer)
            ->canTransform("base_link", "camera_link", kFarFutureNanoseconds,
                           &error))
        << error;
    error.clear();

    // Atlas must not ingest its own map -> odom output from /tf.
    EXPECT_FALSE(buffer->canTransform("map", "odom", query_time, 0.0f, &error));

    // A downstream TF consumer receives the Atlas output in addition to the
    // externally owned odom -> base_link -> camera_link transforms.
    ::autonomy::transform::ApplyTransformStampedToBuffer(
        buffer, dynamic_message->transforms(0), "atlas_output", false);
    error.clear();
    EXPECT_TRUE(
        buffer->canTransform("map", "camera_link", query_time, 0.0f, &error))
        << error;
    buffer->clear();
}

TEST_F(AtlasNodeTfTest, RejectsStaleDynamicTransform) {
    const AtlasNode node{AtlasNode::Options()};
    ::automsgs::msgs::nav_msgs::Odometry odometry;
    odometry.mutable_header()->set_frame_id("map");
    odometry.mutable_header()->mutable_stamp()->set_sec(100);
    odometry.set_child_frame_id("camera_link");
    odometry.mutable_pose()
        ->mutable_pose()
        ->mutable_pose()
        ->mutable_orientation()
        ->set_w(1.0);

    ::automsgs::msgs::geometry_msgs::TransformStamped odom_from_camera;
    odom_from_camera.mutable_header()->set_frame_id("odom");
    odom_from_camera.mutable_header()->mutable_stamp()->set_sec(98);
    odom_from_camera.set_child_frame_id("camera_link");
    odom_from_camera.mutable_transform()->mutable_rotation()->set_w(1.0);

    ::automsgs::msgs::geometry_msgs::TransformStamped map_from_odom;
    EXPECT_FALSE(
        ComputeMapToOdom(node, odometry, odom_from_camera, &map_from_odom));

    odom_from_camera.mutable_header()->mutable_stamp()->set_sec(99);
    odom_from_camera.mutable_header()->mutable_stamp()->set_nanosec(950000000);
    EXPECT_TRUE(
        ComputeMapToOdom(node, odometry, odom_from_camera, &map_from_odom));

    odom_from_camera.mutable_header()->mutable_stamp()->set_sec(0);
    odom_from_camera.mutable_header()->mutable_stamp()->set_nanosec(0);
    EXPECT_TRUE(
        ComputeMapToOdom(node, odometry, odom_from_camera, &map_from_odom));
}

TEST_F(AtlasNodeTfTest, RejectsMismatchedTfFrames) {
    const AtlasNode node{AtlasNode::Options()};
    ::automsgs::msgs::nav_msgs::Odometry odometry;
    odometry.mutable_header()->set_frame_id("map");
    odometry.set_child_frame_id("camera_link");
    odometry.mutable_pose()
        ->mutable_pose()
        ->mutable_pose()
        ->mutable_orientation()
        ->set_w(1.0);

    ::automsgs::msgs::geometry_msgs::TransformStamped odom_from_camera;
    odom_from_camera.mutable_header()->set_frame_id("wrong_odom");
    odom_from_camera.set_child_frame_id("camera_link");
    odom_from_camera.mutable_transform()->mutable_rotation()->set_w(1.0);

    ::automsgs::msgs::geometry_msgs::TransformStamped map_from_odom;
    EXPECT_FALSE(
        ComputeMapToOdom(node, odometry, odom_from_camera, &map_from_odom));
}

}  // namespace autonomy::localization::atlas
