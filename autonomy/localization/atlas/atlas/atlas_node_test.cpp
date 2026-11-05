/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "atlas/atlas_node.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
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

    static bool AlignGroundTruth(
        AtlasNode* node,
        const ::automsgs::msgs::nav_msgs::Odometry& atlas_odometry,
        Eigen::Isometry3d* ground_truth_world_from_camera) {
        return node->AlignGroundTruthToAtlas(
            atlas_odometry, ground_truth_world_from_camera);
    }

    static void ObserveCoordinateFrame(AtlasNode* node,
                                       const TrackingResult& result) {
        node->ObserveGroundTruthCoordinateFrame(result);
    }
};

class AtlasNodeTimestampTest : public ::testing::Test
{
protected:
    static bool Accept(AtlasNode* node, double timestamp) {
        return node->AcceptTrackingTimestamp(timestamp);
    }
};

class MessageAdapterEpochTest : public ::testing::Test
{
protected:
    static bool Record(MessageAdapter* adapter,
                       const TrackingResult& result) {
        return adapter->Record(result);
    }
};

class AtlasNodeCoordinateFrameTest : public ::testing::Test
{
protected:
    static void Observe(AtlasNode* node, const TrackingResult& result) {
        node->ObserveGroundTruthCoordinateFrame(result);
    }

    static bool Align(
        AtlasNode* node,
        const ::automsgs::msgs::nav_msgs::Odometry& atlas_odometry,
        Eigen::Isometry3d* ground_truth_world_from_camera) {
        return node->AlignGroundTruthToAtlas(
            atlas_odometry, ground_truth_world_from_camera);
    }

    static void AddGroundTruthPathPose(AtlasNode* node) {
        node->ground_truth_path_.add_poses();
    }

    static int GroundTruthPathSize(const AtlasNode& node) {
        return node.ground_truth_path_.poses_size();
    }

    static bool HasGroundTruthAlignment(const AtlasNode& node) {
        return node.have_ground_truth_alignment_;
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
    EXPECT_EQ(options.maximum_dense_map_points, 3000000U);
}

TEST_F(AtlasNodeTimestampTest, RejectsNonMonotonicSensorDelivery) {
    AtlasNode node(AtlasNode::Options{});
    EXPECT_TRUE(Accept(&node, 10.0));
    EXPECT_FALSE(Accept(&node, 10.0));
    EXPECT_FALSE(Accept(&node, 9.9));
    EXPECT_TRUE(Accept(&node, 10.1));
    EXPECT_FALSE(Accept(
        &node, std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(Accept(&node, -1.0));
}

TEST(MessageAdapterOptionsTest, UsesCameraLinkByDefault) {
    const MessageAdapter::Options options;
    EXPECT_EQ(options.map_frame, "map");
    EXPECT_EQ(options.camera_frame, "camera_link");
    EXPECT_EQ(options.maximum_dense_map_points, 3000000U);
}

TEST(MessageAdapterMapSnapshotTest, EncodesOneCallerPinnedGeneration) {
    auto snapshot = std::make_shared<map::MapSnapshot>();
    snapshot->dense_map.timestamp = 42.25;
    snapshot->dense_map.generation = 17;
    map::ColorPoint point;
    point.position = Eigen::Vector3f(1.0F, 2.0F, 3.0F);
    point.red = 11;
    point.green = 22;
    point.blue = 33;
    snapshot->dense_map.points.push_back(point);
    snapshot->navigation_map.timestamp = 42.25;
    snapshot->navigation_map.generation = 17;

    snapshot->occupancy_grid.timestamp = 42.25;
    snapshot->occupancy_grid.generation = 17;
    snapshot->occupancy_grid.resolution = 0.1;
    snapshot->occupancy_grid.width = 2;
    snapshot->occupancy_grid.height = 1;
    snapshot->occupancy_grid.origin_translation = Eigen::Vector3d(1.0, 2.0, 0.21);
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
    EXPECT_NEAR(grid.info().origin().position().x(), 1.0, 1e-12);
    EXPECT_NEAR(grid.info().origin().position().y(), 2.0, 1e-12);
    EXPECT_NEAR(grid.info().origin().position().z(), 0.0, 1e-12);
    EXPECT_EQ(grid.data_size(), 2);
}

TEST(MessageAdapterMapSnapshotTest, PublishesGlobalDenseMapNotRollingNavigation) {
    auto snapshot = std::make_shared<map::MapSnapshot>();
    snapshot->dense_map.timestamp = 1.5;
    snapshot->navigation_map.timestamp = 1.5;
    map::ColorPoint kept;
    kept.position = Eigen::Vector3f(10.0F, 0.0F, 0.0F);
    kept.red = 255;
    snapshot->dense_map.points.push_back(kept);
    map::ColorPoint far;
    far.position = Eigen::Vector3f(40.0F, 0.0F, 0.0F);
    far.red = 1;
    snapshot->dense_map.points.push_back(far);
    snapshot->navigation_map.points.push_back(kept);

    MessageAdapter adapter(nullptr);
    ::automsgs::msgs::sensor_msgs::PointCloud2 cloud;
    ASSERT_TRUE(adapter.GetDenseMap(snapshot, &cloud));
    EXPECT_EQ(cloud.width(), 2U);
}

TEST(MessageAdapterMapSnapshotTest, RejectsMissingCallerSnapshot) {
    MessageAdapter adapter(nullptr);
    const std::shared_ptr<const map::MapSnapshot> snapshot;
    ::automsgs::msgs::sensor_msgs::PointCloud2 cloud;
    ::automsgs::msgs::map_msgs::OccupancyGrid grid;
    EXPECT_FALSE(adapter.GetDenseMap(snapshot, &cloud));
    EXPECT_FALSE(adapter.GetOccupancyGrid(snapshot, &grid));
}

TEST_F(MessageAdapterEpochTest, ClearsPathAtInvalidCoordinateEpochBoundary) {
    MessageAdapter adapter(nullptr);
    TrackingResult result;
    result.time = 1.0;
    result.valid = true;
    result.map_id_valid = true;
    result.map_id = 7;
    result.coordinate_epoch = 2;
    result.destructive_epoch = 3;
    result.coordinate_frame_ready = false;
    ASSERT_TRUE(Record(&adapter, result));

    ::automsgs::msgs::nav_msgs::Path path;
    ASSERT_TRUE(adapter.GetPath(&path));
    ASSERT_EQ(path.poses_size(), 1);

    result.time = 2.0;
    result.valid = false;
    result.coordinate_epoch = 3;
    result.coordinate_frame_ready = true;
    EXPECT_FALSE(Record(&adapter, result));
    EXPECT_FALSE(adapter.GetPath(&path));

    result.time = 3.0;
    result.valid = true;
    ASSERT_TRUE(Record(&adapter, result));
    ASSERT_TRUE(adapter.GetPath(&path));
    ASSERT_EQ(path.poses_size(), 1);
    EXPECT_EQ(path.poses(0).header().stamp().sec(), 3);
}

TEST_F(MessageAdapterEpochTest, ClearsPathForDestructiveEpochAndMapChanges) {
    MessageAdapter adapter(nullptr);
    TrackingResult result;
    result.valid = true;
    result.map_id_valid = true;
    result.map_id = 4;
    result.coordinate_epoch = 1;
    result.destructive_epoch = 8;
    ASSERT_TRUE(Record(&adapter, result));

    result.valid = false;
    ++result.destructive_epoch;
    EXPECT_FALSE(Record(&adapter, result));
    ::automsgs::msgs::nav_msgs::Path path;
    EXPECT_FALSE(adapter.GetPath(&path));

    result.valid = true;
    ASSERT_TRUE(Record(&adapter, result));
    ++result.map_id;
    result.valid = false;
    EXPECT_FALSE(Record(&adapter, result));
    EXPECT_FALSE(adapter.GetPath(&path));
}

TEST_F(AtlasNodeTfTest, MapToOdomKeepsChassisUprightWithImuWorld) {
    AtlasNode::Options options;
    options.camera_frame = "camera_optical_frame";
    const AtlasNode node{options};

    Eigen::Matrix3d R_imu_optical;
    R_imu_optical << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
    Eigen::Isometry3d imu_from_optical = Eigen::Isometry3d::Identity();
    imu_from_optical.linear() = R_imu_optical;
    imu_from_optical.translation() = Eigen::Vector3d(0.062, 0.0, 0.132);

    Eigen::Isometry3d odom_from_imu = Eigen::Isometry3d::Identity();
    odom_from_imu.translation() = Eigen::Vector3d(-0.032, 0.0, 0.068);
    const Eigen::Isometry3d odom_from_optical =
        odom_from_imu * imu_from_optical;
    const Eigen::Quaterniond optical_rotation(imu_from_optical.linear());
    const Eigen::Quaterniond odom_optical_rotation(odom_from_optical.linear());

    ::automsgs::msgs::nav_msgs::Odometry odometry;
    odometry.mutable_header()->set_frame_id("map");
    odometry.mutable_header()->mutable_stamp()->set_sec(10);
    odometry.set_child_frame_id("camera_optical_frame");
    SetPose(imu_from_optical.translation(), optical_rotation,
            odometry.mutable_pose()->mutable_pose()->mutable_pose());

    ::automsgs::msgs::geometry_msgs::TransformStamped odom_from_camera;
    odom_from_camera.mutable_header()->set_frame_id("odom");
    *odom_from_camera.mutable_header()->mutable_stamp() =
        odometry.header().stamp();
    odom_from_camera.set_child_frame_id("camera_optical_frame");
    SetTransform(odom_from_optical.translation(), odom_optical_rotation,
                 odom_from_camera.mutable_transform());

    ::automsgs::msgs::geometry_msgs::TransformStamped map_from_odom;
    ASSERT_TRUE(
        ComputeMapToOdom(node, odometry, odom_from_camera, &map_from_odom));
    EXPECT_NEAR(map_from_odom.transform().translation().x(), 0.032, 1e-9);
    EXPECT_NEAR(map_from_odom.transform().translation().y(), 0.0, 1e-9);
    EXPECT_NEAR(map_from_odom.transform().translation().z(), -0.068, 1e-9);
    EXPECT_NEAR(map_from_odom.transform().rotation().x(), 0.0, 1e-9);
    EXPECT_NEAR(map_from_odom.transform().rotation().y(), 0.0, 1e-9);
    EXPECT_NEAR(map_from_odom.transform().rotation().z(), 0.0, 1e-9);
    EXPECT_NEAR(std::abs(map_from_odom.transform().rotation().w()), 1.0, 1e-9);
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

TEST_F(AtlasNodeTfTest, AnchorsGroundTruthInAtlasWorldOnce) {
    AtlasNode::Options options;
    options.camera_frame = "camera_optical_frame";
    AtlasNode node{options};
    TrackingResult coordinate_frame;
    coordinate_frame.map_id_valid = true;
    coordinate_frame.map_id = 1;
    coordinate_frame.coordinate_frame_ready = true;
    ObserveCoordinateFrame(&node, coordinate_frame);

    ::automsgs::msgs::nav_msgs::Odometry atlas_odometry;
    atlas_odometry.mutable_header()->set_frame_id("map");
    atlas_odometry.set_child_frame_id("camera_optical_frame");
    const Eigen::Quaterniond atlas_rotation(
        Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
    SetPose(Eigen::Vector3d(1.0, 2.0, 0.5), atlas_rotation,
            atlas_odometry.mutable_pose()->mutable_pose()->mutable_pose());

    Eigen::Isometry3d ground_truth_first = Eigen::Isometry3d::Identity();
    ground_truth_first.translation() = Eigen::Vector3d(10.0, -4.0, 0.8);
    ground_truth_first.linear() =
        Eigen::AngleAxisd(-0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    ASSERT_TRUE(AlignGroundTruth(
        &node, atlas_odometry, &ground_truth_first));
    EXPECT_TRUE(ground_truth_first.translation().isApprox(
        Eigen::Vector3d(1.0, 2.0, 0.5), 1e-12));
    EXPECT_TRUE(ground_truth_first.linear().isApprox(
        atlas_rotation.toRotationMatrix(), 1e-12));

    Eigen::Isometry3d ground_truth_second = Eigen::Isometry3d::Identity();
    ground_truth_second.translation() = Eigen::Vector3d(10.0, -3.0, 0.8);
    ground_truth_second.linear() =
        Eigen::AngleAxisd(-0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    ASSERT_TRUE(AlignGroundTruth(
        &node, atlas_odometry, &ground_truth_second));

    const Eigen::Vector3d expected_translation =
        Eigen::Vector3d(1.0, 2.0, 0.5) +
        atlas_rotation *
            (Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) *
             Eigen::Vector3d(0.0, 1.0, 0.0));
    EXPECT_TRUE(ground_truth_second.translation().isApprox(
        expected_translation, 1e-12));
    EXPECT_TRUE(ground_truth_second.linear().isApprox(
        atlas_rotation.toRotationMatrix(), 1e-12));
}

TEST_F(AtlasNodeCoordinateFrameTest,
       GatesGroundTruthUntilReadyAndClearsAnchorOnEpochChange) {
    AtlasNode::Options options;
    options.camera_frame = "camera_optical_frame";
    AtlasNode node{options};

    ::automsgs::msgs::nav_msgs::Odometry atlas_odometry;
    atlas_odometry.mutable_header()->set_frame_id("map");
    atlas_odometry.set_child_frame_id("camera_optical_frame");
    atlas_odometry.mutable_pose()
        ->mutable_pose()
        ->mutable_pose()
        ->mutable_orientation()
        ->set_w(1.0);
    Eigen::Isometry3d ground_truth = Eigen::Isometry3d::Identity();

    TrackingResult result;
    result.map_id_valid = true;
    result.map_id = 9;
    result.coordinate_epoch = 1;
    result.destructive_epoch = 2;
    result.coordinate_frame_ready = false;
    Observe(&node, result);
    EXPECT_FALSE(Align(&node, atlas_odometry, &ground_truth));
    EXPECT_FALSE(HasGroundTruthAlignment(node));

    result.coordinate_frame_ready = true;
    Observe(&node, result);
    ASSERT_TRUE(Align(&node, atlas_odometry, &ground_truth));
    EXPECT_TRUE(HasGroundTruthAlignment(node));
    AddGroundTruthPathPose(&node);
    ASSERT_EQ(GroundTruthPathSize(node), 1);

    result.valid = false;
    ++result.coordinate_epoch;
    Observe(&node, result);
    EXPECT_EQ(GroundTruthPathSize(node), 0);
    EXPECT_FALSE(HasGroundTruthAlignment(node));
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
