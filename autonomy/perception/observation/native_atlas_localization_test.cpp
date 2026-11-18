/* Copyright 2026 The Openbot Authors. */
/** @file native_atlas_localization_test.cpp
 * @brief Verify actual generated Atlas message correlation, calibrated composition and coordinate revocation.
 * @details Controlled native-message fixtures do not claim to replace a real SLAM run.
 */
#include <iostream>
#include <stdexcept>
#include "autonomy/perception/observation/native_atlas_localization.hpp"

namespace {
namespace p = autonomy::perception;
using Odometry = ::automsgs::msgs::nav_msgs::Odometry;
using DiagnosticArray = ::automsgs::msgs::diagnostic_msgs::DiagnosticArray;
using TransformStamped = ::automsgs::msgs::geometry_msgs::TransformStamped;

/** @brief Report the specific actual correlation invariant that failed. */
void Require(bool value, const char* reason) { if (!value) throw std::runtime_error(reason); }
/** @brief Configure an explicit physical installation and native endpoints. */
p::NativeAtlasLocalizationOptions Options() {
    p::NativeAtlasLocalizationOptions options;
    options.backend_frame = "atlas"; options.navigation_frame = "navigation";
    options.camera_frame = "depth_optical"; options.base_frame = "base";
    options.odometry_topic = "/actual_atlas/odometry"; options.calibration_revision = 7;
    options.base_from_camera.mutable_header()->set_frame_id("base");
    options.base_from_camera.set_child_frame_id("depth_optical");
    options.base_from_camera.mutable_transform()->mutable_rotation()->set_w(1);
    options.base_from_camera.mutable_transform()->mutable_translation()->set_z(.2);
    return options;
}
/** @brief Native camera pose with exact integer Header identity. */
Odometry Pose(unsigned nanosecond = 250000000) {
    Odometry message;
    message.mutable_header()->set_frame_id("atlas");
    message.mutable_header()->mutable_stamp()->set_sec(3);
    message.mutable_header()->mutable_stamp()->set_nanosec(nanosecond);
    message.set_child_frame_id("depth_optical");
    auto* pose = message.mutable_pose()->mutable_pose();
    *pose->mutable_header() = message.header();
    pose->mutable_pose()->mutable_orientation()->set_w(1);
    pose->mutable_pose()->mutable_position()->set_x(1);
    pose->mutable_pose()->mutable_position()->set_y(2);
    pose->mutable_pose()->mutable_position()->set_z(.5);
    return message;
}
/** @brief Actual diagnostic schema, with controllable source health and raw zero-valued map ids. */
DiagnosticArray Status(unsigned nanosecond = 250000000, bool healthy = true,
    const char* instance = "actual_backend_start", const char* epoch = "0") {
    DiagnosticArray message;
    *message.mutable_header() = Pose(nanosecond).header();
    auto* status = message.add_status(); status->set_name("atlas/localization");
    status->set_hardware_id(instance);
    status->set_level(healthy ? ::automsgs::msgs::diagnostic_msgs::DiagnosticStatus::OK :
        ::automsgs::msgs::diagnostic_msgs::DiagnosticStatus::ERROR);
    for (const auto& item : {std::pair<const char*, const char*>{"schema", "autonomy.atlas.tracking.v1"},
        {"sensor_mode", "rgbd"}, {"camera_frame", "depth_optical"}, {"odometry_topic", "/actual_atlas/odometry"},
        {"healthy", healthy ? "true" : "false"}, {"pose_valid", healthy ? "true" : "false"},
        {"map_id_valid", "true"}, {"coordinate_frame_ready", "true"},
        {"matching_odometry", healthy ? "true" : "false"}, {"map_id", "0"},
        {"coordinate_epoch", epoch}, {"destructive_epoch", "0"}}) {
        auto* value = status->add_values(); value->set_key(item.first); value->set_value(item.second);
    }
    return message;
}
/** @brief Explicit platform-supplied alignment, anchored to a real native acquisition. */
TransformStamped Alignment(const Odometry& pose) {
    TransformStamped value;
    *value.mutable_header()->mutable_stamp() = pose.header().stamp();
    value.mutable_header()->set_frame_id("navigation"); value.set_child_frame_id("atlas");
    value.mutable_transform()->mutable_rotation()->set_w(1);
    value.mutable_transform()->mutable_translation()->set_x(10);
    value.mutable_transform()->mutable_translation()->set_y(-5);
    value.mutable_transform()->mutable_translation()->set_z(1);
    return value;
}
/** @brief Native messages may arrive in either order; neither half can attest a pose. */
void AcquisitionAndCalibration() {
    for (bool pose_first : {false, true}) {
        p::NativeAtlasLocalization source(Options());
        Require(pose_first ? source.PushOdometry(Pose()) : source.PushStatus(Status()), "first native half accepted");
        Require(!source.LatestBackendPose(3.4) && !source.Lookup(p::SensorKind::kRgbd, "depth_optical", 3.25, 3.4),
            "one native half cannot manufacture localization");
        Require(pose_first ? source.PushStatus(Status()) : source.PushOdometry(Pose()), "second native half accepted");
        Require(source.LatestBackendPose(3.4).has_value() && source.map_epoch() == 1,
            "actual healthy pair maps legal raw zero ids to a nonzero consumer epoch");
        Require(!source.Lookup(p::SensorKind::kRgbd, "depth_optical", 3.25, 3.4), "no default world alignment");
        Require(source.SetWorldAlignment(Alignment(Pose()), 3.4), "explicit acquisition-anchored alignment installed");
        const auto proof = source.Lookup(p::SensorKind::kRgbd, "depth_optical", 3.25, 3.4);
        Require(proof && proof->Valid(p::SensorKind::kRgbd, "depth_optical", 3.25, 3.4) &&
            proof->world_from_sensor.translation.x == 11 && proof->world_from_sensor.translation.y == -3 &&
            std::abs(proof->world_from_base.translation.z - 1.3) < 1e-12 && proof->calibration_revision == 7,
            "base and camera poses use actual installation and explicit world alignment");
        Require(!source.Lookup(p::SensorKind::kLidar, "depth_optical", 3.25, 3.4) &&
            !source.Lookup(p::SensorKind::kRgbd, "rgb_frame", 3.25, 3.4) &&
            !source.Lookup(p::SensorKind::kRgbd, "depth_optical", 3.35, 3.4) &&
            !source.Lookup(p::SensorKind::kRgbd, "depth_optical", 3.25, 3.9),
            "wrong source, wrong frame, different acquisition and expired evidence are rejected");
    }
}
/** @brief Actual coordinate changes revoke alignment and cannot be undone by a late old status. */
void CoordinateRevocation() {
    p::NativeAtlasLocalization source(Options());
    Require(source.PushStatus(Status()) && source.PushOdometry(Pose()) &&
        source.SetWorldAlignment(Alignment(Pose()), 3.4), "initial true pair and alignment");
    Require(source.PushStatus(Status(350000000, true, "actual_backend_start", "1")) &&
        source.PushOdometry(Pose(350000000)) && source.map_epoch() == 2 && !source.WorldAlignment(),
        "real coordinate epoch change revokes the previous alignment");
    Require(!source.PushStatus(Status()) &&
        !source.Lookup(p::SensorKind::kRgbd, "depth_optical", 3.25, 3.4), "late old identity cannot restore old pose support");
    Require(source.SetWorldAlignment(Alignment(Pose(350000000)), 3.4), "new epoch receives its own explicit alignment");
    Require(source.PushStatus(Status(450000000, true, "next_backend_start", "0")) &&
        source.map_epoch() == 3 && !source.WorldAlignment(), "backend restart with reused raw map ids still changes consumer epoch");
}
/** @brief Contradictory equal-time data and duplicate producer keys cannot become healthy proofs. */
void ContradictionAndBounds() {
    auto options = Options(); options.maximum_samples = 2;
    p::NativeAtlasLocalization source(options);
    Require(source.PushStatus(Status()) && source.PushOdometry(Pose()) &&
        source.SetWorldAlignment(Alignment(Pose()), 3.4), "initial bounded proof");
    auto contradictory = Pose(); contradictory.mutable_pose()->mutable_pose()->mutable_pose()->mutable_position()->set_x(2);
    Require(!source.PushOdometry(contradictory) && !source.PushOdometry(Pose()) &&
        !source.Lookup(p::SensorKind::kRgbd, "depth_optical", 3.25, 3.4), "contradictory identity cannot be revived by replay");
    auto duplicate = Status(350000000); *duplicate.mutable_status(0)->add_values() = duplicate.status(0).values(0);
    Require(!source.PushStatus(duplicate), "duplicate diagnostic key rejected");
    Require(source.PushStatus(Status(350000000)) && source.PushOdometry(Pose(350000000)) &&
        source.SetWorldAlignment(Alignment(Pose(350000000)), 3.4), "new true acquisition can establish explicit alignment");
    Require(source.PushStatus(Status(450000000, false)) && !source.LatestBackendPose(3.5),
        "latest unhealthy attempt cannot reuse the older healthy pose as current localization");
    Require(source.PushStatus(Status(550000000)) && source.PushOdometry(Pose(550000000)) &&
        !source.Lookup(p::SensorKind::kRgbd, "depth_optical", 3.35, 3.6), "bounded retention does not invent a pruned proof");
}

/** @brief Two distinct integer epochs cannot share one ambiguous double-clock acquisition proof. */
void DoubleClockAmbiguity() {
    p::NativeAtlasLocalization source(Options());
    auto first = Pose(788033485), second = Pose(788033486);
    auto first_status = Status(788033485), second_status = Status(788033486);
    for (auto* pose : {&first, &second}) {
        pose->mutable_header()->mutable_stamp()->set_sec(1795004291);
        *pose->mutable_pose()->mutable_pose()->mutable_header() = pose->header();
    }
    *first_status.mutable_header() = first.header(); *second_status.mutable_header() = second.header();
    const double acquisition = first.header().stamp().sec() + first.header().stamp().nanosec() * 1e-9;
    const double other = second.header().stamp().sec() + second.header().stamp().nanosec() * 1e-9;
    Require(acquisition == other, "test exercises actual epoch double-clock ambiguity");
    Require(source.PushStatus(first_status) && source.PushOdometry(first) &&
        source.SetWorldAlignment(Alignment(first), acquisition + .1) &&
        source.Lookup(p::SensorKind::kRgbd, "depth_optical", acquisition, acquisition + .1).has_value(),
        "one exact native acquisition is unambiguous before a colliding sample exists");
    Require(source.PushStatus(second_status) && source.PushOdometry(second) &&
        !source.Lookup(p::SensorKind::kRgbd, "depth_optical", acquisition, acquisition + .1),
        "different integer source stamps cannot silently stand for one another");
}
}  // namespace

/** @brief Run meaningful generated-message admission cases against the production receiver. */
int main() {
    try {
        AcquisitionAndCalibration(); CoordinateRevocation(); ContradictionAndBounds(); DoubleClockAmbiguity();
        std::cout << "PASS native_atlas_localization" << std::endl; return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL native_atlas_localization: " << error.what() << std::endl; return 1;
    }
}
