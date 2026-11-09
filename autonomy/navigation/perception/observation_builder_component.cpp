/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/perception/observation_builder_component.hpp"

#include "automsgs/msgs/map_msgs/occupancy_grid.pb.h"
#include "automsgs/msgs/nav_msgs/odometry.pb.h"
#include "automsgs/msgs/sensor_msgs/camera_info.pb.h"
#include "automsgs/msgs/sensor_msgs/imu.pb.h"
#include "automsgs/msgs/tf2_msgs/tf_message.pb.h"

namespace navigation {
namespace perception {

ObservationBuilderComponent::ObservationBuilderComponent() = default;

ObservationBuilderComponent::~ObservationBuilderComponent() {
    Clear();
}

bool ObservationBuilderComponent::Init() {
    camera_info_reader_ =
        node_->CreateReader<automsgs::msgs::sensor_msgs::CameraInfo>(
            "/camera/camera_info");
    odometry_reader_ =
        node_->CreateReader<automsgs::msgs::nav_msgs::Odometry>("/odom");
    imu_reader_ = node_->CreateReader<automsgs::msgs::sensor_msgs::Imu>("/imu");
    map_reader_ =
        node_->CreateReader<automsgs::msgs::map_msgs::OccupancyGrid>("/map");
    tf_reader_ =
        node_->CreateReader<automsgs::msgs::tf2_msgs::TFMessage>("/tf");
    observation_writer_ = node_->CreateWriter<observation::Observation>(
        "/navigation/observation");
    return camera_info_reader_ != nullptr && odometry_reader_ != nullptr &&
           imu_reader_ != nullptr && map_reader_ != nullptr &&
           tf_reader_ != nullptr && observation_writer_ != nullptr;
}

bool ObservationBuilderComponent::Proc(
    const std::shared_ptr<automsgs::msgs::sensor_msgs::Image>& depth) {
    if (depth == nullptr || observation_writer_ == nullptr) {
        return false;
    }
    ObservationInputs inputs;
    inputs.depth = depth.get();
    camera_info_reader_->Observe();
    odometry_reader_->Observe();
    imu_reader_->Observe();
    map_reader_->Observe();
    tf_reader_->Observe();
    const auto camera = camera_info_reader_->GetLatestObserved();
    const auto odometry = odometry_reader_->GetLatestObserved();
    const auto imu = imu_reader_->GetLatestObserved();
    if (camera != nullptr) {
        inputs.camera_info = camera.get();
    }
    if (const auto tf = tf_reader_->GetLatestObserved(); tf != nullptr) {
        for (const auto& transform : tf->transforms()) {
            if (transform.header().frame_id() == "map" &&
                transform.child_frame_id() == "odom") {
                map_to_odom_ = transform;
            }
        }
    }
    if (odometry != nullptr) {
        inputs.odom = odometry.get();
        if (map_to_odom_.has_value()) {
            inputs.map_to_odom = &*map_to_odom_;
            inputs.tf_valid = true;
        }
    }
    if (imu != nullptr) {
        inputs.imu = imu.get();
    }
    const auto map = map_reader_->GetLatestObserved();
    inputs.map = map.get();
    observation::Observation observation;
    if (!builder_.Build(inputs, &observation)) {
        return false;
    }
    return observation_writer_->Write(observation);
}

void ObservationBuilderComponent::Clear() {
    if (camera_info_reader_ != nullptr)
        camera_info_reader_->Shutdown();
    if (odometry_reader_ != nullptr)
        odometry_reader_->Shutdown();
    if (imu_reader_ != nullptr)
        imu_reader_->Shutdown();
    if (map_reader_ != nullptr)
        map_reader_->Shutdown();
    if (tf_reader_ != nullptr)
        tf_reader_->Shutdown();
    camera_info_reader_.reset();
    odometry_reader_.reset();
    imu_reader_.reset();
    map_reader_.reset();
    tf_reader_.reset();
    map_to_odom_.reset();
    observation_writer_.reset();
    builder_.Reset();
}

}  // namespace perception
}  // namespace navigation

AUTOLINK_REGISTER_COMPONENT(navigation::perception::ObservationBuilderComponent)
