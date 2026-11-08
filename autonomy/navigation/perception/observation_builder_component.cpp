/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/perception/observation_builder_component.hpp"

#include "automsgs/msgs/map_msgs/occupancy_grid.pb.h"
#include "automsgs/msgs/nav_msgs/odometry.pb.h"
#include "automsgs/msgs/sensor_msgs/camera_info.pb.h"
#include "automsgs/msgs/sensor_msgs/imu.pb.h"

namespace navigation
{
namespace perception
{

ObservationBuilderComponent::ObservationBuilderComponent() = default;

ObservationBuilderComponent::~ObservationBuilderComponent()
{
    Clear();
}

bool ObservationBuilderComponent::Init()
{
    camera_info_reader_ =
        node_->CreateReader<automsgs::msgs::sensor_msgs::CameraInfo>(
            "/camera/camera_info");
    odometry_reader_ =
        node_->CreateReader<automsgs::msgs::nav_msgs::Odometry>("/odom");
    imu_reader_ =
        node_->CreateReader<automsgs::msgs::sensor_msgs::Imu>("/imu");
    map_reader_ =
        node_->CreateReader<automsgs::msgs::map_msgs::OccupancyGrid>("/map");
    observation_writer_ =
        node_->CreateWriter<observation::Observation>("/navigation/observation");
    return camera_info_reader_ != nullptr && odometry_reader_ != nullptr &&
           imu_reader_ != nullptr && map_reader_ != nullptr &&
           observation_writer_ != nullptr;
}

bool ObservationBuilderComponent::Proc(
    const std::shared_ptr<automsgs::msgs::sensor_msgs::Image>& depth)
{
    if (depth == nullptr || observation_writer_ == nullptr)
    {
        return false;
    }
    ObservationInputs inputs;
    inputs.depth = depth.get();
    if (const auto camera = camera_info_reader_->GetLatestObserved(); camera != nullptr)
    {
        inputs.camera_info = camera.get();
    }
    if (const auto odometry = odometry_reader_->GetLatestObserved(); odometry != nullptr)
    {
        inputs.odom = odometry.get();
        inputs.tf_valid = true;
    }
    if (const auto imu = imu_reader_->GetLatestObserved(); imu != nullptr)
    {
        inputs.imu = imu.get();
    }
    observation::Observation observation;
    if (!builder_.Build(inputs, &observation))
    {
        return false;
    }
    if (const auto map = map_reader_->GetLatestObserved(); map != nullptr)
    {
        *observation.mutable_map() = *map;
    }
    return observation_writer_->Write(observation);
}

void ObservationBuilderComponent::Clear()
{
    camera_info_reader_.reset();
    odometry_reader_.reset();
    imu_reader_.reset();
    map_reader_.reset();
    observation_writer_.reset();
    builder_.Reset();
}

}  // namespace perception
}  // namespace navigation

AUTOLINK_REGISTER_COMPONENT(navigation::perception::ObservationBuilderComponent)
