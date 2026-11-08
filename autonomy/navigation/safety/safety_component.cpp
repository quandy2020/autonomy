/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

#include "navigation/safety/safety_component.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>

#include "automsgs/msgs/geometry_msgs/twist_stamped.pb.h"
#include "automsgs/msgs/nav_msgs/odometry.pb.h"
#include "automsgs/msgs/sensor_msgs/camera_info.pb.h"
#include "automsgs/msgs/sensor_msgs/image.pb.h"
#include "automsgs/msgs/time_utils.hpp"
#include "navigation/common/configuration_loader.hpp"
#include "navigation/common/twist2d_convert.hpp"

namespace navigation
{
namespace safety
{
namespace
{

void AddDepthObstacles(const automsgs::msgs::sensor_msgs::Image& image,
                       const automsgs::msgs::sensor_msgs::CameraInfo* camera_info,
                       ReflexObservation* observation)
{
    if (image.encoding() != "32FC1" || image.width() == 0 || image.height() == 0 ||
        image.data().size() < image.width() * image.height() * sizeof(float))
    {
        return;
    }
    const auto* pixels = reinterpret_cast<const float*>(image.data().data());
    const int row = static_cast<int>(image.height()) / 2;
    const double fx = camera_info != nullptr && camera_info->k_size() > 0
                          ? camera_info->k(0)
                          : 0.0;
    const double cx = camera_info != nullptr && camera_info->k_size() > 2
                          ? camera_info->k(2)
                          : static_cast<double>(image.width()) / 2.0;
    for (int column = 0; column < 5; ++column)
    {
        const int u = (image.width() * (2 * column + 1)) / 10;
        const double distance = pixels[static_cast<std::size_t>(row) * image.width() +
                                       static_cast<std::size_t>(u)];
        if (!std::isfinite(distance) || distance <= 0.1 || distance > 5.0)
        {
            continue;
        }
        auto* obstacle = observation->add_obstacles();
        obstacle->set_x(distance);
        obstacle->set_y(fx > 1.0 ? (static_cast<double>(u) - cx) * distance / fx : 0.0);
        obstacle->set_radius(0.1);
    }
}

double CenterDepth(const automsgs::msgs::sensor_msgs::Image& image)
{
    if (image.encoding() != "32FC1" || image.width() == 0 || image.height() == 0)
    {
        return 0.0;
    }
    const std::size_t index =
        static_cast<std::size_t>(image.height() / 2) * image.width() + image.width() / 2;
    if (image.data().size() < (index + 1U) * sizeof(float))
    {
        return 0.0;
    }
    const auto* pixels = reinterpret_cast<const float*>(image.data().data());
    return pixels[index];
}

}  // namespace

SafetyComponent::SafetyComponent() = default;

SafetyComponent::~SafetyComponent()
{
    Clear();
}

bool SafetyComponent::Init()
{
    config::SafetyConfig configuration;
    if (!common::LoadTextProto(ConfigFilePath(), &configuration).Ok() ||
        !shield_.Configure(configuration))
    {
        return false;
    }
    command_reader_ = node_->CreateReader<automsgs::msgs::geometry_msgs::Twist>(
        "/navigation/cmd_vel_raw");
    depth_reader_ =
        node_->CreateReader<automsgs::msgs::sensor_msgs::Image>("/camera/depth/image_raw");
    odometry_reader_ =
        node_->CreateReader<automsgs::msgs::nav_msgs::Odometry>("/odom");
    camera_info_reader_ =
        node_->CreateReader<automsgs::msgs::sensor_msgs::CameraInfo>(
            "/camera/camera_info");
    command_writer_ =
        node_->CreateWriter<automsgs::msgs::geometry_msgs::TwistStamped>("/cmd_vel");
    heartbeat_writer_ =
        node_->CreateWriter<lifecycle::Heartbeat>("/safety/heartbeat");
    return command_reader_ != nullptr && depth_reader_ != nullptr &&
           odometry_reader_ != nullptr && camera_info_reader_ != nullptr &&
           command_writer_ != nullptr && heartbeat_writer_ != nullptr;
}

bool SafetyComponent::Proc()
{
    const auto depth = depth_reader_->GetLatestObserved();
    std::int64_t now_ns = 0;
    if (depth != nullptr && depth->has_header())
    {
        now_ns = static_cast<std::int64_t>(
            automsgs::msgs::builtin_interfaces::TimeToNanoseconds(
                depth->header().stamp()));
    }
    if (now_ns <= 0)
    {
        now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                     std::chrono::steady_clock::now().time_since_epoch())
                     .count();
    }

    common::Twist2D candidate;
    if (const auto command = command_reader_->GetLatestObserved(); command != nullptr)
    {
        candidate = common::FromProto(*command);
    }
    common::Twist2D measured;
    if (const auto odometry = odometry_reader_->GetLatestObserved(); odometry != nullptr)
    {
        measured = common::FromProto(odometry->twist().twist());
    }

    ReflexObservation observation;
    observation.set_valid(true);
    observation.set_stamp_ns(now_ns);
    observation.set_frame("base_link");
    if (depth != nullptr)
    {
        const auto camera_info = camera_info_reader_->GetLatestObserved();
        AddDepthObstacles(*depth, camera_info.get(), &observation);
        if (observation.obstacles_size() == 0)
        {
            const double distance = CenterDepth(*depth);
            if (std::isfinite(distance) && distance > 0.0)
            {
                auto* obstacle = observation.add_obstacles();
                obstacle->set_x(distance);
                obstacle->set_radius(0.1);
            }
        }
    }
    const auto result = shield_.Process(candidate, observation, measured, now_ns, 0.02);
    automsgs::msgs::geometry_msgs::TwistStamped message;
    if (depth != nullptr)
    {
        *message.mutable_header() = depth->header();
    }
    else
    {
        *message.mutable_header()->mutable_stamp() =
            automsgs::msgs::builtin_interfaces::TimeFromNanoseconds(
                static_cast<std::uint64_t>(now_ns));
    }
    message.mutable_header()->set_frame_id("base_link");
    message.mutable_twist()->mutable_linear()->set_x(result.command.x());
    message.mutable_twist()->mutable_linear()->set_y(result.command.y());
    message.mutable_twist()->mutable_angular()->set_z(result.command.theta());
    (void)command_writer_->Write(message);

    lifecycle::Heartbeat heartbeat;
    heartbeat.set_module("safety");
    heartbeat.set_sequence(++heartbeat_sequence_);
    heartbeat.set_stamp_ns(now_ns);
    heartbeat.set_state(result.emergency ? lifecycle::MODULE_STATE_DEGRADED
                                         : lifecycle::MODULE_STATE_READY);
    (void)heartbeat_writer_->Write(heartbeat);
    return true;
}

void SafetyComponent::Clear()
{
    command_reader_.reset();
    depth_reader_.reset();
    odometry_reader_.reset();
    camera_info_reader_.reset();
    command_writer_.reset();
    heartbeat_writer_.reset();
}

}  // namespace safety
}  // namespace navigation

AUTOLINK_REGISTER_COMPONENT(navigation::safety::SafetyComponent)
