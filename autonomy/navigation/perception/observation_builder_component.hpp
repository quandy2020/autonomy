/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

/**
 * @file observation_builder_component.hpp
 * @brief autolink component fusing depth/odom/imu/map into Observation.
 */

#pragma once

#include <memory>

#include "autolink/component/component.hpp"
#include "autonomy/common/macros.hpp"
#include "navigation/perception/observation_builder.hpp"

namespace navigation
{
namespace perception
{

/**
 * @brief Message-driven fusion component.
 *
 * @details Triggered by depth; polls the latest camera_info, odom, imu and map
 * and publishes `/navigation/observation`.
 */
class ObservationBuilderComponent final
    : public autolink::Component<automsgs::msgs::sensor_msgs::Image>
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS_NOT_COPYABLE(ObservationBuilderComponent)

    /** @brief Construct an uninitialized component. */
    ObservationBuilderComponent();

    /** @brief Release readers/writer. */
    ~ObservationBuilderComponent() override;

protected:
    /** @brief Create readers/writer from the DAG. */
    bool Init() override;

    /** @brief Release endpoints. */
    void Clear() override;

private:
    /** @brief Build and publish one fused Observation. */
    bool Proc(const std::shared_ptr<automsgs::msgs::sensor_msgs::Image>& depth) override;

    /** @brief Fusion helper. */
    ObservationBuilder builder_;
    /** @brief Camera calibration reader. */
    std::shared_ptr<autolink::Reader<automsgs::msgs::sensor_msgs::CameraInfo>>
        camera_info_reader_{nullptr};
    /** @brief Odometry reader. */
    std::shared_ptr<autolink::Reader<automsgs::msgs::nav_msgs::Odometry>>
        odometry_reader_{nullptr};
    /** @brief IMU reader. */
    std::shared_ptr<autolink::Reader<automsgs::msgs::sensor_msgs::Imu>>
        imu_reader_{nullptr};
    /** @brief Occupancy grid reader. */
    std::shared_ptr<autolink::Reader<automsgs::msgs::map_msgs::OccupancyGrid>>
        map_reader_{nullptr};
    /** @brief /navigation/observation writer. */
    std::shared_ptr<autolink::Writer<observation::Observation>>
        observation_writer_{nullptr};
};

}  // namespace perception
}  // namespace navigation
