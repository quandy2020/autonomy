/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#pragma once

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/control/proto/smoother_options.pb.h"
#include "autonomy/control/utils/odometry_utils.hpp"

namespace autonomy {
namespace control {
namespace utils {

/**
 * @class VelocitySmoother
 * @brief Smooths cmd_vel velocities for robot bases
 */
class VelocitySmoother
{
public:
    explicit VelocitySmoother(const proto::VelocitySmootherOptions& options);

    ~VelocitySmoother();

    double findEtaConstraint(const double v_curr, const double v_cmd,
                             const double accel, const double decel);

    double applyConstraints(const double v_curr, const double v_cmd,
                            const double accel, const double decel,
                            const double eta);

protected:
    void inputCommandCallback(
        const std::shared_ptr<commsgs::geometry_msgs::Twist>& msg);
    void inputCommandStampedCallback(
        const std::shared_ptr<commsgs::geometry_msgs::TwistStamped>& msg);

    void smootherTimer();

    std::unique_ptr<OdomSmoother> odom_smoother_;
    std::shared_ptr<commsgs::geometry_msgs::TwistStamped> last_cmd_;
    std::shared_ptr<commsgs::geometry_msgs::TwistStamped> command_;

    double smoothing_frequency_;
    double odom_duration_;
    std::string odom_topic_;
    bool open_loop_;
    bool stopped_{true};
    bool scale_velocities_;
    std::vector<double> max_velocities_;
    std::vector<double> min_velocities_;
    std::vector<double> max_accels_;
    std::vector<double> max_decels_;
    std::vector<double> deadband_velocities_;
    commsgs::builtin_interfaces::Duration velocity_timeout_;
    commsgs::builtin_interfaces::Time last_command_time_;
};

}  // namespace utils
}  // namespace control
}  // namespace autonomy
