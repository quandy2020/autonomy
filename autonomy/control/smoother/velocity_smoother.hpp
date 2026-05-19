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
namespace smoother {

/**
 * @class VelocitySmoother
 * @brief This class that smooths cmd_vel velocities for robot bases
 */
class VelocitySmoother
{
public:
    /**
     * @brief A constructor for VelocitySmoother
     * @param options The options to use
     */
    explicit VelocitySmoother(const proto::VelocitySmootherOptions& options);

    /**
     * @brief Destructor for VelocitySmoother
     */
    ~VelocitySmoother();

    /**
     * @brief Find the scale factor, eta, which scales axis into acceleration
     * range
     * @param v_curr current velocity
     * @param v_cmd commanded velocity
     * @param accel maximum acceleration
     * @param decel maximum deceleration
     * @return Scale factor, eta
     */
    double findEtaConstraint(const double v_curr, const double v_cmd,
                             const double accel, const double decel);

    /**
     * @brief Apply acceleration and scale factor constraints
     * @param v_curr current velocity
     * @param v_cmd commanded velocity
     * @param accel maximum acceleration
     * @param decel maximum deceleration
     * @param eta Scale factor
     * @return Velocity command
     */
    double applyConstraints(const double v_curr, const double v_cmd,
                            const double accel, const double decel,
                            const double eta);

protected:
    /**
     * @brief Callback for incoming velocity commands
     * @param msg Twist message
     */
    void inputCommandCallback(
        const std::shared_ptr<commsgs::geometry_msgs::Twist>& msg);
    void inputCommandStampedCallback(
        const std::shared_ptr<commsgs::geometry_msgs::TwistStamped>& msg);

    /**
     * @brief Main worker timer function
     */
    void smootherTimer();

    // Network interfaces
    std::unique_ptr<utils::OdomSmoother> odom_smoother_;
    std::shared_ptr<commsgs::geometry_msgs::TwistStamped> last_cmd_;
    std::shared_ptr<commsgs::geometry_msgs::TwistStamped> command_;

    // Parameters
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

}  // namespace smoother
}  // namespace control
}  // namespace autonomy
