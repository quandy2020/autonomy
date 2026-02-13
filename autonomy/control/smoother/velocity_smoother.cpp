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

#include "autonomy/control/smoother/velocity_smoother.hpp"
#include "autolink/timer/timer.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace control {
namespace smoother {

using Time = commsgs::builtin_interfaces::Time;
using Duration = commsgs::builtin_interfaces::Duration;

VelocitySmoother::VelocitySmoother(const std::shared_ptr<::autolink::Node>& node,
                                   const proto::VelocitySmootherOptions& options)
    : node_(node), last_command_time_(Time::Now()) {
    AINFO << "Configuring velocity smoother";

    // Get parameters from options proto
    smoothing_frequency_ = options.smoothing_frequency() > 0.0 ? options.smoothing_frequency() : 20.0;
    scale_velocities_ = options.scale_velocities();

    // Kinematics - all vectors represent (x, y, theta)
    max_velocities_ = options.max_velocity_size() == 3
                          ? std::vector<double>(options.max_velocity().begin(), options.max_velocity().end())
                          : std::vector<double>{0.50, 0.0, 2.5};
    min_velocities_ = options.min_velocity_size() == 3
                          ? std::vector<double>(options.min_velocity().begin(), options.min_velocity().end())
                          : std::vector<double>{-0.50, 0.0, -2.5};
    max_accels_ = options.max_accel_size() == 3
                      ? std::vector<double>(options.max_accel().begin(), options.max_accel().end())
                      : std::vector<double>{2.5, 0.0, 3.2};
    max_decels_ = options.max_decel_size() == 3
                      ? std::vector<double>(options.max_decel().begin(), options.max_decel().end())
                      : std::vector<double>{-2.5, 0.0, -3.2};

    // Validate kinematics parameters
    for (unsigned int i = 0; i != 3; i++) {
        if (max_decels_[i] > 0.0) {
            AERROR << "Positive values set of deceleration! These should be negative to slow down!";
            return;
        }
        if (max_accels_[i] < 0.0) {
            AERROR << "Negative values set of acceleration! These should be positive to speed up!";
            return;
        }
        if (min_velocities_[i] > 0.0) {
            AERROR << "Positive values set of min_velocities! These should be negative!";
            return;
        }
        if (max_velocities_[i] < 0.0) {
            AERROR << "Negative values set of max_velocities! These should be positive!";
            return;
        }
        if (min_velocities_[i] > max_velocities_[i]) {
            AERROR << "Min velocities are higher than max velocities!";
            return;
        }
    }

    // Get feature parameters
    odom_topic_ = options.odom_topic().empty() ? "odom" : options.odom_topic();
    odom_duration_ = options.odom_duration() > 0.0 ? options.odom_duration() : 0.1;
    deadband_velocities_ =
        options.deadband_velocity_size() == 3
            ? std::vector<double>(options.deadband_velocity().begin(), options.deadband_velocity().end())
            : std::vector<double>{0.0, 0.0, 0.0};
    double velocity_timeout_dbl = options.velocity_timeout() > 0.0 ? options.velocity_timeout() : 1.0;
    velocity_timeout_ = Duration::FromSeconds(velocity_timeout_dbl);

    if (max_velocities_.size() != 3 || min_velocities_.size() != 3 || max_accels_.size() != 3 ||
        max_decels_.size() != 3 || deadband_velocities_.size() != 3) {
        AERROR << "Invalid setting of kinematic and/or deadband limits!"
               << " All limits must be size of 3 representing (x, y, theta).";
        return;
    }

    // Get control type
    std::string feedback = options.feedback().empty() ? "OPEN_LOOP" : options.feedback();
    if (feedback == "OPEN_LOOP") {
        open_loop_ = true;
    } else if (feedback == "CLOSED_LOOP") {
        open_loop_ = false;
        odom_smoother_ = std::make_unique<utils::OdomSmoother>(node, odom_duration_, odom_topic_);
    } else {
        AERROR << "Invalid feedback_type, options are OPEN_LOOP and CLOSED_LOOP.";
        return;
    }

    // Setup inputs / outputs
    smoothed_cmd_pub_ = node->CreateWriter<commsgs::geometry_msgs::TwistStamped>("cmd_vel_smoothed");

    // Create reader with callback for TwistStamped messages
    auto callback = [this](const std::shared_ptr<commsgs::geometry_msgs::TwistStamped>& msg) {
        this->inputCommandStampedCallback(msg);
    };
    cmd_sub_ = node->CreateReader<commsgs::geometry_msgs::TwistStamped>("cmd_vel", callback);

    // Note: Real-time priority setting is not available in autolink
    // Skipping use_realtime_priority option

    // Create timer
    uint32_t timer_duration_ms = static_cast<uint32_t>(1000.0 / smoothing_frequency_);
    auto timer_callback = [this]() { this->smootherTimer(); };
    timer_ = std::make_shared<::autolink::Timer>(timer_duration_ms, timer_callback, false);
    timer_->Start();
}

VelocitySmoother::~VelocitySmoother() {
    if (timer_) {
        timer_->Stop();
        timer_.reset();
    }
}

void VelocitySmoother::inputCommandStampedCallback(const std::shared_ptr<commsgs::geometry_msgs::TwistStamped>& msg) {
    // If message contains NaN or Inf, ignore
    auto isFinite = [](float val) { return std::isfinite(val); };
    if (!isFinite(msg->twist.linear.x) || !isFinite(msg->twist.linear.y) || !isFinite(msg->twist.linear.z) ||
        !isFinite(msg->twist.angular.x) || !isFinite(msg->twist.angular.y) || !isFinite(msg->twist.angular.z)) {
        AERROR << "Velocity message contains NaNs or Infs! Ignoring as invalid!";
        return;
    }

    command_ = msg;
    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
        last_command_time_ = Time::Now();
    } else {
        last_command_time_ = msg->header.stamp;
    }
}

void VelocitySmoother::inputCommandCallback(const std::shared_ptr<commsgs::geometry_msgs::Twist>& msg) {
    auto twist_stamped = std::make_shared<commsgs::geometry_msgs::TwistStamped>();
    twist_stamped->twist = *msg;
    inputCommandStampedCallback(twist_stamped);
}

double VelocitySmoother::findEtaConstraint(const double v_curr, const double v_cmd, const double accel,
                                           const double decel) {
    // Exploiting vector scaling properties
    double dv = v_cmd - v_curr;

    double v_component_max;
    double v_component_min;

    // Accelerating if magnitude of v_cmd is above magnitude of v_curr
    // and if v_cmd and v_curr have the same sign (i.e. speed is NOT passing through 0.0)
    // Decelerating otherwise
    if (abs(v_cmd) >= abs(v_curr) && v_curr * v_cmd >= 0.0) {
        v_component_max = accel / smoothing_frequency_;
        v_component_min = -accel / smoothing_frequency_;
    } else {
        v_component_max = -decel / smoothing_frequency_;
        v_component_min = decel / smoothing_frequency_;
    }

    if (dv > v_component_max) {
        return v_component_max / dv;
    }

    if (dv < v_component_min) {
        return v_component_min / dv;
    }

    return -1.0;
}

double VelocitySmoother::applyConstraints(const double v_curr, const double v_cmd, const double accel,
                                          const double decel, const double eta) {
    double dv = v_cmd - v_curr;

    double v_component_max;
    double v_component_min;

    // Accelerating if magnitude of v_cmd is above magnitude of v_curr
    // and if v_cmd and v_curr have the same sign (i.e. speed is NOT passing through 0.0)
    // Decelerating otherwise
    if (abs(v_cmd) >= abs(v_curr) && v_curr * v_cmd >= 0.0) {
        v_component_max = accel / smoothing_frequency_;
        v_component_min = -accel / smoothing_frequency_;
    } else {
        v_component_max = -decel / smoothing_frequency_;
        v_component_min = decel / smoothing_frequency_;
    }

    return v_curr + std::clamp(eta * dv, v_component_min, v_component_max);
}

void VelocitySmoother::smootherTimer() {
    // Wait until the first command is received
    if (!command_) {
        return;
    }

    auto cmd_vel = std::make_unique<commsgs::geometry_msgs::TwistStamped>();
    cmd_vel->header = command_->header;

    // Check for velocity timeout. If nothing received, publish zeros to apply deceleration
    Time current_time = Time::Now();
    Duration elapsed = current_time - last_command_time_;
    if (elapsed.Seconds() > velocity_timeout_.Seconds()) {
        if (!last_cmd_ || stopped_) {
            stopped_ = true;
            return;
        }
        command_ = std::make_shared<commsgs::geometry_msgs::TwistStamped>();
        command_->header.stamp = current_time;
    }

    stopped_ = false;

    // Get current velocity based on feedback type
    commsgs::geometry_msgs::TwistStamped current_;
    if (open_loop_) {
        if (last_cmd_) {
            current_ = *last_cmd_;
        } else {
            current_ = commsgs::geometry_msgs::TwistStamped();
        }
    } else {
        current_ = odom_smoother_->getTwistStamped();
    }

    // Apply absolute velocity restrictions to the command
    command_->twist.linear.x =
        std::clamp(static_cast<double>(command_->twist.linear.x), min_velocities_[0], max_velocities_[0]);
    command_->twist.linear.y =
        std::clamp(static_cast<double>(command_->twist.linear.y), min_velocities_[1], max_velocities_[1]);
    command_->twist.angular.z =
        std::clamp(static_cast<double>(command_->twist.angular.z), min_velocities_[2], max_velocities_[2]);

    // Find if any component is not within the acceleration constraints. If so, store the most
    // significant scale factor to apply to the vector <dvx, dvy, dvw>, eta, to reduce all axes
    // proportionally to follow the same direction, within change of velocity bounds.
    // In case eta reduces another axis out of its own limit, apply accel constraint to guarantee
    // output is within limits, even if it deviates from requested command slightly.
    double eta = 1.0;
    if (scale_velocities_) {
        double curr_eta = -1.0;

        curr_eta = findEtaConstraint(current_.twist.linear.x, command_->twist.linear.x, max_accels_[0], max_decels_[0]);
        if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
            eta = curr_eta;
        }

        curr_eta = findEtaConstraint(current_.twist.linear.y, command_->twist.linear.y, max_accels_[1], max_decels_[1]);
        if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
            eta = curr_eta;
        }

        curr_eta =
            findEtaConstraint(current_.twist.angular.z, command_->twist.angular.z, max_accels_[2], max_decels_[2]);
        if (curr_eta > 0.0 && std::fabs(1.0 - curr_eta) > std::fabs(1.0 - eta)) {
            eta = curr_eta;
        }
    }

    cmd_vel->twist.linear.x =
        applyConstraints(current_.twist.linear.x, command_->twist.linear.x, max_accels_[0], max_decels_[0], eta);
    cmd_vel->twist.linear.y =
        applyConstraints(current_.twist.linear.y, command_->twist.linear.y, max_accels_[1], max_decels_[1], eta);
    cmd_vel->twist.angular.z =
        applyConstraints(current_.twist.angular.z, command_->twist.angular.z, max_accels_[2], max_decels_[2], eta);
    last_cmd_ = std::make_shared<commsgs::geometry_msgs::TwistStamped>(*cmd_vel);

    // Apply deadband restrictions & publish
    cmd_vel->twist.linear.x = fabs(cmd_vel->twist.linear.x) < deadband_velocities_[0] ? 0.0 : cmd_vel->twist.linear.x;
    cmd_vel->twist.linear.y = fabs(cmd_vel->twist.linear.y) < deadband_velocities_[1] ? 0.0 : cmd_vel->twist.linear.y;
    cmd_vel->twist.angular.z =
        fabs(cmd_vel->twist.angular.z) < deadband_velocities_[2] ? 0.0 : cmd_vel->twist.angular.z;

    smoothed_cmd_pub_->Write(*cmd_vel);
}

}  // namespace smoother
}  // namespace control
}  // namespace autonomy
