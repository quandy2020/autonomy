/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/policy_controller/action/safety_filter.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>

#include "autonomy/control/controller/policy_controller/action/space.hpp"
#include "autonomy/control/proto/controller_options.pb.h"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/footprint_collision_checker.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {
namespace {

double Clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(upper, value));
}

double RateLimit(double target, double previous, double max_acceleration,
                 double elapsed_seconds) {
    if (max_acceleration <= 0.0 || elapsed_seconds <= 0.0) {
        return target;
    }
    const double max_delta = max_acceleration * elapsed_seconds;
    return Clamp(target, previous - max_delta, previous + max_delta);
}

bool FilterAxis(double raw, double previous, bool has_previous, double scale,
                double max_velocity, double max_acceleration,
                double elapsed_seconds, double* out) {
    double value = raw * scale;
    bool modified = false;
    if (max_velocity > 0.0) {
        const double clamped = Clamp(value, -max_velocity, max_velocity);
        modified = modified || (clamped != value);
        value = clamped;
    }
    if (has_previous) {
        const double limited =
            RateLimit(value, previous, max_acceleration, elapsed_seconds);
        modified = modified || (limited != value);
        value = limited;
    }
    *out = value;
    return modified;
}

double YawFromQuaternion(
    const automsgs::msgs::geometry_msgs::Quaternion& orientation) {
    return std::atan2(
        2.0 * (orientation.w() * orientation.z() +
               orientation.x() * orientation.y()),
        1.0 - 2.0 * (orientation.y() * orientation.y() +
                     orientation.z() * orientation.z()));
}

bool FootprintOrPointBlocked(
    map::costmap_2d::Costmap2DWrapper* costmap_wrapper, double x, double y,
    double yaw, bool use_footprint, double cost_threshold) {
    auto* costmap = costmap_wrapper->getCostmap();
    std::lock_guard<::autonomy::map::costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
    map::costmap_2d::FootprintCollisionChecker<map::costmap_2d::Costmap2D*>
        checker(costmap);

    if (use_footprint) {
        const auto footprint = costmap_wrapper->getRobotFootprint();
        if (!footprint.empty()) {
            const double cost =
                checker.footprintCostAtPose(x, y, yaw, footprint);
            return cost >= cost_threshold;
        }
    }

    unsigned int map_x = 0;
    unsigned int map_y = 0;
    if (!checker.worldToMap(x, y, map_x, map_y)) {
        return true;
    }
    return static_cast<double>(checker.pointCost(map_x, map_y)) >=
           cost_threshold;
}

}  // namespace

void CommandSafetyFilter::Configure(
    const proto::PolicyControllerOptions& options,
    const proto::TwistLimits& limits) {
    std::lock_guard<std::mutex> lock(mutex_);
    limits_ = limits;
    if (limits_.max_vx() <= 0.0) {
        limits_.set_max_vx(0.5);
    }
    if (limits_.max_wz() <= 0.0) {
        limits_.set_max_wz(1.0);
    }
    hold_last_command_on_failure_ =
        options.has_hold_on_failure() ? options.hold_on_failure() : true;
    enable_collision_check_ = options.collision_check();
    use_footprint_collision_ = options.use_footprint_collision();
    collision_check_distance_ = options.collision_distance() > 0.0
                                    ? options.collision_distance()
                                    : 0.5;
    collision_cost_threshold_ =
        options.collision_cost_threshold() > 0.0
            ? options.collision_cost_threshold()
            : static_cast<double>(map::costmap_2d::LETHAL_OBSTACLE);
    // Keep speed_scale_ across Configure (hot-swap).
}

void CommandSafetyFilter::SetSpeedScale(double scale) {
    std::lock_guard<std::mutex> lock(mutex_);
    speed_scale_ = std::max(0.0, scale);
}

void CommandSafetyFilter::Reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    last_command_.reset();
}

bool CommandSafetyFilter::MotionCollisionBlocked(
    const ObservationSnapshot& snapshot,
    map::costmap_2d::Costmap2DWrapper* costmap_wrapper, double linear_x,
    double linear_y, double angular_z) const {
    if (!enable_collision_check_ || costmap_wrapper == nullptr ||
        costmap_wrapper->getCostmap() == nullptr ||
        !snapshot.pose.has_value()) {
        return false;
    }
    const double speed = std::hypot(linear_x, linear_y);
    const double x0 = snapshot.pose->pose().position().x();
    const double y0 = snapshot.pose->pose().position().y();
    const double yaw0 = YawFromQuaternion(snapshot.pose->pose().orientation());

    // Pure rotation (or near-zero translation): probe footprint at current
    // pose and at a small yaw step in the commanded turn direction.
    if (speed <= 0.05) {
        if (std::fabs(angular_z) <= 0.05) {
            return false;
        }
        const double yaw_step =
            std::copysign(std::min(0.35, collision_check_distance_), angular_z);
        if (FootprintOrPointBlocked(costmap_wrapper, x0, y0, yaw0,
                                    use_footprint_collision_,
                                    collision_cost_threshold_)) {
            return true;
        }
        return FootprintOrPointBlocked(costmap_wrapper, x0, y0, yaw0 + yaw_step,
                                       use_footprint_collision_,
                                       collision_cost_threshold_);
    }

    const double dir_x =
        (std::cos(yaw0) * linear_x - std::sin(yaw0) * linear_y) / speed;
    const double dir_y =
        (std::sin(yaw0) * linear_x + std::cos(yaw0) * linear_y) / speed;
    const double x = x0 + dir_x * collision_check_distance_;
    const double y = y0 + dir_y * collision_check_distance_;
    const double probe_yaw = std::atan2(dir_y, dir_x);
    return FootprintOrPointBlocked(costmap_wrapper, x, y, probe_yaw,
                                   use_footprint_collision_,
                                   collision_cost_threshold_);
}

uint32_t CommandSafetyFilter::Apply(
    const automsgs::msgs::geometry_msgs::Twist& action,
    const ObservationSnapshot& snapshot,
    map::costmap_2d::Costmap2DWrapper* costmap_wrapper, double elapsed_seconds,
    automsgs::msgs::geometry_msgs::TwistStamped* command_velocity,
    std::string* message, bool* clamp_or_rate_limit,
    bool* collision_blocked) {
    if (clamp_or_rate_limit != nullptr) {
        *clamp_or_rate_limit = false;
    }
    if (collision_blocked != nullptr) {
        *collision_blocked = false;
    }
    if (command_velocity == nullptr) {
        return ApplyFailure("Invalid action", command_velocity, message);
    }

    double speed_scale = 1.0;
    proto::TwistLimits limits;
    bool enable_collision = false;
    bool has_previous = false;
    automsgs::msgs::geometry_msgs::Twist previous;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        speed_scale = speed_scale_;
        limits = limits_;
        enable_collision = enable_collision_check_;
        has_previous = last_command_.has_value();
        if (has_previous) {
            previous = *last_command_;
        }
    }

    double linear_x = 0.0;
    double linear_y = 0.0;
    double linear_z = 0.0;
    double angular_x = 0.0;
    double angular_y = 0.0;
    double angular_z = 0.0;
    bool modified = false;
    modified |= FilterAxis(action.linear().x(), previous.linear().x(),
                           has_previous, speed_scale, limits.max_vx(),
                           limits.max_ax(), elapsed_seconds, &linear_x);
    modified |= FilterAxis(action.linear().y(), previous.linear().y(),
                           has_previous, speed_scale, limits.max_vy(),
                           limits.max_ay(), elapsed_seconds, &linear_y);
    modified |= FilterAxis(action.linear().z(), previous.linear().z(),
                           has_previous, speed_scale, limits.max_vz(),
                           limits.max_az(), elapsed_seconds, &linear_z);
    modified |= FilterAxis(action.angular().x(), previous.angular().x(),
                           has_previous, speed_scale, limits.max_wx(),
                           limits.max_alpha_x(), elapsed_seconds, &angular_x);
    modified |= FilterAxis(action.angular().y(), previous.angular().y(),
                           has_previous, speed_scale, limits.max_wy(),
                           limits.max_alpha_y(), elapsed_seconds, &angular_y);
    modified |= FilterAxis(action.angular().z(), previous.angular().z(),
                           has_previous, speed_scale, limits.max_wz(),
                           limits.max_alpha_z(), elapsed_seconds, &angular_z);
    if (clamp_or_rate_limit != nullptr) {
        *clamp_or_rate_limit = modified;
    }

    if (enable_collision &&
        MotionCollisionBlocked(snapshot, costmap_wrapper, linear_x, linear_y,
                               angular_z)) {
        if (collision_blocked != nullptr) {
            *collision_blocked = true;
        }
        if (message != nullptr) {
            *message = "Footprint/motion collision check blocked command";
        }
        std::lock_guard<std::mutex> lock(mutex_);
        automsgs::msgs::geometry_msgs::Twist zero;
        last_command_ = zero;
        WriteTwistStamped(*last_command_, command_velocity);
        return static_cast<uint32_t>(proto::CONTROLLER_RESULT_COLLISION);
    }

    automsgs::msgs::geometry_msgs::Twist filtered;
    filtered.mutable_linear()->set_x(linear_x);
    filtered.mutable_linear()->set_y(linear_y);
    filtered.mutable_linear()->set_z(linear_z);
    filtered.mutable_angular()->set_x(angular_x);
    filtered.mutable_angular()->set_y(angular_y);
    filtered.mutable_angular()->set_z(angular_z);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_command_ = filtered;
    }
    WriteTwistStamped(filtered, command_velocity);
    if (message != nullptr) {
        *message = "ok";
    }
    return static_cast<uint32_t>(proto::CONTROLLER_RESULT_SUCCESS);
}

uint32_t CommandSafetyFilter::ApplyFailure(
    const std::string& reason,
    automsgs::msgs::geometry_msgs::TwistStamped* command_velocity,
    std::string* message, bool force_zero) {
    if (message != nullptr) {
        *message = reason;
    }
    if (command_velocity == nullptr) {
        return static_cast<uint32_t>(proto::CONTROLLER_RESULT_NO_VALID_CMD);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (force_zero) {
        automsgs::msgs::geometry_msgs::Twist zero;
        last_command_ = zero;
        WriteTwistStamped(zero, command_velocity);
    } else if (hold_last_command_on_failure_ && last_command_.has_value()) {
        WriteTwistStamped(*last_command_, command_velocity);
    } else {
        WriteTwistStamped(automsgs::msgs::geometry_msgs::Twist{},
                          command_velocity);
    }
    return static_cast<uint32_t>(proto::CONTROLLER_RESULT_NO_VALID_CMD);
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
