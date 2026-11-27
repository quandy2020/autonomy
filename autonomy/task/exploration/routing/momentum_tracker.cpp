/* Copyright 2026 The Openbot Authors. */

#include "autonomy/task/exploration/routing/momentum_tracker.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace autonomy::task::exploration {

MomentumTracker::MomentumTracker(MomentumTrackerOptions options)
    : options_(options) {
    if (!std::isfinite(options_.sampling_distance) ||
        options_.sampling_distance <= 0.0) {
        throw std::invalid_argument(
            "momentum sampling distance must be finite and positive");
    }
}

void MomentumTracker::Reset() noexcept {
    identity_.reset();
    sample_pose_.reset();
    movement_direction_.reset();
    latest_stamp_ = 0;
    active_ = false;
    reversal_count_ = 0;
    stable_count_ = 0;
}

bool MomentumTracker::SameIdentity(
    const MomentumSampleIdentity& identity) const noexcept {
    return identity_ && identity.session_id == identity_->session_id &&
           identity.map_epoch == identity_->map_epoch &&
           identity.localization_generation ==
               identity_->localization_generation &&
           identity.frame_id == identity_->frame_id;
}

MomentumState MomentumTracker::Update(
    const autonomy::navigation::Pose2d& measured_pose, double acquisition_stamp,
    const MomentumSampleIdentity& identity) noexcept {
    const bool valid =
        autonomy::navigation::IsFinite(measured_pose) &&
        std::isfinite(acquisition_stamp) && acquisition_stamp > 0 &&
        identity.session_id != 0 && identity.map_epoch != 0 &&
        identity.localization_generation != 0 && !identity.frame_id.empty();
    if (!valid) {
        Reset();
        return state();
    }

    if (!identity_) {
        identity_ = identity;
        sample_pose_ = measured_pose;
        latest_stamp_ = acquisition_stamp;
        return state();
    }
    if (!SameIdentity(identity) || acquisition_stamp <= latest_stamp_) {
        Reset();
        identity_ = identity;
        sample_pose_ = measured_pose;
        latest_stamp_ = acquisition_stamp;
        return state();
    }
    latest_stamp_ = acquisition_stamp;

    const Eigen::Vector2d displacement(measured_pose.x - sample_pose_->x,
                                       measured_pose.y - sample_pose_->y);
    const double distance = displacement.norm();
    if (!std::isfinite(distance)) {
        Reset();
        return state();
    }
    if (distance <= options_.sampling_distance) {
        return state();
    }

    const Eigen::Vector2d direction = displacement / distance;
    if (movement_direction_) {
        if (movement_direction_->dot(direction) < 0.0) {
            if (reversal_count_ < std::numeric_limits<unsigned>::max()) {
                ++reversal_count_;
            }
            stable_count_ = 0;
            if (reversal_count_ > options_.activation_count_threshold) {
                active_ = true;
            }
        } else {
            if (stable_count_ < std::numeric_limits<unsigned>::max()) {
                ++stable_count_;
            }
            if (stable_count_ > options_.release_count_threshold) {
                reversal_count_ = 0;
                active_ = false;
            }
        }
    }
    movement_direction_ = direction;
    sample_pose_ = measured_pose;
    return state();
}

MomentumState MomentumTracker::state() const noexcept {
    MomentumState result;
    result.active = active_;
    result.reversal_count = reversal_count_;
    result.stable_count = stable_count_;
    if (active_) {
        result.preferred_direction = movement_direction_;
    }
    return result;
}

}  // namespace autonomy::task::exploration
