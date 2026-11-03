/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_TRACKING_RECOVERY_POLICY_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_TRACKING_RECOVERY_POLICY_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace autonomy::localization::atlas::internal {

inline constexpr int kImuRgbdRecoveryCovisibilityCandidateLimit = 5;
inline constexpr int kImuRgbdRecoveryReferenceCandidateLimit = 7;
inline constexpr double kImuRgbdVisualRecoveryTimeoutSeconds = 1.0;
inline constexpr int kRgbdTemporalFallbackMinimumGridCells = 4;
inline constexpr int kRgbdTemporalStandardMinimumGridCells = 6;
inline constexpr int kRgbdTemporalLocalMapProjectionRadius = 3;
inline constexpr double kRgbdTemporalMapReprojectionLimitPixels = 3.0;
inline constexpr double kRgbdTemporalMapXyzRelativeTolerance = 0.03;
inline constexpr double kRgbdTemporalMapXyzMinimumToleranceMeters = 0.10;
inline constexpr double kRgbdTemporalMapXyzMaximumToleranceMeters = 0.25;

enum class LocalMapCommitMode : std::uint8_t {
    kCommit,
    kTransactional,
};

struct RgbdTemporalSeedIdentity {
    std::uint64_t reference_frame_id = 0;
    std::uint64_t current_frame_id = 0;
    std::uint64_t reference_map_id = 0;
    std::uint64_t current_map_id = 0;
    std::uint64_t reference_coordinate_epoch = 0;
    std::uint64_t current_coordinate_epoch = 0;
    int reference_map_revision = -1;
    int current_map_revision = -1;
};

struct RgbdTemporalSeedContext {
    bool last_frame_set = false;
    std::uint64_t last_frame_id = 0;
    std::uint64_t current_frame_id = 0;
    std::uint64_t map_id = 0;
    std::uint64_t coordinate_epoch = 0;
    int map_revision = -1;
};

constexpr bool HasMatchingRgbdTemporalSeedIdentity(
    const RgbdTemporalSeedIdentity& identity,
    const RgbdTemporalSeedContext& context) {
    return context.last_frame_set &&
           identity.reference_frame_id == context.last_frame_id &&
           identity.current_frame_id == context.current_frame_id &&
           identity.reference_map_id == context.map_id &&
           identity.current_map_id == context.map_id &&
           identity.reference_coordinate_epoch == context.coordinate_epoch &&
           identity.current_coordinate_epoch == context.coordinate_epoch &&
           identity.reference_map_revision == context.map_revision &&
           identity.current_map_revision == context.map_revision;
}

constexpr int RequiredRgbdTemporalReanchorInliers(bool imu_initialized) {
    return imu_initialized ? 15 : 50;
}
constexpr bool HasEnoughRgbdTemporalReanchorInliers(bool imu_initialized,
                                                    int inliers) {
    return inliers >= RequiredRgbdTemporalReanchorInliers(imu_initialized);
}

struct BoundedRgbdTemporalOdometryState {
    static constexpr double kMaximumDurationSeconds = 1.0;
    static constexpr double kMaximumReferenceStepSeconds = 0.25;
    static constexpr int kMaximumConsecutiveLowCoverageSteps = 1;
    static constexpr int kMaximumTotalLowCoverageSteps = 2;

    bool active = false;
    double start_timestamp = 0.0;
    double reference_timestamp = 0.0;
    std::uint64_t map_id = 0;
    std::uint64_t coordinate_epoch = 0;
    int map_revision = -1;
    int consecutive_low_coverage_steps = 0;
    int total_low_coverage_steps = 0;

    void Start(double recovery_start_timestamp,
               double initial_reference_timestamp, std::uint64_t current_map_id,
               std::uint64_t current_coordinate_epoch,
               int current_map_revision) {
        active = true;
        start_timestamp = recovery_start_timestamp;
        reference_timestamp = initial_reference_timestamp;
        map_id = current_map_id;
        coordinate_epoch = current_coordinate_epoch;
        map_revision = current_map_revision;
        consecutive_low_coverage_steps = 0;
        total_low_coverage_steps = 0;
    }

    bool CanEstimate(double current_timestamp, std::uint64_t current_map_id,
                     std::uint64_t current_coordinate_epoch,
                     int current_map_revision) const {
        return active && map_id == current_map_id &&
               coordinate_epoch == current_coordinate_epoch &&
               map_revision == current_map_revision &&
               current_timestamp >= reference_timestamp &&
               current_timestamp - reference_timestamp <=
                   kMaximumReferenceStepSeconds &&
               current_timestamp - start_timestamp <= kMaximumDurationSeconds;
    }

    bool CanAccept(bool low_coverage) const {
        return active &&
               (!low_coverage ||
                (consecutive_low_coverage_steps <
                     kMaximumConsecutiveLowCoverageSteps &&
                 total_low_coverage_steps < kMaximumTotalLowCoverageSteps));
    }

    void Advance(double accepted_timestamp, bool low_coverage) {
        reference_timestamp = accepted_timestamp;
        if (low_coverage) {
            ++consecutive_low_coverage_steps;
            ++total_low_coverage_steps;
        } else {
            consecutive_low_coverage_steps = 0;
        }
    }

    bool RebaseMapRevision(std::uint64_t expected_map_id,
                           std::uint64_t expected_coordinate_epoch,
                           int expected_map_revision, int new_map_revision) {
        if (!active || map_id != expected_map_id ||
            coordinate_epoch != expected_coordinate_epoch ||
            map_revision != expected_map_revision ||
            expected_map_revision < 0 ||
            new_map_revision <= expected_map_revision) {
            return false;
        }
        map_revision = new_map_revision;
        return true;
    }

    void Reset() {
        *this = BoundedRgbdTemporalOdometryState();
    }
};

struct TrajectoryRecordDecision {
    double timestamp;
    bool reuse_previous_pose;
    bool lost;
};

constexpr TrajectoryRecordDecision MakeTrajectoryRecordDecision(
    double current_frame_timestamp, bool current_frame_has_pose,
    bool mark_current_frame_lost) {
    // A reused relative pose is only a storage placeholder. Marking it lost
    // prevents trajectory exporters from emitting a duplicate pose as though
    // it were a valid estimate for this frame.
    return {
        current_frame_timestamp,
        !current_frame_has_pose,
        mark_current_frame_lost || !current_frame_has_pose,
    };
}

constexpr bool ShouldUseImuPredictionResult(bool is_imu_rgbd) {
    return is_imu_rgbd;
}

constexpr bool HasEnoughMotionModelPoseSeedMatches(int projected_matches) {
    // This is only the minimum for seeding pose optimization. The later
    // optimized-map-inlier and local-map checks still decide tracking success.
    return projected_matches >= 15;
}

constexpr bool CanStartImuRgbdVisualRecovery(bool is_imu_rgbd,
                                             bool local_mapping_enabled,
                                             bool imu_initialized,
                                             bool inertial_ba2_complete) {
    return is_imu_rgbd && local_mapping_enabled &&
           (!imu_initialized || !inertial_ba2_complete);
}

constexpr bool ShouldUseImuRgbdVisualRecovery(bool is_imu_rgbd,
                                              bool local_mapping_enabled,
                                              bool recovery_active,
                                              bool tracking_recently_lost,
                                              bool imu_initialized,
                                              bool inertial_ba2_complete) {
    return (is_imu_rgbd && local_mapping_enabled && recovery_active &&
            tracking_recently_lost) ||
           CanStartImuRgbdVisualRecovery(is_imu_rgbd, local_mapping_enabled,
                                         imu_initialized,
                                         inertial_ba2_complete);
}

constexpr bool ShouldUseVisualPoseOptimization(
    bool imu_initialized, bool force_visual_pose_optimization) {
    return !imu_initialized || force_visual_pose_optimization;
}

constexpr bool ShouldForceVisualRecoveryPoseOptimization(bool recovery_frame,
                                                         bool imu_initialized) {
    return recovery_frame && !imu_initialized;
}

constexpr bool FailsRecentRelocalizationInlierThreshold(
    bool recently_relocalized, int matches_inliers,
    bool force_visual_pose_optimization) {
    return recently_relocalized && !force_visual_pose_optimization &&
           matches_inliers < 50;
}

constexpr bool ShouldUseCurrentFrameForLocalKeyFrameVoting(
    bool imu_initialized, bool recently_relocalized, bool is_imu_rgbd,
    bool visual_recovery_active, bool current_frame_has_visual_votes) {
    if (is_imu_rgbd && visual_recovery_active) {
        return current_frame_has_visual_votes;
    }
    return !imu_initialized || recently_relocalized;
}

constexpr bool ShouldMergeRgbdRecoveryLocalKeyFrameVotes(
    bool transactional, bool is_imu_rgbd, bool visual_recovery_active,
    bool current_frame_has_visual_votes) {
    return transactional && is_imu_rgbd && visual_recovery_active &&
           current_frame_has_visual_votes;
}

constexpr bool ShouldUseStrictRgbdTemporalLocalMapProjection(
    bool transactional, bool is_imu_rgbd, bool visual_recovery_active) {
    return transactional && is_imu_rgbd && visual_recovery_active;
}

inline double RgbdTemporalMapXyzToleranceMeters(double predicted_depth,
                                                 double measured_depth) {
    if (!std::isfinite(predicted_depth) || predicted_depth <= 0.0 ||
        !std::isfinite(measured_depth) || measured_depth <= 0.0) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return std::clamp(kRgbdTemporalMapXyzRelativeTolerance *
                          std::max(predicted_depth, measured_depth),
                      kRgbdTemporalMapXyzMinimumToleranceMeters,
                      kRgbdTemporalMapXyzMaximumToleranceMeters);
}

inline bool IsRgbdTemporalMapReprojectionConsistent(double error_pixels) {
    return std::isfinite(error_pixels) && error_pixels >= 0.0 &&
           error_pixels <= kRgbdTemporalMapReprojectionLimitPixels;
}

inline bool IsRgbdTemporalMapXyzConsistent(double predicted_depth,
                                           double measured_depth,
                                           double error_meters) {
    const double tolerance =
        RgbdTemporalMapXyzToleranceMeters(predicted_depth, measured_depth);
    return std::isfinite(tolerance) && std::isfinite(error_meters) &&
           error_meters >= 0.0 && error_meters <= tolerance;
}

constexpr bool ShouldSeedImuRgbdRecoveryLocalKeyFrames(
    bool is_imu_rgbd, bool visual_recovery_active,
    bool current_frame_has_visual_votes) {
    return is_imu_rgbd && visual_recovery_active &&
           !current_frame_has_visual_votes;
}

constexpr bool ShouldUseImuPoseSeedAfterReferenceFailure(
    bool is_imu_rgbd, bool visual_recovery_active,
    bool reference_tracking_succeeded, bool imu_initialized) {
    return is_imu_rgbd && visual_recovery_active &&
           !reference_tracking_succeeded && imu_initialized;
}

constexpr bool ShouldAppendImuRgbdRecoveryReferenceCandidate(
    bool candidate_valid, bool already_present, int candidate_count) {
    return candidate_valid && !already_present && candidate_count >= 0 &&
           candidate_count < kImuRgbdRecoveryReferenceCandidateLimit;
}

constexpr double RecentlyLostTimeoutSeconds(bool is_imu_rgbd,
                                            bool visual_recovery_active,
                                            double default_timeout_seconds) {
    return is_imu_rgbd && visual_recovery_active
               ? kImuRgbdVisualRecoveryTimeoutSeconds
               : default_timeout_seconds;
}

constexpr bool HasRecentlyLostTimedOut(double elapsed_seconds, bool is_imu_rgbd,
                                       bool visual_recovery_active,
                                       double default_timeout_seconds) {
    return elapsed_seconds >
           RecentlyLostTimeoutSeconds(is_imu_rgbd, visual_recovery_active,
                                      default_timeout_seconds);
}

constexpr bool ShouldDeclareImuRgbdVisualRecoveryLost(
    bool recovery_timed_out, bool visual_tracking_succeeded) {
    return recovery_timed_out && !visual_tracking_succeeded;
}

constexpr bool CanHoldLastImuRgbdPose(bool is_imu_rgbd,
                                      bool last_frame_has_pose) {
    return is_imu_rgbd && last_frame_has_pose;
}

constexpr bool ShouldKeepActiveMapAfterLost(bool is_imu_rgbd,
                                            bool imu_initialized,
                                            bool last_frame_has_pose) {
    return is_imu_rgbd && imu_initialized && last_frame_has_pose;
}

constexpr bool HasEnoughRecentlyLostInliers(int matches_inliers,
                                            bool is_imu_rgbd) {
    return is_imu_rgbd ? matches_inliers >= 15 : matches_inliers > 10;
}

/** Sensor-time gaps above this trigger soft recovery instead of tracking OK. */
inline constexpr double kInertialTimestampJumpSoftRecoverSeconds = 1.0;
/**
 * Gaps above this are treated as a true discontinuity (bag seek / long outage)
 * and still force a hard map reset. Soft recovery covers loop-correction
 * stalls that block tracking for up to the loop-correction wait timeout.
 */
inline constexpr double kInertialTimestampJumpHardResetSeconds = 30.0;

enum class InertialTimestampJumpAction : std::uint8_t {
    kNone = 0,
    kSoftRecover = 1,
    kHardReset = 2,
};

constexpr InertialTimestampJumpAction ClassifyInertialTimestampJump(
    double current_timestamp, double last_timestamp) {
    if (!(current_timestamp > last_timestamp)) {
        return InertialTimestampJumpAction::kNone;
    }
    const double delta_seconds = current_timestamp - last_timestamp;
    if (delta_seconds <= kInertialTimestampJumpSoftRecoverSeconds) {
        return InertialTimestampJumpAction::kNone;
    }
    if (delta_seconds <= kInertialTimestampJumpHardResetSeconds) {
        return InertialTimestampJumpAction::kSoftRecover;
    }
    return InertialTimestampJumpAction::kHardReset;
}

constexpr bool ShouldEvaluateNewKeyFrameAfterFailedImuRgbdRecovery(
    bool is_imu_rgbd, bool visual_recovery_active, bool failed_recovery,
    bool imu_initialized, bool attempted_local_map, bool current_pose_set,
    bool insert_key_frames_when_lost, bool used_imu_pose_seed) {
    return is_imu_rgbd && visual_recovery_active && failed_recovery &&
           imu_initialized && attempted_local_map && current_pose_set &&
           insert_key_frames_when_lost && used_imu_pose_seed;
}

}  // namespace autonomy::localization::atlas::internal

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_TRACKING_RECOVERY_POLICY_HPP_
