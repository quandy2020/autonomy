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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_TRACKING_KEYFRAME_ADMISSION_POLICY_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_TRACKING_KEYFRAME_ADMISSION_POLICY_HPP_

namespace autonomy::localization::atlas::internal {

inline constexpr int kBusyKeyFrameQueueLimit = 3;

enum class KeyFrameAdmissionDecision {
    kDefer,
    kInsertWithoutInterrupt,
    kInterruptBaAndInsert,
};

struct KeyFrameAdmissionSignals {
    // Keep the names tied to Tracker::NeedNewKeyFrame so the policy cannot
    // silently reinterpret the established trigger composition.
    bool c1a_max_frames_elapsed = false;
    bool c1b_min_frames_and_idle = false;
    bool c1c_noninertial_weak_or_close = false;
    bool c2_tracking_or_close_need = false;
    bool c3_inertial_time_elapsed = false;
    bool c4_imu_monocular_recovery = false;
    bool need_close_point_insertion = false;

    bool local_mapper_idle = false;
    bool local_mapper_initializing = false;
    bool supports_busy_admission = false;
    int queued_keyframes = 0;
};

constexpr bool IsKeyFrameRequested(const KeyFrameAdmissionSignals& signals) {
    return ((signals.c1a_max_frames_elapsed ||
             signals.c1b_min_frames_and_idle ||
             signals.c1c_noninertial_weak_or_close) &&
            signals.c2_tracking_or_close_need) ||
           signals.c3_inertial_time_elapsed ||
           signals.c4_imu_monocular_recovery;
}

constexpr KeyFrameAdmissionDecision DecideKeyFrameAdmission(
    const KeyFrameAdmissionSignals& signals) {
    if (!IsKeyFrameRequested(signals)) {
        return KeyFrameAdmissionDecision::kDefer;
    }

    // Preserve the existing local-mapper idle and initialization semantics.
    if (signals.local_mapper_idle || signals.local_mapper_initializing) {
        return KeyFrameAdmissionDecision::kInsertWithoutInterrupt;
    }

    // Routine cadence (c1a/c1b/c3/c4) must wait for local mapping. Only an
    // already-viable c1c weak-tracking request or an explicit close-point
    // emergency may interrupt an in-flight LBA.
    const bool emergency = signals.c2_tracking_or_close_need &&
                           (signals.c1c_noninertial_weak_or_close ||
                            signals.need_close_point_insertion);
    if (!signals.supports_busy_admission || !emergency ||
        signals.queued_keyframes < 0 ||
        signals.queued_keyframes >= kBusyKeyFrameQueueLimit) {
        return KeyFrameAdmissionDecision::kDefer;
    }

    return KeyFrameAdmissionDecision::kInterruptBaAndInsert;
}

}  // namespace autonomy::localization::atlas::internal

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_TRACKING_KEYFRAME_ADMISSION_POLICY_HPP_
