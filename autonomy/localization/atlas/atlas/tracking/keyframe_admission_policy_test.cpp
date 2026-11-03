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

#include "atlas/tracking/keyframe_admission_policy.hpp"

#include <gtest/gtest.h>

namespace autonomy::localization::atlas::internal {
namespace {

KeyFrameAdmissionSignals RoutineCadenceSignals() {
    KeyFrameAdmissionSignals signals;
    signals.c1a_max_frames_elapsed = true;
    signals.c2_tracking_or_close_need = true;
    signals.supports_busy_admission = true;
    return signals;
}

TEST(KeyFrameAdmissionPolicyTest, RequiresEstablishedCompositeTrigger) {
    KeyFrameAdmissionSignals signals;
    signals.local_mapper_idle = true;
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kDefer);

    signals.c1a_max_frames_elapsed = true;
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kDefer);

    signals.c1a_max_frames_elapsed = false;
    signals.c1c_noninertial_weak_or_close = true;
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kDefer);
}

TEST(KeyFrameAdmissionPolicyTest, PreservesIdleCompositeTriggers) {
    KeyFrameAdmissionSignals signals = RoutineCadenceSignals();
    signals.local_mapper_idle = true;
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kInsertWithoutInterrupt);

    signals.c1a_max_frames_elapsed = false;
    signals.c1b_min_frames_and_idle = true;
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kInsertWithoutInterrupt);

    signals.c1b_min_frames_and_idle = false;
    signals.c1c_noninertial_weak_or_close = true;
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kInsertWithoutInterrupt);
}

TEST(KeyFrameAdmissionPolicyTest, PreservesIdleInertialTriggers) {
    KeyFrameAdmissionSignals signals;
    signals.local_mapper_idle = true;
    signals.c3_inertial_time_elapsed = true;
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kInsertWithoutInterrupt);

    signals.c3_inertial_time_elapsed = false;
    signals.c4_imu_monocular_recovery = true;
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kInsertWithoutInterrupt);
}

TEST(KeyFrameAdmissionPolicyTest, PreservesInitializingAdmission) {
    KeyFrameAdmissionSignals signals = RoutineCadenceSignals();
    signals.local_mapper_initializing = true;
    signals.supports_busy_admission = false;
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kInsertWithoutInterrupt);
}

TEST(KeyFrameAdmissionPolicyTest, DefersRoutineBusyRequestsWithoutInterrupt) {
    KeyFrameAdmissionSignals signals = RoutineCadenceSignals();
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kDefer);

    signals.c1a_max_frames_elapsed = false;
    signals.c1b_min_frames_and_idle = true;
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kDefer);

    signals.c1b_min_frames_and_idle = false;
    signals.c3_inertial_time_elapsed = true;
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kDefer);

    signals.c3_inertial_time_elapsed = false;
    signals.c4_imu_monocular_recovery = true;
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kDefer);
}

TEST(KeyFrameAdmissionPolicyTest, BoundsBusyWeakTrackingAdmission) {
    KeyFrameAdmissionSignals signals;
    signals.c1c_noninertial_weak_or_close = true;
    signals.c2_tracking_or_close_need = true;
    signals.supports_busy_admission = true;
    signals.queued_keyframes = kBusyKeyFrameQueueLimit - 1;
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kInterruptBaAndInsert);

    signals.queued_keyframes = kBusyKeyFrameQueueLimit;
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kDefer);

    signals.queued_keyframes = -1;
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kDefer);
}

TEST(KeyFrameAdmissionPolicyTest, AllowsBoundedClosePointEmergency) {
    KeyFrameAdmissionSignals signals = RoutineCadenceSignals();
    signals.need_close_point_insertion = true;
    signals.queued_keyframes = kBusyKeyFrameQueueLimit - 1;
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kInterruptBaAndInsert);

    signals.queued_keyframes = kBusyKeyFrameQueueLimit;
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kDefer);
}

TEST(KeyFrameAdmissionPolicyTest, BusyEmergencyStillRequiresC2) {
    KeyFrameAdmissionSignals signals;
    signals.c1c_noninertial_weak_or_close = true;
    signals.c3_inertial_time_elapsed = true;
    signals.supports_busy_admission = true;
    signals.queued_keyframes = 0;
    EXPECT_TRUE(IsKeyFrameRequested(signals));
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kDefer);
}

TEST(KeyFrameAdmissionPolicyTest, MonocularFamilyNeverBusyAdmits) {
    KeyFrameAdmissionSignals signals = RoutineCadenceSignals();
    signals.supports_busy_admission = false;
    signals.queued_keyframes = 0;
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kDefer);

    signals.c1a_max_frames_elapsed = false;
    signals.c2_tracking_or_close_need = false;
    signals.c4_imu_monocular_recovery = true;
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kDefer);
}

TEST(KeyFrameAdmissionPolicyTest, EmergencyAloneDoesNotCreateARequest) {
    KeyFrameAdmissionSignals signals;
    signals.c2_tracking_or_close_need = true;
    signals.need_close_point_insertion = true;
    signals.supports_busy_admission = true;
    EXPECT_FALSE(IsKeyFrameRequested(signals));
    EXPECT_EQ(DecideKeyFrameAdmission(signals),
              KeyFrameAdmissionDecision::kDefer);
}

}  // namespace
}  // namespace autonomy::localization::atlas::internal
