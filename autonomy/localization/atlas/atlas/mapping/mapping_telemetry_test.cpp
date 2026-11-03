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

#include "atlas/mapping/mapping_telemetry.hpp"

#include <gtest/gtest.h>

namespace autonomy::localization::atlas::internal {
namespace {

TEST(MappingTelemetryTest, DisabledTelemetryDoesNotMutateStats) {
    MappingTelemetry telemetry(false);
    telemetry.RecordProcessedKeyFrame();
    telemetry.RecordClosePointThresholds(true, true);
    telemetry.RecordBusyKeyFrameRequest(BusyKeyFrameReason::kWeakOnly, true,
                                        false, 1, 0);
    telemetry.RecordBusyKeyFrameRequest(BusyKeyFrameReason::kCloseOnly, false,
                                        true, 2, 1);
    telemetry.ObservePendingQueueDepth(7);
    telemetry.RecordNeighborsSkippedPending();
    telemetry.RecordLbaSkippedPending();
    telemetry.RecordLbaStarted();
    telemetry.RecordLbaCompleted(8, 120);
    telemetry.RecordLbaReturnedWithAbortFlag();

    const MappingTelemetryStats& stats = telemetry.stats();
    EXPECT_EQ(stats.processed_kfs, 0);
    EXPECT_EQ(stats.busy_kf_admissions, 0);
    EXPECT_EQ(stats.busy_kf_requests, 0);
    EXPECT_EQ(stats.busy_kf_deferred, 0);
    EXPECT_EQ(stats.busy_kf_queue_limit_deferred, 0);
    EXPECT_EQ(stats.close_point_evaluations, 0);
    EXPECT_EQ(stats.tracked_close_lt_100, 0);
    EXPECT_EQ(stats.nontracked_close_gt_70, 0);
    EXPECT_EQ(stats.close_need, 0);
    EXPECT_EQ(stats.pending_queue_peak, 0);
    EXPECT_EQ(stats.lba_started, 0);
    EXPECT_EQ(stats.lba_completed, 0);
}

TEST(MappingTelemetryTest, TracksCountersPeaksAndCompletedLbaGap) {
    MappingTelemetry telemetry(true);
    telemetry.RecordBusyKeyFrameRequest(BusyKeyFrameReason::kWeakOnly, true,
                                        false, 1, 0);
    telemetry.RecordBusyKeyFrameRequest(BusyKeyFrameReason::kCloseOnly, true,
                                        false, 2, 1);
    telemetry.ObservePendingQueueDepth(2);
    telemetry.ObservePendingQueueDepth(5);
    telemetry.ObservePendingQueueDepth(3);
    telemetry.RecordNeighborsSkippedPending();
    telemetry.RecordLbaSkippedPending();

    telemetry.RecordProcessedKeyFrame();
    telemetry.RecordProcessedKeyFrame();
    telemetry.RecordLbaStarted();
    telemetry.RecordLbaReturnedWithAbortFlag();
    telemetry.RecordProcessedKeyFrame();
    telemetry.RecordProcessedKeyFrame();
    telemetry.RecordLbaStarted();
    telemetry.RecordLbaCompleted(8, 120);
    telemetry.RecordProcessedKeyFrame();

    const MappingTelemetryStats& stats = telemetry.stats();
    EXPECT_EQ(stats.processed_kfs, 5);
    EXPECT_EQ(stats.busy_kf_admissions, 2);
    EXPECT_EQ(stats.pending_queue_peak, 5);
    EXPECT_EQ(stats.neighbors_skipped_pending, 1);
    EXPECT_EQ(stats.lba_skipped_pending, 1);
    EXPECT_EQ(stats.lba_started, 2);
    EXPECT_EQ(stats.lba_completed, 1);
    EXPECT_EQ(stats.lba_returned_with_abort_flag, 1);
    EXPECT_EQ(stats.max_kfs_since_completed_lba, 4);
    EXPECT_EQ(stats.trailing_kfs_since_completed_lba, 1);
}

TEST(MappingTelemetryTest, TracksBusyAdmissionReasonsAndDeferrals) {
    MappingTelemetry telemetry(true);
    telemetry.RecordClosePointThresholds(true, false);
    telemetry.RecordClosePointThresholds(false, true);
    telemetry.RecordClosePointThresholds(true, true);

    telemetry.RecordBusyKeyFrameRequest(BusyKeyFrameReason::kWeakOnly, true,
                                        false, 1, 0);
    telemetry.RecordBusyKeyFrameRequest(BusyKeyFrameReason::kCloseOnly, false,
                                        true, 4, 0);
    telemetry.RecordBusyKeyFrameRequest(BusyKeyFrameReason::kWeakAndClose, true,
                                        false, 5, 2);

    const MappingTelemetryStats& stats = telemetry.stats();
    EXPECT_EQ(stats.busy_kf_requests, 3);
    EXPECT_EQ(stats.busy_kf_admissions, 2);
    EXPECT_EQ(stats.busy_kf_deferred, 1);
    EXPECT_EQ(stats.busy_kf_queue_limit_deferred, 1);
    EXPECT_EQ(stats.close_point_evaluations, 3);
    EXPECT_EQ(stats.tracked_close_lt_100, 2);
    EXPECT_EQ(stats.nontracked_close_gt_70, 2);
    EXPECT_EQ(stats.close_need, 1);

    EXPECT_EQ(stats.busy_kf_requests_by_reason[TelemetryIndex(
                  BusyKeyFrameReason::kWeakOnly)],
              1);
    EXPECT_EQ(stats.busy_kf_requests_by_reason[TelemetryIndex(
                  BusyKeyFrameReason::kCloseOnly)],
              1);
    EXPECT_EQ(stats.busy_kf_requests_by_reason[TelemetryIndex(
                  BusyKeyFrameReason::kWeakAndClose)],
              1);
    EXPECT_EQ(stats.busy_kf_admitted_by_reason[TelemetryIndex(
                  BusyKeyFrameReason::kWeakOnly)],
              1);
    EXPECT_EQ(stats.busy_kf_admitted_by_reason[TelemetryIndex(
                  BusyKeyFrameReason::kCloseOnly)],
              0);
    EXPECT_EQ(stats.busy_kf_admitted_by_reason[TelemetryIndex(
                  BusyKeyFrameReason::kWeakAndClose)],
              1);
    EXPECT_EQ(stats.busy_kf_deferred_by_reason[TelemetryIndex(
                  BusyKeyFrameReason::kWeakOnly)],
              0);
    EXPECT_EQ(stats.busy_kf_deferred_by_reason[TelemetryIndex(
                  BusyKeyFrameReason::kCloseOnly)],
              1);
    EXPECT_EQ(stats.busy_kf_deferred_by_reason[TelemetryIndex(
                  BusyKeyFrameReason::kWeakAndClose)],
              0);
    EXPECT_EQ(stats.busy_admission_frame_gap[TelemetryIndex(
                  AdmissionFrameGapBucket::kUpToOne)],
              1);
    EXPECT_EQ(stats.busy_admission_frame_gap[TelemetryIndex(
                  AdmissionFrameGapBucket::kFiveToNine)],
              1);
    EXPECT_EQ(stats.busy_admission_lba_generation_delta[TelemetryIndex(
                  AdmissionLbaGenerationDeltaBucket::kZero)],
              1);
    EXPECT_EQ(stats.busy_admission_lba_generation_delta[TelemetryIndex(
                  AdmissionLbaGenerationDeltaBucket::kTwoOrMore)],
              1);
}

TEST(MappingTelemetryTest, BucketsAdmissionFrameGapAndLbaGenerationDelta) {
    MappingTelemetry telemetry(true);
    telemetry.RecordBusyKeyFrameRequest(BusyKeyFrameReason::kWeakOnly, true,
                                        false, 1, 0);
    telemetry.RecordBusyKeyFrameRequest(BusyKeyFrameReason::kWeakOnly, true,
                                        false, 2, 1);
    telemetry.RecordBusyKeyFrameRequest(BusyKeyFrameReason::kWeakOnly, true,
                                        false, 4, 1);
    telemetry.RecordBusyKeyFrameRequest(BusyKeyFrameReason::kWeakOnly, true,
                                        false, 5, 3);
    telemetry.RecordBusyKeyFrameRequest(BusyKeyFrameReason::kWeakOnly, true,
                                        false, 9, 4);
    telemetry.RecordBusyKeyFrameRequest(BusyKeyFrameReason::kWeakOnly, true,
                                        false, 10, 6);

    const MappingTelemetryStats& stats = telemetry.stats();
    EXPECT_EQ(stats.busy_admission_frame_gap[TelemetryIndex(
                  AdmissionFrameGapBucket::kUpToOne)],
              1);
    EXPECT_EQ(stats.busy_admission_frame_gap[TelemetryIndex(
                  AdmissionFrameGapBucket::kTwoToFour)],
              2);
    EXPECT_EQ(stats.busy_admission_frame_gap[TelemetryIndex(
                  AdmissionFrameGapBucket::kFiveToNine)],
              2);
    EXPECT_EQ(stats.busy_admission_frame_gap[TelemetryIndex(
                  AdmissionFrameGapBucket::kTenOrMore)],
              1);
    EXPECT_EQ(stats.busy_admission_lba_generation_delta[TelemetryIndex(
                  AdmissionLbaGenerationDeltaBucket::kZero)],
              2);
    EXPECT_EQ(stats.busy_admission_lba_generation_delta[TelemetryIndex(
                  AdmissionLbaGenerationDeltaBucket::kOne)],
              2);
    EXPECT_EQ(stats.busy_admission_lba_generation_delta[TelemetryIndex(
                  AdmissionLbaGenerationDeltaBucket::kTwoOrMore)],
              2);
}

TEST(MappingTelemetryTest, ComputesRatiosAndCompletedLbaMeans) {
    MappingTelemetry telemetry(true);
    for (int i = 0; i < 4; ++i) {
        telemetry.RecordProcessedKeyFrame();
    }
    telemetry.RecordLbaStarted();
    telemetry.RecordLbaReturnedWithAbortFlag();
    telemetry.RecordLbaStarted();
    telemetry.RecordLbaCompleted(6, 90);
    telemetry.RecordLbaStarted();
    telemetry.RecordLbaCompleted(10, 150);

    const MappingTelemetryStats& stats = telemetry.stats();
    EXPECT_DOUBLE_EQ(stats.LbaCompletionRatio(), 2.0 / 3.0);
    EXPECT_DOUBLE_EQ(stats.CompletedLbasPerProcessedKeyFrame(), 0.5);
    EXPECT_DOUBLE_EQ(stats.MeanOptimizedKeyFrames(), 8.0);
    EXPECT_DOUBLE_EQ(stats.MeanEdges(), 120.0);
}

TEST(MappingTelemetryTest, FormatsStableSingleLineSummary) {
    MappingTelemetry telemetry(true);
    telemetry.RecordProcessedKeyFrame();
    telemetry.RecordProcessedKeyFrame();
    telemetry.RecordClosePointThresholds(true, true);
    telemetry.RecordBusyKeyFrameRequest(BusyKeyFrameReason::kWeakAndClose, true,
                                        false, 5, 2);
    telemetry.RecordBusyKeyFrameRequest(BusyKeyFrameReason::kCloseOnly, false,
                                        true, 2, 2);
    telemetry.ObservePendingQueueDepth(3);
    telemetry.RecordNeighborsSkippedPending();
    telemetry.RecordLbaSkippedPending();
    telemetry.RecordLbaStarted();
    telemetry.RecordLbaCompleted(4, 40);

    EXPECT_EQ(
        FormatMappingTelemetrySummary(telemetry.stats()),
        "Atlas mapping telemetry: processed_kfs=2 busy_kf_admissions=1 "
        "busy_kf_requests=2 busy_kf_deferred=1 busy_req_weak_only=0 "
        "busy_req_close_only=1 busy_req_both=1 busy_admit_weak_only=0 "
        "busy_admit_close_only=0 busy_admit_both=1 "
        "busy_defer_weak_only=0 busy_defer_close_only=1 "
        "busy_defer_both=0 busy_queue_limit_deferred=1 "
        "close_evaluations=1 tracked_close_lt_100=1 "
        "nontracked_close_gt_70=1 close_need=1 busy_admit_gap_le_1=0 "
        "busy_admit_gap_2_4=0 busy_admit_gap_5_9=1 "
        "busy_admit_gap_ge_10=0 busy_admit_lba_delta_0=0 "
        "busy_admit_lba_delta_1=0 busy_admit_lba_delta_ge_2=1 "
        "pending_queue_peak=3 neighbors_skipped_pending=1 "
        "lba_skipped_pending=1 lba_started=1 lba_completed=1 "
        "lba_returned_with_abort_flag=0 max_kfs_since_completed_lba=2 "
        "trailing_kfs_since_completed_lba=0 lba_completion_ratio=1.000000 "
        "lba_completed_per_processed_kf=0.500000 mean_opt_kfs=4.000000 "
        "mean_edges=40.000000");
}

}  // namespace
}  // namespace autonomy::localization::atlas::internal
