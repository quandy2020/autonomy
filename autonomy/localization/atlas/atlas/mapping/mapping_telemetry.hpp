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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_MAPPING_MAPPING_TELEMETRY_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_MAPPING_MAPPING_TELEMETRY_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>

namespace autonomy::localization::atlas::internal {

enum class BusyKeyFrameReason : std::size_t {
    kWeakOnly = 0,
    kCloseOnly,
    kWeakAndClose,
    kCount,
};

enum class AdmissionFrameGapBucket : std::size_t {
    kUpToOne = 0,
    kTwoToFour,
    kFiveToNine,
    kTenOrMore,
    kCount,
};

enum class AdmissionLbaGenerationDeltaBucket : std::size_t {
    kZero = 0,
    kOne,
    kTwoOrMore,
    kCount,
};

inline constexpr std::size_t kBusyKeyFrameReasonCount =
    static_cast<std::size_t>(BusyKeyFrameReason::kCount);
inline constexpr std::size_t kAdmissionFrameGapBucketCount =
    static_cast<std::size_t>(AdmissionFrameGapBucket::kCount);
inline constexpr std::size_t kAdmissionLbaGenerationDeltaBucketCount =
    static_cast<std::size_t>(AdmissionLbaGenerationDeltaBucket::kCount);

template <typename Enum>
constexpr std::size_t TelemetryIndex(Enum value) {
    return static_cast<std::size_t>(value);
}

inline bool MappingTelemetryEnabledFromEnvironment() {
    static const bool enabled = [] {
        const char* const value = std::getenv("ATLAS_MAPPING_TELEMETRY");
        return value != nullptr && std::string_view(value) == "1";
    }();
    return enabled;
}

struct MappingTelemetryStats {
    std::uint64_t processed_kfs = 0;
    std::uint64_t busy_kf_admissions = 0;
    std::uint64_t busy_kf_requests = 0;
    std::uint64_t busy_kf_deferred = 0;
    std::array<std::uint64_t, kBusyKeyFrameReasonCount>
        busy_kf_requests_by_reason{};
    std::array<std::uint64_t, kBusyKeyFrameReasonCount>
        busy_kf_admitted_by_reason{};
    std::array<std::uint64_t, kBusyKeyFrameReasonCount>
        busy_kf_deferred_by_reason{};
    std::uint64_t busy_kf_queue_limit_deferred = 0;
    std::uint64_t close_point_evaluations = 0;
    std::uint64_t tracked_close_lt_100 = 0;
    std::uint64_t nontracked_close_gt_70 = 0;
    std::uint64_t close_need = 0;
    std::array<std::uint64_t, kAdmissionFrameGapBucketCount>
        busy_admission_frame_gap{};
    std::array<std::uint64_t, kAdmissionLbaGenerationDeltaBucketCount>
        busy_admission_lba_generation_delta{};
    std::uint64_t pending_queue_peak = 0;
    std::uint64_t neighbors_skipped_pending = 0;
    std::uint64_t lba_skipped_pending = 0;
    std::uint64_t lba_started = 0;
    std::uint64_t lba_completed = 0;
    std::uint64_t lba_returned_with_abort_flag = 0;
    std::uint64_t max_kfs_since_completed_lba = 0;
    std::uint64_t trailing_kfs_since_completed_lba = 0;
    std::uint64_t completed_lba_optimized_kfs_sum = 0;
    std::uint64_t completed_lba_edges_sum = 0;

    // System merges the tracker-owned admission counters into the final
    // local-mapper snapshot only after both worker threads have stopped.
    void MergeKeyFrameAdmissionStatsFrom(const MappingTelemetryStats& source) {
        busy_kf_admissions += source.busy_kf_admissions;
        busy_kf_requests += source.busy_kf_requests;
        busy_kf_deferred += source.busy_kf_deferred;
        busy_kf_queue_limit_deferred += source.busy_kf_queue_limit_deferred;
        close_point_evaluations += source.close_point_evaluations;
        tracked_close_lt_100 += source.tracked_close_lt_100;
        nontracked_close_gt_70 += source.nontracked_close_gt_70;
        close_need += source.close_need;
        for (std::size_t index = 0; index < kBusyKeyFrameReasonCount; ++index) {
            busy_kf_requests_by_reason[index] +=
                source.busy_kf_requests_by_reason[index];
            busy_kf_admitted_by_reason[index] +=
                source.busy_kf_admitted_by_reason[index];
            busy_kf_deferred_by_reason[index] +=
                source.busy_kf_deferred_by_reason[index];
        }
        for (std::size_t index = 0; index < kAdmissionFrameGapBucketCount;
             ++index) {
            busy_admission_frame_gap[index] +=
                source.busy_admission_frame_gap[index];
        }
        for (std::size_t index = 0;
             index < kAdmissionLbaGenerationDeltaBucketCount; ++index) {
            busy_admission_lba_generation_delta[index] +=
                source.busy_admission_lba_generation_delta[index];
        }
    }

    double LbaCompletionRatio() const {
        if (lba_started == 0) {
            return 0.0;
        }
        return static_cast<double>(lba_completed) /
               static_cast<double>(lba_started);
    }

    double CompletedLbasPerProcessedKeyFrame() const {
        if (processed_kfs == 0) {
            return 0.0;
        }
        return static_cast<double>(lba_completed) /
               static_cast<double>(processed_kfs);
    }

    double MeanOptimizedKeyFrames() const {
        if (lba_completed == 0) {
            return 0.0;
        }
        return static_cast<double>(completed_lba_optimized_kfs_sum) /
               static_cast<double>(lba_completed);
    }

    double MeanEdges() const {
        if (lba_completed == 0) {
            return 0.0;
        }
        return static_cast<double>(completed_lba_edges_sum) /
               static_cast<double>(lba_completed);
    }
};

// Fixed-size telemetry with no internal synchronization. Each owner must
// serialize updates and read the final snapshot only after its worker stops.
class MappingTelemetry
{
public:
    MappingTelemetry()
        : MappingTelemetry(MappingTelemetryEnabledFromEnvironment()) {}

    explicit MappingTelemetry(bool enabled) : enabled_(enabled) {}

    bool enabled() const {
        return enabled_;
    }

    const MappingTelemetryStats& stats() const {
        return stats_;
    }

    void RecordProcessedKeyFrame() {
        if (!enabled_) {
            return;
        }
        ++stats_.processed_kfs;
        ++stats_.trailing_kfs_since_completed_lba;
        stats_.max_kfs_since_completed_lba =
            std::max(stats_.max_kfs_since_completed_lba,
                     stats_.trailing_kfs_since_completed_lba);
    }

    void RecordClosePointThresholds(bool tracked_close_lt_100,
                                    bool nontracked_close_gt_70) {
        if (!enabled_) {
            return;
        }
        ++stats_.close_point_evaluations;
        if (tracked_close_lt_100) {
            ++stats_.tracked_close_lt_100;
        }
        if (nontracked_close_gt_70) {
            ++stats_.nontracked_close_gt_70;
        }
        if (tracked_close_lt_100 && nontracked_close_gt_70) {
            ++stats_.close_need;
        }
    }

    // The request cohort is restricted to busy-mapper emergency candidates,
    // so weak-only, close-only, and both are exhaustive. Routine cadence
    // deferrals deliberately do not enter these reason counters.
    void RecordBusyKeyFrameRequest(BusyKeyFrameReason reason, bool admitted,
                                   bool queue_limit_deferred,
                                   std::uint64_t frame_gap,
                                   std::uint64_t lba_completion_generation) {
        if (!enabled_) {
            return;
        }
        const std::size_t reason_index = TelemetryIndex(reason);
        if (reason_index >= kBusyKeyFrameReasonCount) {
            return;
        }
        ++stats_.busy_kf_requests;
        ++stats_.busy_kf_requests_by_reason[reason_index];
        if (!admitted) {
            ++stats_.busy_kf_deferred;
            ++stats_.busy_kf_deferred_by_reason[reason_index];
            if (queue_limit_deferred) {
                ++stats_.busy_kf_queue_limit_deferred;
            }
            return;
        }

        ++stats_.busy_kf_admissions;
        ++stats_.busy_kf_admitted_by_reason[reason_index];
        AdmissionFrameGapBucket gap_bucket =
            AdmissionFrameGapBucket::kTenOrMore;
        if (frame_gap <= 1) {
            gap_bucket = AdmissionFrameGapBucket::kUpToOne;
        } else if (frame_gap <= 4) {
            gap_bucket = AdmissionFrameGapBucket::kTwoToFour;
        } else if (frame_gap <= 9) {
            gap_bucket = AdmissionFrameGapBucket::kFiveToNine;
        }
        ++stats_.busy_admission_frame_gap[TelemetryIndex(gap_bucket)];

        const std::uint64_t generation_delta =
            lba_completion_generation >= last_admission_lba_generation_
                ? lba_completion_generation - last_admission_lba_generation_
                : 0;
        last_admission_lba_generation_ = lba_completion_generation;
        AdmissionLbaGenerationDeltaBucket generation_bucket =
            AdmissionLbaGenerationDeltaBucket::kTwoOrMore;
        if (generation_delta == 0) {
            generation_bucket = AdmissionLbaGenerationDeltaBucket::kZero;
        } else if (generation_delta == 1) {
            generation_bucket = AdmissionLbaGenerationDeltaBucket::kOne;
        }
        ++stats_.busy_admission_lba_generation_delta[TelemetryIndex(
            generation_bucket)];
    }

    void ObservePendingQueueDepth(std::size_t depth) {
        if (!enabled_) {
            return;
        }
        stats_.pending_queue_peak = std::max(stats_.pending_queue_peak,
                                             static_cast<std::uint64_t>(depth));
    }

    void RecordNeighborsSkippedPending() {
        if (enabled_) {
            ++stats_.neighbors_skipped_pending;
        }
    }

    void RecordLbaSkippedPending() {
        if (enabled_) {
            ++stats_.lba_skipped_pending;
        }
    }

    void RecordLbaStarted() {
        if (enabled_) {
            ++stats_.lba_started;
        }
    }

    void RecordLbaCompleted(int optimized_keyframes, int edges) {
        if (!enabled_) {
            return;
        }
        ++stats_.lba_completed;
        stats_.completed_lba_optimized_kfs_sum +=
            static_cast<std::uint64_t>(std::max(optimized_keyframes, 0));
        stats_.completed_lba_edges_sum +=
            static_cast<std::uint64_t>(std::max(edges, 0));
        stats_.trailing_kfs_since_completed_lba = 0;
    }

    void RecordLbaReturnedWithAbortFlag() {
        if (enabled_) {
            ++stats_.lba_returned_with_abort_flag;
        }
    }

private:
    const bool enabled_;
    MappingTelemetryStats stats_;
    std::uint64_t last_admission_lba_generation_ = 0;
};

inline std::string FormatMappingTelemetrySummary(
    const MappingTelemetryStats& stats) {
    std::ostringstream stream;
    stream << "Atlas mapping telemetry: processed_kfs=" << stats.processed_kfs
           << " busy_kf_admissions=" << stats.busy_kf_admissions
           << " busy_kf_requests=" << stats.busy_kf_requests
           << " busy_kf_deferred=" << stats.busy_kf_deferred
           << " busy_req_weak_only="
           << stats.busy_kf_requests_by_reason[TelemetryIndex(
                  BusyKeyFrameReason::kWeakOnly)]
           << " busy_req_close_only="
           << stats.busy_kf_requests_by_reason[TelemetryIndex(
                  BusyKeyFrameReason::kCloseOnly)]
           << " busy_req_both="
           << stats.busy_kf_requests_by_reason[TelemetryIndex(
                  BusyKeyFrameReason::kWeakAndClose)]
           << " busy_admit_weak_only="
           << stats.busy_kf_admitted_by_reason[TelemetryIndex(
                  BusyKeyFrameReason::kWeakOnly)]
           << " busy_admit_close_only="
           << stats.busy_kf_admitted_by_reason[TelemetryIndex(
                  BusyKeyFrameReason::kCloseOnly)]
           << " busy_admit_both="
           << stats.busy_kf_admitted_by_reason[TelemetryIndex(
                  BusyKeyFrameReason::kWeakAndClose)]
           << " busy_defer_weak_only="
           << stats.busy_kf_deferred_by_reason[TelemetryIndex(
                  BusyKeyFrameReason::kWeakOnly)]
           << " busy_defer_close_only="
           << stats.busy_kf_deferred_by_reason[TelemetryIndex(
                  BusyKeyFrameReason::kCloseOnly)]
           << " busy_defer_both="
           << stats.busy_kf_deferred_by_reason[TelemetryIndex(
                  BusyKeyFrameReason::kWeakAndClose)]
           << " busy_queue_limit_deferred="
           << stats.busy_kf_queue_limit_deferred
           << " close_evaluations=" << stats.close_point_evaluations
           << " tracked_close_lt_100=" << stats.tracked_close_lt_100
           << " nontracked_close_gt_70=" << stats.nontracked_close_gt_70
           << " close_need=" << stats.close_need << " busy_admit_gap_le_1="
           << stats.busy_admission_frame_gap[TelemetryIndex(
                  AdmissionFrameGapBucket::kUpToOne)]
           << " busy_admit_gap_2_4="
           << stats.busy_admission_frame_gap[TelemetryIndex(
                  AdmissionFrameGapBucket::kTwoToFour)]
           << " busy_admit_gap_5_9="
           << stats.busy_admission_frame_gap[TelemetryIndex(
                  AdmissionFrameGapBucket::kFiveToNine)]
           << " busy_admit_gap_ge_10="
           << stats.busy_admission_frame_gap[TelemetryIndex(
                  AdmissionFrameGapBucket::kTenOrMore)]
           << " busy_admit_lba_delta_0="
           << stats.busy_admission_lba_generation_delta[TelemetryIndex(
                  AdmissionLbaGenerationDeltaBucket::kZero)]
           << " busy_admit_lba_delta_1="
           << stats.busy_admission_lba_generation_delta[TelemetryIndex(
                  AdmissionLbaGenerationDeltaBucket::kOne)]
           << " busy_admit_lba_delta_ge_2="
           << stats.busy_admission_lba_generation_delta[TelemetryIndex(
                  AdmissionLbaGenerationDeltaBucket::kTwoOrMore)]
           << " pending_queue_peak=" << stats.pending_queue_peak
           << " neighbors_skipped_pending=" << stats.neighbors_skipped_pending
           << " lba_skipped_pending=" << stats.lba_skipped_pending
           << " lba_started=" << stats.lba_started
           << " lba_completed=" << stats.lba_completed
           << " lba_returned_with_abort_flag="
           << stats.lba_returned_with_abort_flag
           << " max_kfs_since_completed_lba="
           << stats.max_kfs_since_completed_lba
           << " trailing_kfs_since_completed_lba="
           << stats.trailing_kfs_since_completed_lba << std::fixed
           << std::setprecision(6)
           << " lba_completion_ratio=" << stats.LbaCompletionRatio()
           << " lba_completed_per_processed_kf="
           << stats.CompletedLbasPerProcessedKeyFrame()
           << " mean_opt_kfs=" << stats.MeanOptimizedKeyFrames()
           << " mean_edges=" << stats.MeanEdges();
    return stream.str();
}

}  // namespace autonomy::localization::atlas::internal

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_MAPPING_MAPPING_TELEMETRY_HPP_
