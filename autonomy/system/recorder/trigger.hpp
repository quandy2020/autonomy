/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

/**
 * @file trigger.hpp
 * @brief L1 triggers: BT failure, E-Stop, path deviation, timeout, task, loc quality.
 */

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/system/recorder/proto/evidence.pb.h"
#include "autonomy/system/recorder/proto/recorder_options.pb.h"
#include "autonomy/system/recorder/rolling_buffer.hpp"

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief Emit TriggerEvent from samples and external signals; Classify to P0–P3.
 *
 * Channel timeouts are polled in PollTimeouts. E-Stop and localization quality
 * are edge-triggered to avoid duplicates.
 */
class TriggerEngine {
public:
    /**
     * @brief Load timeout table and thresholds from options.
     * @param [in] options: RecorderOptions.
     */
    explicit TriggerEngine(const proto::RecorderOptions& options);

    /**
     * @brief Record last-seen time for the channel. Does not emit a trigger.
     * @param [in] sample: Bypass sample.
     * @return Always nullopt; dedicated On* methods emit events.
     */
    std::optional<proto::TriggerEvent> OnSample(const Sample& sample);

    /**
     * @brief Trigger when a BT node enters FAILURE.
     * @param [in] node_name: BT node name.
     * @param [in] previous: Previous status.
     * @param [in] current: Current status.
     * @param [in] clock: Dual clock.
     * @return TriggerEvent on a FAILURE edge.
     */
    std::optional<proto::TriggerEvent> OnBtStatus(const std::string& node_name,
                                                  const std::string& previous,
                                                  const std::string& current,
                                                  const proto::DualClock& clock);

    /**
     * @brief Trigger on a SafetyLatch edge.
     * @param [in] latched: Whether the latch is now set.
     * @param [in] reason: Latch reason.
     * @param [in] clock: Dual clock.
     * @return Event on an edge change.
     */
    std::optional<proto::TriggerEvent> OnSafetyLatch(
        bool latched, const std::string& reason, const proto::DualClock& clock);

    /**
     * @brief Trigger when lateral path deviation exceeds the threshold.
     * @param [in] meters: Deviation in meters.
     * @param [in] clock: Dual clock.
     * @return Event when over threshold.
     */
    std::optional<proto::TriggerEvent> OnPathDeviation(
        double meters, const proto::DualClock& clock);

    /**
     * @brief Trigger when planning cost exceeds the threshold.
     * @param [in] value: Cost value.
     * @param [in] clock: Dual clock.
     * @return Event when over threshold.
     */
    std::optional<proto::TriggerEvent> OnCost(double value,
                                              const proto::DualClock& clock);

    /**
     * @brief Trigger on a teleop takeover.
     * @param [in] clock: Dual clock.
     * @param [in] channel: Source topic.
     * @return TriggerEvent.
     */
    std::optional<proto::TriggerEvent> OnTakeover(const proto::DualClock& clock,
                                                  const std::string& channel);

    /**
     * @brief Trigger on a task-feedback failure.
     * @param [in] clock: Dual clock.
     * @param [in] detail: Failure text.
     * @param [in] channel: Source topic.
     * @return TriggerEvent.
     */
    std::optional<proto::TriggerEvent> OnTaskFailure(
        const proto::DualClock& clock, const std::string& detail,
        const std::string& channel);

    /**
     * @brief Edge-trigger when localization quality drops below the floor.
     * @param [in] quality: Quality in [0, 1].
     * @param [in] clock: Dual clock.
     * @return Event on a falling edge.
     */
    std::optional<proto::TriggerEvent> OnLocalizationQuality(
        double quality, const proto::DualClock& clock);

    /**
     * @brief Scan the timeout table and emit TRIGGER_TIMEOUT for stale channels.
     * @param [in] now: Current dual clock.
     * @return Timeout events this tick (may be empty).
     */
    std::vector<proto::TriggerEvent> PollTimeouts(const proto::DualClock& now);

    /**
     * @brief Map episode events and task outcome to P0–P3.
     * @param [in] events: Triggers in this episode.
     * @param [in] task_success: Whether the task succeeded.
     * @return Priority.
     */
    proto::Priority Classify(const std::vector<proto::TriggerEvent>& events,
                             bool task_success) const;

private:
    /**
     * @brief Assemble a TriggerEvent.
     * @param [in] kind: Trigger kind.
     * @param [in] clock: Dual clock.
     * @param [in] detail: Text detail.
     * @param [in] channel: Optional source topic.
     * @return TriggerEvent.
     */
    proto::TriggerEvent Make(proto::TriggerKind kind, const proto::DualClock& clock,
                             const std::string& detail,
                             const std::string& channel = {}) const;

    proto::RecorderOptions options_;
    std::unordered_map<std::string, int64_t> last_seen_ns_;  ///< Last arrival per channel.
    std::unordered_map<std::string, double> timeouts_sec_;   ///< Timeout seconds per channel.
    bool estop_latched_{false};                              ///< E-Stop edge memory.
    bool loc_quality_latched_{false};                        ///< Loc-quality edge memory.
    std::vector<double> costs_;                              ///< Recent cost samples.
};

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
