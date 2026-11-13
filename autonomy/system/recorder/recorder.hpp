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
 * @file recorder.hpp
 * @brief L1 bypass recorder process: rolling buffer, event trigger, MCAP flush.
 *
 * Independent of planning/control/perception. Capture never back-pressures
 * publishers. Flush moves payloads via TakeAll and writes MCAP + sidecar.
 */

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autolink/node/node.hpp"
#include "autonomy/system/recorder/episode.hpp"
#include "autonomy/system/recorder/hash_chain.hpp"
#include "autonomy/system/recorder/mcap_writer.hpp"
#include "autonomy/system/recorder/proto/recorder_options.pb.h"
#include "autonomy/system/recorder/rolling_buffer.hpp"
#include "autonomy/system/recorder/signals.hpp"
#include "autonomy/system/recorder/subscriber.hpp"
#include "autonomy/system/recorder/trigger.hpp"
#include "autonomy/system/recorder/version_tuple.hpp"
#include "autonomy/system/safety/safety_latch.hpp"

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief Load RecorderOptions from a text protobuf file.
 * @param [in] conf_file: Path to recorder.pb.txt (or equivalent).
 * @return Parsed options; defaults if the file is missing.
 */
proto::RecorderOptions LoadRecorderOptions(const std::string& conf_file);

/**
 * @brief Default record root: AUTONOMY_RECORD_ROOT, else $HOME/.autonomy/records.
 * @return Directory path.
 */
std::string DefaultRecordRoot();

/**
 * @brief Bypass capture node. Attach BEST_EFFORT readers; flush on trigger / period.
 *
 * HandleSample parses only channels whose SampleFlag bits are set. cmd_vel
 * actions cap at 8192. SafetyLatch is a member, not reconstructed each tick.
 */
class Recorder {
public:
    /**
     * @brief Apply options, version quadruple, buffer, and trigger engine.
     * @param [in] options: RecorderOptions.
     * @return True if configuration is usable.
     */
    bool Configure(const proto::RecorderOptions& options);

    /**
     * @brief Attach bypass subscribers on the autolink node.
     * @param [in] node: Shared autolink node.
     * @return True if at least one channel attached.
     */
    bool Start(const std::shared_ptr<autolink::Node>& node);

    /**
     * @brief Poll timeouts, SafetyLatch edges, and periodic flush (~100 ms).
     */
    void SpinOnce();

    /**
     * @brief Close the open episode and detach readers.
     */
    void Shutdown();

    /**
     * @brief Ingest one bypass sample: parse by flags, buffer, maybe trigger flush.
     * @param [in] sample: Copied RawMessage sample (by value for move into the window).
     */
    void HandleSample(Sample sample);

    /**
     * @brief Version-quadruple check from Configure.
     * @return Admitted flag and missing field names.
     */
    const VersionCheck& version() const { return version_; }

    /**
     * @brief Number of episodes flushed this process.
     * @return Episode count.
     */
    size_t episode_count() const { return episode_count_; }

    /**
     * @brief Path of the last flushed MCAP (or sidecar dir).
     * @return Last output path; empty before the first flush.
     */
    const std::string& last_path() const { return last_path_; }

private:
    /**
     * @brief Queue a trigger and flush if this sample opened a new event.
     * @param [in] event: Trigger to enqueue.
     */
    void HandleTrigger(const proto::TriggerEvent& event);

    /**
     * @brief TakeAll the window and write MCAP + sidecar. Caller holds mutex_.
     * @param [in] episode_id: Episode id.
     * @param [in] close_episode: True to close the tracker after write.
     * @param [in] closed: Closed EpisodeState, or nullptr if still open.
     */
    void FlushLocked(const std::string& episode_id, bool close_episode,
                     const EpisodeState* closed);

    /**
     * @brief Build EpisodeHeader from current version, triggers, and BT root.
     * @param [in] episode_id: Episode id.
     * @param [in] closed: Closed state, or nullptr.
     * @return Header proto.
     */
    proto::EpisodeHeader MakeHeaderLocked(const std::string& episode_id,
                                          const EpisodeState* closed) const;

    /**
     * @brief Output directory: episodes if admitted, else diagnostics.
     * @param [in] admitted: Version quadruple complete.
     * @return Directory path.
     */
    std::string OutputDir(bool admitted) const;

    proto::RecorderOptions options_;
    VersionCheck version_;
    std::unique_ptr<RollingBuffer> buffer_;
    std::unique_ptr<TriggerEngine> triggers_;
    EpisodeTracker episodes_;
    BypassSubscriber subscriber_;
    McapWriter writer_;
    PathTracker path_tracker_;
    SafetyHashChain chain_{};
    safety::SafetyLatch latch_;  ///< Process-shared E-Stop; reused each SpinOnce.
    std::mutex mutex_;
    std::vector<proto::TriggerEvent> pending_triggers_;
    std::vector<proto::Observation> observations_;
    std::vector<proto::ActionEvent> actions_;  ///< Capped (8192, keep last 4096).
    std::vector<proto::SafetyEvent> pending_safety_;
    int64_t last_flushed_ns_{0};
    int64_t last_periodic_flush_ns_{0};
    uint64_t episode_count_{0};
    std::string last_path_;
    proto::TimeSource time_source_{proto::TIME_SOURCE_SOFTWARE};
};

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
