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
 * @file episode.hpp
 * @brief Episode lifetime: BT-root open/close, goal window, session full-dump.
 */

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "autonomy/system/recorder/proto/evidence.pb.h"

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief One behavior-tree status transition.
 */
struct BtTransition {
    std::string node_name;   ///< BT node.
    std::string previous;    ///< Previous status.
    std::string current;     ///< Current status.
    proto::DualClock clock;  ///< Transition time.
};

/**
 * @brief Snapshot of the current (or just-closed) episode.
 */
struct EpisodeState {
    bool open{false};                       ///< True while the window is open.
    std::string episode_id;                 ///< e.g. rec_<wall>_<n>.
    std::string bt_root;                    ///< Root node at open.
    proto::DualClock start;                 ///< Open time.
    std::vector<BtTransition> transitions;  ///< BT transitions in the window.
};

/**
 * @brief Open and close episodes from BT root status and goals.
 *
 * Default: Close when the BT root reaches SUCCESS / CLOSED. session_mode
 * (full_dump) never auto-closes; transitions cap at 2048 (drop the first half
 * when exceeded) so a long session cannot unbounded-grow RAM.
 */
class EpisodeTracker {
public:
    /**
     * @brief Tester full dump: do not close the window on BT recovery.
     * @param [in] session: True enables session mode.
     */
    void SetSessionMode(bool session) { session_mode_ = session; }

    /**
     * @brief Whether session mode is on.
     * @return True for full_dump sessions.
     */
    bool session_mode() const { return session_mode_; }

    /**
     * @brief Open a window on a task/nav goal if none is open.
     * @param [in] clock: Dual clock.
     */
    void OnGoal(const proto::DualClock& clock);

    /**
     * @brief Ingest a BT transition; return the closed state if the root closed.
     * @param [in] transition: BT transition.
     * @return Closed EpisodeState; nullopt if the window stays open.
     */
    std::optional<EpisodeState> OnBtStatus(const BtTransition& transition);

    /**
     * @brief Force-close the window.
     * @param [in] clock: Close time.
     * @param [in] reason: Reason (shutdown / periodic).
     * @return Closed state; nullopt if it was not open.
     */
    std::optional<EpisodeState> Close(const proto::DualClock& clock,
                                      const std::string& reason);

    /**
     * @brief Current state, including an unclosed window.
     * @return EpisodeState reference.
     */
    const EpisodeState& state() const { return state_; }

private:
    /**
     * @brief Open a new window and allocate episode_id.
     * @param [in] clock: Open time.
     * @param [in] root: BT root name.
     */
    void Open(const proto::DualClock& clock, const std::string& root);

    /**
     * @brief Allocate the next episode_id.
     * @param [in] clock: Used for the timestamp fragment.
     * @return New id.
     */
    std::string NextId(const proto::DualClock& clock);

    EpisodeState state_;
    uint64_t counter_{0};       ///< In-process sequence.
    bool session_mode_{false};  ///< Tester full_dump.
};

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
