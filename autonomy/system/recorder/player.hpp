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
 * @file player.hpp
 * @brief SIL/HIL MCAP player: republish RawMessage bytes; compare action sequences.
 */

#include <memory>
#include <string>
#include <vector>

#include "autolink/node/node.hpp"
#include "autonomy/system/recorder/proto/evidence.pb.h"

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief Matches REPLAY_PROFILE / replay_play.sh.
 *
 * open_loop = all topics; mid_stack = skip cmd_vel*; sensor_in = sensors only.
 */
enum class ReplayProfile {
    kOpenLoop = 0,   ///< Republish every recorded topic.
    kMidStack = 1,   ///< Skip /cmd_vel* (stack closed on actuators).
    kSensorIn = 2,   ///< Sensors only (HIL / perception-in).
};

/**
 * @brief Parse a profile name (open_loop / mid_stack / sensor_in). Unknown → open_loop.
 * @param [in] name: Profile string.
 * @return ReplayProfile.
 */
ReplayProfile ParseReplayProfile(const std::string& name);

/**
 * @brief Whether this topic should be skipped for the given profile.
 * @param [in] profile: Replay profile.
 * @param [in] topic: Topic name.
 * @return True if the player must not republish the topic.
 */
bool SkipReplayTopic(ReplayProfile profile, const std::string& topic);

/**
 * @brief SIL: republish MCAP payloads as RawMessage on the original topics.
 *
 * rate=1 follows log_time realtime; rate=0 is as-fast-as-possible.
 *
 * @param [in] node: Autolink node used to advertise.
 * @param [in] mcap_path: Recorded MCAP.
 * @param [in] rate: Playback rate (1 realtime, 0 ASAP).
 * @param [in] profile: Topic filter.
 * @return True if playback completed without a hard I/O error.
 */
bool PlayMcap(const std::shared_ptr<autolink::Node>& node,
              const std::string& mcap_path, double rate,
              ReplayProfile profile = ReplayProfile::kOpenLoop);

/**
 * @brief Extract /cmd_vel ActionEvent sequence from an MCAP.
 * @param [in] mcap_path: MCAP path.
 * @return Action list (may be empty).
 */
std::vector<proto::ActionEvent> ActionsFromMcap(const std::string& mcap_path);

/**
 * @brief L6 verdict without publishing: recorded vs replayed MCAP action sequences.
 * @param [in] recorded_mcap: Capture MCAP.
 * @param [in] replayed_mcap: Replay MCAP.
 * @param [in] match_threshold: Action-match pass threshold.
 * @param [in] factual: True → FactualReplay; false → InterventionalReplay.
 * @return ReplayReport.
 */
proto::ReplayReport CompareMcaps(const std::string& recorded_mcap,
                                 const std::string& replayed_mcap,
                                 double match_threshold, bool factual);

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
