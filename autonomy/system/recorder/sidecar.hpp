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
 * @file sidecar.hpp
 * @brief Episode sidecar files: JSON sidecar, manifest, replay overlay, hazard pointer.
 */

#include <string>
#include <vector>

#include "autonomy/system/recorder/proto/evidence.pb.h"
#include "autonomy/system/recorder/value_tier.hpp"

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief Write EpisodeSidecar as pretty JSON next to the MCAP.
 * @param [in] path: Output .json / .sidecar.json path.
 * @param [in] sidecar: Sidecar proto.
 * @return True on success.
 */
bool WriteSidecarJson(const std::string& path,
                      const proto::EpisodeSidecar& sidecar);

/**
 * @brief Write the episode manifest (header + upload decision).
 * @param [in] path: Output .manifest.json path.
 * @param [in] header: Episode header.
 * @param [in] upload: P0–P3 upload decision.
 * @return True on success.
 */
bool WriteManifest(const std::string& path, const proto::EpisodeHeader& header,
                   const UploadDecision& upload);

/**
 * @brief Write a replay overlay JSON for autoviz.
 * @param [in] path: Output .overlay.json path.
 * @param [in] report: L6 replay report.
 * @return True on success.
 */
bool WriteReplayOverlay(const std::string& path,
                        const proto::ReplayReport& report);

/**
 * @brief Load observations from a JSONL file (skips malformed lines).
 * @param [in] path: JSONL path.
 * @return Observations that parsed.
 */
std::vector<proto::Observation> LoadObservationsJsonl(const std::string& path);

/**
 * @brief Write a P0 hazard pointer under $AUTONOMY_LOG_ROOT/events/recorder_<id>/.
 * @param [in] log_root: AUTONOMY_LOG_ROOT (or equivalent).
 * @param [in] episode_id: Episode id.
 * @param [in] mcap_path: Flushed MCAP path.
 * @param [in] sidecar_json: Sidecar JSON path.
 * @return True on success.
 */
bool WriteHazardPointer(const std::string& log_root,
                        const std::string& episode_id,
                        const std::string& mcap_path,
                        const std::string& sidecar_json);

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
