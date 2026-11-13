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
 * @file observation.hpp
 * @brief L2 observation gate: only observations bound to an evidence_node_id are kept.
 */

#include "autonomy/system/recorder/proto/evidence.pb.h"

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief True iff the observation binds a non-empty evidence_node_id.
 *
 * Unbound observations must not enter L2 hypotheses.
 *
 * @param [in] observation: Candidate observation.
 * @return True if admissible.
 */
bool AcceptObservation(const proto::Observation& observation);

/**
 * @brief Append one observation as a JSONL line.
 * @param [in] path: JSONL path (created if missing).
 * @param [in] observation: Observation to append.
 * @return True on success.
 */
bool AppendObservationJsonl(const std::string& path,
                            const proto::Observation& observation);

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
