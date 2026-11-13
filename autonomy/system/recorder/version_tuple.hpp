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
 * @file version_tuple.hpp
 * @brief Deterministic version quadruple: policy_version / env_version / git_sha / seed.
 *
 * Missing any field sets admitted=false; the episode goes to the diagnostic
 * queue, not training or replay.
 */

#include <string>
#include <vector>

#include "autonomy/system/recorder/proto/evidence.pb.h"
#include "autonomy/system/recorder/proto/recorder_options.pb.h"

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief Result of assembling and checking the version quadruple.
 */
struct VersionCheck {
    proto::VersionTuple tuple;         ///< Populated fields.
    bool admitted{false};              ///< True iff all four fields are set.
    std::vector<std::string> missing;  ///< Names of empty fields.
};

/**
 * @brief Build the quadruple from options and environment.
 *
 * policy_version may fall back to AUTONOMY_SOFTWARE_VERSION.
 *
 * @param [in] options: RecorderOptions.
 * @return Check result.
 */
VersionCheck MakeVersionTuple(const proto::RecorderOptions& options);

/**
 * @brief True when every field of the quadruple is non-empty.
 * @param [in] tuple: Version quadruple.
 * @return True if admitted.
 */
bool IsAdmitted(const proto::VersionTuple& tuple);

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
