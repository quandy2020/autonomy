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
 * @file value_tier.hpp
 * @brief P0–P3 upload decision: always upload P0/P1; sample P2/P3.
 */

#include <string>

#include "autonomy/system/recorder/proto/evidence.pb.h"

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief Upload policy for one episode.
 */
struct UploadDecision {
    proto::Priority priority{proto::PRIORITY_P3};  ///< Classified priority.
    bool upload{false};                            ///< Whether to copy off-robot.
    bool expert_demo{false};                       ///< Teleop / expert demonstration.
    bool sampled{false};                           ///< True if P2/P3 won the coin flip.
    std::string policy;                            ///< Human-readable policy tag.
};

/**
 * @brief Decide whether this episode is uploaded.
 *
 * P0/P1 always upload. P2/P3 are Bernoulli-sampled with p2_rate / p3_rate
 * using episode_id as a stable seed.
 *
 * @param [in] priority: Classified priority.
 * @param [in] episode_id: Stable seed for sampling.
 * @param [in] scene: Scene id (logged on the decision).
 * @param [in] p2_rate: Sample rate for P2 in [0, 1].
 * @param [in] p3_rate: Sample rate for P3 in [0, 1].
 * @return UploadDecision.
 */
UploadDecision DecideUpload(proto::Priority priority,
                            const std::string& episode_id,
                            const std::string& scene, double p2_rate,
                            double p3_rate);

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
