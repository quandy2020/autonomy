/*
 * Copyright 2026 The Openbot Authors
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

//! backend/lidar_loop_detector — stub for lidar NDT/scan-context loop (P2).
//! TODO(atlas-lio): port lightning-lm NDT loop candidate search + geometric
//! verification; wire into LoopClosing alongside vision BoW detector.

#include "autonomy/localization/atlas/type.hpp"

namespace autonomy::localization::atlas {
namespace backend {

class LidarLoopDetector {
public:
    struct Options {
        double candidate_radius_m = 15.0;
        double ndt_score_thresh = 0.5;
    };

    LidarLoopDetector() = default;
    explicit LidarLoopDetector(Options options)
        : options_(std::move(options)) {}

    //! Stub: always false until lightning NDT loop is ported.
    [[nodiscard]] bool Detect(const Mat44_t& /*T_wb*/) const {
        (void)options_;
        return false;
    }

    const Options& options() const { return options_; }

private:
    Options options_;
};

}  // namespace backend
}  // namespace autonomy::localization::atlas
