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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_TRACKING_VISUAL_INITIAL_MAPPING_BARRIER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_TRACKING_VISUAL_INITIAL_MAPPING_BARRIER_HPP_

#include <atomic>

namespace autonomy::localization::atlas::internal {

constexpr bool ShouldUseVisualInitialMappingBarrier(bool is_monocular,
                                                    bool is_imu_rgbd) {
    return is_monocular || is_imu_rgbd;
}

/** One-shot event emitted after visual initial keyframe insertion. */
class VisualInitialMappingBarrier final
{
public:
    void MarkPending() {
        pending_.store(true, std::memory_order_release);
    }

    bool Consume() {
        return pending_.exchange(false, std::memory_order_acq_rel);
    }

    void Clear() {
        pending_.store(false, std::memory_order_release);
    }

private:
    std::atomic<bool> pending_{false};
};

}  // namespace autonomy::localization::atlas::internal

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_TRACKING_VISUAL_INITIAL_MAPPING_BARRIER_HPP_
