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

/**
 * @file relocalization_pool_config.hpp
 * @brief Worker-pool settings for async relocalization BnB.
 */

#ifndef ARGUS_RELOCALIZATION_RELOCALIZATION_POOL_CONFIG_HPP_
#define ARGUS_RELOCALIZATION_RELOCALIZATION_POOL_CONFIG_HPP_

namespace argus {
namespace relocalization {

/** @brief Thread-pool knobs for RelocalizationThreadPool. */
struct RelocalizationPoolConfig {
    RelocalizationPoolConfig();

    int worker_count = 4;
    int max_queue_size = 256;
    bool enable_low_priority = true;
    /** @brief When queue is full: drop oldest pending task (default). */
    enum class OnFull { kDropOldest, kReject };
    OnFull on_full = OnFull::kDropOldest;
};

}  // namespace relocalization
}  // namespace argus

#endif  // ARGUS_RELOCALIZATION_RELOCALIZATION_POOL_CONFIG_HPP_
