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
 * @file snapshot_writer.hpp
 * @brief Flush ring window to MCAP + sidecar.pb. Capture never blocks control.
 */

#include <cstdint>
#include <string>
#include <vector>

#include "common/base/status.hpp"
#include "robot/data_agent/snapshot_writer/snapshot_config.hpp"
#include "robot/data_agent/trigger_manager/trigger_types.hpp"
#include "robot/data_agent/ring_buffer/ring_buffer.hpp"
#include "robot/data_agent/version_stamp/version_tuple.hpp"

namespace autoloop {
namespace robot {

class SnapshotWriter {
public:
    explicit SnapshotWriter(SnapshotConfig cfg);

    /**
     * @brief Write one episode from a copied window.
     * @param [in] samples: Ring snapshot (copy, not TakeAll unless flushing).
     * @param [in] trigger: Firing rule.
     * @param [in] version: Quadruple check.
     * @param [out] episode_dir: Created directory.
     */
    common::Status Write(const std::vector<Sample>& samples,
                         const TriggerEvent& trigger,
                         const VersionCheck& version,
                         std::string* episode_dir);

    uint64_t snapshots_written() const { return written_; }

private:
    SnapshotConfig cfg_;
    uint64_t written_{0};
};

}  // namespace robot
}  // namespace autoloop
