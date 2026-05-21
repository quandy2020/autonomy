/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include <vector>

#include "autonomy/control/proto/tdmpc_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace tdmpc {
namespace topology {

/** T-MPC++ style parallel topology: lateral offsets along the path normal. */
struct TopologyCandidate {
    int id{0};
    double lateral_offset{0.0};
    bool is_baseline{false};
};

class TopologyManager
{
public:
    explicit TopologyManager(const proto::TdmpcControllerOptions& options);

    const std::vector<TopologyCandidate>& candidates() const {
        return candidates_;
    }

private:
    std::vector<TopologyCandidate> candidates_;
};

}  // namespace topology
}  // namespace tdmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
