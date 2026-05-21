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

#include "autonomy/control/controller/tdmpc_controller/topology/topology_manager.hpp"

#include <algorithm>

namespace autonomy {
namespace control {
namespace controller {
namespace tdmpc {
namespace topology {

TopologyManager::TopologyManager(const proto::TdmpcControllerOptions& options) {
    const int n = options.num_topology_candidates() > 0
                      ? options.num_topology_candidates()
                      : 3;
    const double spacing = options.topology_lateral_spacing() > 0.0
                               ? options.topology_lateral_spacing()
                               : 0.35;

    candidates_.clear();
    if (options.enable_baseline_topology()) {
        candidates_.push_back({0, 0.0, true});
    }

    const int lateral_count = std::max(1, n - (options.enable_baseline_topology() ? 1 : 0));
    if (lateral_count == 1) {
        candidates_.push_back({static_cast<int>(candidates_.size()), 0.0, false});
        return;
    }

    for (int i = 0; i < lateral_count; ++i) {
        const double t =
            static_cast<double>(i) / static_cast<double>(lateral_count - 1);
        const double offset = spacing * (2.0 * t - 1.0);
        TopologyCandidate c;
        c.id = static_cast<int>(candidates_.size());
        c.lateral_offset = offset;
        c.is_baseline = false;
        candidates_.push_back(c);
    }
}

}  // namespace topology
}  // namespace tdmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
