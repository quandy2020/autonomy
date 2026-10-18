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

#include "argus/relocalization/bbs_search_engine.hpp"

#include <utility>

namespace argus {
namespace relocalization {

BbsSearchEngine::BbsSearchEngine(PriorOccupancyMap::ConstSharedPtr map,
                                 BranchAndBoundConfig branch_and_bound_config)
    : map_(std::move(map)),
      branch_and_bound_config_(std::move(branch_and_bound_config)) {}

void BbsSearchEngine::SetMap(PriorOccupancyMap::ConstSharedPtr map) {
    map_ = std::move(map);
}

RelocResult BbsSearchEngine::Search(const RelocRequest& request,
                                    std::atomic<bool>* cancel) {
    if (!map_) {
        RelocResult empty;
        empty.request_id = request.id;
        return empty;
    }
    BranchAndBound bnb(map_, branch_and_bound_config_);
    return bnb.Search(request, cancel);
}

}  // namespace relocalization
}  // namespace argus
