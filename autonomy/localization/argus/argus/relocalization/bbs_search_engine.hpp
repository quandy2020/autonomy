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
 * @file bbs_search_engine.hpp
 * @brief 3D-BBS-style engine: occupancy prior + BranchAndBound.
 */

#ifndef ARGUS_RELOCALIZATION_BBS_SEARCH_ENGINE_HPP_
#define ARGUS_RELOCALIZATION_BBS_SEARCH_ENGINE_HPP_

#include <atomic>
#include <memory>

#include "argus/common/macros.hpp"
#include "argus/relocalization/branch_and_bound.hpp"
#include "argus/relocalization/prior_map_loader.hpp"
#include "argus/relocalization/relocalization_base.hpp"

namespace argus {
namespace relocalization {

/**
 * @brief Relocalization engine wrapping `BranchAndBound` on a prior map.
 */
class BbsSearchEngine : public RelocalizationBase
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(BbsSearchEngine);

    /**
     * @brief Construct with an optional prior map.
     * @param map Shared occupancy prior; may be null until SetMap.
     * @param branch_and_bound_config Branch-and-bound search parameters.
     */
    BbsSearchEngine(PriorOccupancyMap::ConstSharedPtr map,
                    BranchAndBoundConfig branch_and_bound_config = {});

    /**
     * @brief Replace the occupancy prior used for scoring.
     * @param map New prior map (shared).
     */
    void SetMap(PriorOccupancyMap::ConstSharedPtr map);

    /**
     * @brief Whether a prior map is currently set.
     * @return True if `map_` is non-null.
     */
    bool HasMap() const {
        return static_cast<bool>(map_);
    }

    /**
     * @brief Run hierarchical pose search for one request.
     * @param request Query cloud and optional pose guess.
     * @param cancel Optional cooperative cancel flag.
     * @return Best pose / score / status.
     */
    RelocResult Search(const RelocRequest& request,
                       std::atomic<bool>* cancel) override;

private:
    PriorOccupancyMap::ConstSharedPtr map_;
    BranchAndBoundConfig branch_and_bound_config_;
};

}  // namespace relocalization
}  // namespace argus

#endif  // ARGUS_RELOCALIZATION_BBS_SEARCH_ENGINE_HPP_
