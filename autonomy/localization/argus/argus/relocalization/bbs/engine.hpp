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
 * @file engine.hpp
 * @brief Reloc search: optional SC++ crop → BranchAndBound → RefineSE3.
 */

#ifndef ARGUS_RELOCALIZATION_BBS_ENGINE_HPP_
#define ARGUS_RELOCALIZATION_BBS_ENGINE_HPP_

#include <atomic>
#include <memory>

#include "argus/common/macros.hpp"
#include "argus/relocalization/bbs/search.hpp"
#include "argus/relocalization/candidate.hpp"
#include "argus/relocalization/prior_map.hpp"
#include "argus/relocalization/scan_context/scan_context.hpp"
#include "argus/relocalization/types.hpp"

namespace argus {
namespace relocalization {

/**
 * @brief Occupancy prior + hierarchical pose search.
 * @note Optional cascade: aux candidates → SC++ gate/retrieve → cropped BnB.
 */
class BbsSearchEngine : public RelocalizationBase
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(BbsSearchEngine);

    /**
     * @brief Construct with optional prior map and search settings.
     * @param map Occupancy prior (may be null until SetMap).
     * @param branch_and_bound_config BnB parameters.
     * @param place_recognition SC++ crop settings.
     */
    BbsSearchEngine(PriorOccupancyMap::ConstSharedPtr map,
                    BranchAndBoundConfig branch_and_bound_config = {},
                    PlaceRecognitionConfig place_recognition = {});

    /**
     * @brief Replace prior and rebuild the place index from voxel centers.
     * @param map New occupancy prior.
     */
    void SetMap(PriorOccupancyMap::ConstSharedPtr map);

    /**
     * @brief Replace prior and rebuild place index from `cloud_w`.
     * @param map Occupancy prior.
     * @param cloud_w Preferred world cloud for SC++ indexing.
     */
    void SetMap(PriorOccupancyMap::ConstSharedPtr map,
                const sensor::PointCloudXYZI& cloud_w);

    /**
     * @brief Rebuild place index from an explicit world cloud.
     * @param cloud_w World-frame prior points.
     */
    void BuildPlaceIndexFromCloud(const sensor::PointCloudXYZI& cloud_w);

    /**
     * @brief Update SC++ settings and rebuild the index if a map is set.
     * @param config New place-recognition config.
     */
    void SetPlaceRecognitionConfig(PlaceRecognitionConfig config);

    /**
     * @brief Optional upstream candidate source (e.g. visual).
     * @param source Shared source; nullptr clears. Seeds are SC-gated when the
     *        place index is non-empty.
     */
    void SetAuxCandidateSource(PlaceCandidateSource::SharedPtr source);

    /** @return True if an occupancy prior is set. */
    bool HasMap() const {
        return static_cast<bool>(map_);
    }

    /** @return SC++ place-index size (`0` if disabled / empty). */
    int place_index_size() const;

    /**
     * @brief Run pose search for one request.
     * @param request Query cloud and optional pose guess.
     * @param cancel Cooperative cancel flag (may be null).
     * @return Best pose / score / status.
     */
    RelocResult Search(const RelocRequest& request,
                       std::atomic<bool>* cancel) override;

private:
    void RebuildPlaceIndexLocked();
    std::vector<PlaceCandidate> CollectCandidates(
        const sensor::PointCloudXYZI& query_cloud_b) const;
    std::vector<PlaceCandidate> GateWithScanContext(
        const sensor::PointCloudXYZI& query_cloud_b,
        const std::vector<PlaceCandidate>& seeds) const;
    RelocResult SearchFull(const RelocRequest& request,
                           std::atomic<bool>* cancel,
                           const BranchAndBoundConfig& bnb_config) const;
    RelocResult SearchWithPlaceCrop(const RelocRequest& request,
                                    std::atomic<bool>* cancel) const;

    PriorOccupancyMap::ConstSharedPtr map_;
    BranchAndBoundConfig branch_and_bound_config_;
    PlaceRecognitionConfig place_recognition_;
    ScanContextPlaceIndex place_index_;
    PlaceCandidateSource::SharedPtr aux_source_;
    sensor::PointCloudXYZI preferred_index_cloud_;
};

}  // namespace relocalization
}  // namespace argus

#endif  // ARGUS_RELOCALIZATION_BBS_ENGINE_HPP_
