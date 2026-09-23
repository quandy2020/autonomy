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
 *
 * KeyFrameDatabase adapted from ORB-SLAM3, scoring via FBoW BoWVector::score.
 */

/**
 * @file keyframe_database.hpp
 * @brief Keyframe inverted-index database: relocalization / loop / merge
 *        candidate retrieval.
 *
 * Corresponds to ORB-SLAM3 `KeyFrameDatabase`; similarity via FBoW
 * `BoWVector::score`.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MAP_KEYFRAME_DATABASE_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MAP_KEYFRAME_DATABASE_HPP_

#include <list>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "autonomy/localization/atlas/frontend/feature/orb/orb_vocabulary.hpp"
#include "autonomy/localization/atlas/frontend/tracking/frame.hpp"
#include "autonomy/localization/atlas/map/keyframe.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

/**
 * @class autonomy::localization::atlas::KeyFrameDatabase
 * @brief Bag-of-words inverted file: retrieve candidate keyframes by BoW co-occurrence.
 *
 * Corresponds to ORB-SLAM3 `KeyFrameDatabase` + FBoW. Keyframes must have
 * `ComputeBoW()` / `HasBoW()` before insert.
 *
 * @note Loop candidates
 * @code{.cpp}
 * KeyFrameDatabase db(vocab);
 * db.Add(kf);
 * auto cands = db.DetectLoopCandidates(query_kf, min_score);
 * @endcode
 */
class KeyFrameDatabase {
public:
    /**
     * @brief Construct an empty inverted file bound to a vocabulary.
     * @param vocabulary ORB/FBoW vocabulary.
     */
    explicit KeyFrameDatabase(
        std::shared_ptr<feature::OrbVocabulary> vocabulary);

    /**
     * @brief Insert a keyframe into the inverted file by BoW words.
     * @param keyframe Must already have BoW; otherwise ignored.
     */
    void Add(const std::shared_ptr<KeyFrame>& keyframe);
    /**
     * @brief Remove a keyframe from the inverted file.
     * @param keyframe Keyframe.
     */
    void Erase(const std::shared_ptr<KeyFrame>& keyframe);
    /** @brief Clear the inverted file. */
    void clear();

    /**
     * @brief Relocalization: retrieve candidate keyframes from the frame BoW.
     * @param frame Tracking frame (must already have BoW).
     * @return Candidate keyframe list.
     *
     * Corresponds to ORB-SLAM3 `DetectRelocalizationCandidates`.
     */
    std::vector<std::shared_ptr<KeyFrame>>
    DetectRelocalizationCandidates(const tracking::Frame& frame) const;

    /**
     * @brief Loop-detection candidates (same map, excluding covisible neighborhood).
     * @param keyframe Query keyframe.
     * @param min_score Threshold relative to the covisibility minimum score (ORB habit).
     * @return Candidate list.
     *
     * Corresponds to ORB-SLAM3 `DetectLoopCandidates`.
     */
    std::vector<std::shared_ptr<KeyFrame>> DetectLoopCandidates(
        const std::shared_ptr<KeyFrame>& keyframe, float min_score) const;

    /**
     * @brief Emit top-N loop and merge candidates (split by map identity).
     * @param keyframe Query keyframe.
     * @param[out] loop_cands Same-map loop candidates.
     * @param[out] merge_cands Cross-map merge candidates.
     * @param num_candidates Max count per class, default 3.
     *
     * Corresponds to ORB-SLAM3 `DetectNBestCandidates`: accumulate covisibility
     * scores then split by map identity.
     */
    void DetectNBestCandidates(const std::shared_ptr<KeyFrame>& keyframe,
                               std::vector<std::shared_ptr<KeyFrame>>* loop_cands,
                               std::vector<std::shared_ptr<KeyFrame>>* merge_cands,
                               int num_candidates = 3) const;

private:
    std::shared_ptr<feature::OrbVocabulary> vocabulary_;  ///< Vocabulary
    mutable std::mutex mutex_;                            ///< Inverted-file lock
    //! Word id → list of keyframes containing that word
    std::unordered_map<uint32_t, std::list<std::weak_ptr<KeyFrame>>>
        inverted_file_;
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MAP_KEYFRAME_DATABASE_HPP_
