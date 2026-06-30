/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_DATA_BOW_DATABASE_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_DATA_BOW_DATABASE_HPP_

#include "autonomy/localization/atlas/data/bow_vocabulary.hpp"

#include <mutex>
#include <list>
#include <vector>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <memory>

namespace autonomy::localization::atlas {
namespace data {

class frame;
class keyframe;

class bow_database {
public:
    /**
     * Constructor
     * @param bow_vocab
     */
    explicit bow_database(bow_vocabulary* bow_vocab);

    /**
     * Destructor
     */
    ~bow_database();

    /**
     * Add a keyframe to the database
     * @param keyfrm
     */
    void add_keyframe(const std::shared_ptr<keyframe>& keyfrm);

    /**
     * Erase the keyframe from the database
     * @param keyfrm
     */
    void erase_keyframe(const std::shared_ptr<keyframe>& keyfrm);

    /**
     * Clear the database
     */
    void clear();

    /**
     * Acquire keyframes over score
     */
    std::vector<std::shared_ptr<keyframe>> acquire_keyframes(const bow_vector& bow_vec, const float min_score = 0.0f,
                                                             const float num_common_words_thr_ratio = 0.8f,
                                                             const std::set<std::shared_ptr<keyframe>>& keyfrms_to_reject = {});

protected:
    /**
     * Initialize temporary variables
     */
    void initialize();

    /**
     * Compute the number of shared words
     * @param bow_vec
     * @param keyfrms_to_reject
     * @return number of shared words between the query and the each of keyframes contained in the database (key: keyframes that share word with query keyframe, value: number of shared words)
     */
    std::unordered_map<std::shared_ptr<keyframe>, unsigned int>
    compute_num_common_words(const bow_vector& bow_vec,
                             const std::set<std::shared_ptr<keyframe>>& keyfrms_to_reject = {}) const;

    /**
     * Compute scores (scores_) between the query and the each of keyframes contained in the database
     * @param num_common_words
     * @param bow_vec
     * @param min_num_common_words_thr
     * @return similarity scores between the query and the each of keyframes contained in the database (key: keyframes that share word with query keyframe, value: score)
     */
    std::unordered_map<std::shared_ptr<keyframe>, float>
    compute_scores(const std::unordered_map<std::shared_ptr<keyframe>, unsigned int>& num_common_words,
                   const bow_vector& bow_vec,
                   const unsigned int min_num_common_words_thr,
                   const float min_score,
                   float& best_score) const;

    //-----------------------------------------
    // BoW feature vectors

    //! mutex to access BoW database
    mutable std::mutex mtx_;
    //! BoW database
    std::unordered_map<unsigned int, std::list<std::shared_ptr<keyframe>>> keyfrms_in_node_;

    //-----------------------------------------
    // BoW vocabulary

    //! BoW vocabulary
    bow_vocabulary* bow_vocab_;
};

} // namespace data
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_DATA_BOW_DATABASE_HPP_
