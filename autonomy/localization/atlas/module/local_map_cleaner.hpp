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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MODULE_LOCAL_MAP_CLEANER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MODULE_LOCAL_MAP_CLEANER_HPP_

#include <list>
#include <memory>

namespace autonomy::localization::atlas {

namespace data {
class keyframe;
class landmark;
class bow_database;
class map_database;
} // namespace data

namespace module {

class local_map_cleaner {
public:
    /**
     * Constructor
     */
    explicit local_map_cleaner(const YAML::Node& yaml_node, data::map_database* map_db, data::bow_database* bow_db);

    /**
     * Destructor
     */
    ~local_map_cleaner() = default;

    /**
     * Add fresh landmark to check their redundancy
     */
    void add_fresh_landmark(std::shared_ptr<data::landmark>& lm) {
        fresh_landmarks_.push_back(lm);
    }

    /**
     * Reset the buffer
     */
    void reset();

    /**
     * Remove redundant landmarks
     */
    unsigned int remove_invalid_landmarks(const unsigned int cur_keyfrm_id);

    /**
     * Remove redundant keyframes
     */
    unsigned int remove_redundant_keyframes(const std::shared_ptr<data::keyframe>& cur_keyfrm) const;

    /**
     * Count the valid and the redundant observations in the specified keyframe
     */
    void count_redundant_observations(const std::shared_ptr<data::keyframe>& keyfrm, unsigned int& num_valid_obs, unsigned int& num_redundant_obs) const;

private:
    //! map database
    data::map_database* map_db_ = nullptr;
    //! BoW database
    data::bow_database* bow_db_ = nullptr;

    //!
    double redundant_obs_ratio_thr_;

    //!
    double observed_ratio_thr_ = 0.3;

    //!
    unsigned int num_reliable_keyfrms_ = 2;

    //! Top n covisibilities to search (0 means disabled)
    unsigned int top_n_covisibilities_to_search_;

    //! fresh landmarks to check their redundancy
    std::list<std::shared_ptr<data::landmark>> fresh_landmarks_;
};

} // namespace module
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MODULE_LOCAL_MAP_CLEANER_HPP_
