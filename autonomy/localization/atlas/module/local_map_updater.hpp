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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MODULE_LOCAL_MAP_UPDATER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MODULE_LOCAL_MAP_UPDATER_HPP_

#include <memory>

namespace autonomy::localization::atlas {

namespace data {
class frame;
class keyframe;
class landmark;
} // namespace data

namespace module {

class local_map_updater {
public:
    using keyframe_to_num_shared_lms_t = nondeterministic::unordered_map<std::shared_ptr<data::keyframe>, unsigned int>;

    //! Constructor
    explicit local_map_updater(const unsigned int max_num_local_keyfrms);

    //! Destructor
    ~local_map_updater() = default;

    //! Get the local keyframes
    std::vector<std::shared_ptr<data::keyframe>> get_local_keyframes() const;

    //! Get the local landmarks
    std::vector<std::shared_ptr<data::landmark>> get_local_landmarks() const;

    //! Get the nearest covisibility
    std::shared_ptr<data::keyframe> get_nearest_covisibility() const;

    //! Acquire the new local map
    bool acquire_local_map(const std::vector<std::shared_ptr<data::landmark>>& frm_lms);
    bool acquire_local_map(const std::vector<std::shared_ptr<data::landmark>>& frm_lms,
                           unsigned int keyframe_id_threshold,
                           unsigned int& num_temporal_keyfrms);

private:
    //! Find the local keyframes
    bool find_local_keyframes(const std::vector<std::shared_ptr<data::landmark>>& frm_lms,
                              unsigned int keyframe_id_threshold,
                              unsigned int& num_temporal_keyfrms);

    //! Count the number of shared landmarks between the current frame and each of the neighbor keyframes
    auto count_num_shared_lms(
        const std::vector<std::shared_ptr<data::landmark>>& frm_lms,
        unsigned int keyframe_id_threshold) const
        -> std::vector<std::pair<unsigned int, std::shared_ptr<data::keyframe>>>;

    //! Find the first-order local keyframes
    auto find_first_local_keyframes(
        const std::vector<std::pair<unsigned int, std::shared_ptr<data::keyframe>>>& keyfrm_weights,
        const unsigned int keyframe_id_threshold,
        std::unordered_set<unsigned int>& already_found_keyfrm_ids,
        unsigned int& num_temporal_keyfrms)
        -> std::vector<std::shared_ptr<data::keyframe>>;

    //! Find the second-order local keyframes
    auto find_second_local_keyframes(const std::vector<std::shared_ptr<data::keyframe>>& first_local_keyframes,
                                     unsigned int keyframe_id_threshold,
                                     std::unordered_set<unsigned int>& already_found_keyfrm_ids,
                                     unsigned int& num_temporal_keyfrms) const
        -> std::vector<std::shared_ptr<data::keyframe>>;

    //! Find the local landmarks
    bool find_local_landmarks(const std::vector<std::shared_ptr<data::landmark>>& frm_lms);

    // maximum number of the local keyframes
    const unsigned int max_num_local_keyfrms_;

    // found local keyframes
    std::vector<std::shared_ptr<data::keyframe>> local_keyfrms_;
    // found local landmarks
    std::vector<std::shared_ptr<data::landmark>> local_lms_;
    // the nearst keyframe in covisibility graph, which will be found in find_first_local_keyframes()
    std::shared_ptr<data::keyframe> nearest_covisibility_;
};

} // namespace module
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MODULE_LOCAL_MAP_UPDATER_HPP_
