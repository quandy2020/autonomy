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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MATCH_BOW_TREE_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MATCH_BOW_TREE_HPP_

#include "autonomy/localization/atlas/match/base.hpp"

#include <memory>

namespace autonomy::localization::atlas {

namespace data {
class frame;
class keyframe;
class landmark;
} // namespace data

namespace match {

class bow_tree final : public base {
public:
    explicit bow_tree(const float lowe_ratio = 0.6, const bool check_orientation = true)
        : base(lowe_ratio, check_orientation) {}

    ~bow_tree() final = default;

    unsigned int match_for_triangulation(const std::shared_ptr<data::keyframe>& keyfrm_1,
                                         const std::shared_ptr<data::keyframe>& keyfrm_2,
                                         const Mat33_t& E_12,
                                         std::vector<std::pair<unsigned int, unsigned int>>& matched_idx_pairs,
                                         const float residual_rad_thr) const;

    //! Find the correspondence between the feature points observed in the frame and the feature points observed in the keyframe,
    //! and obtain the correspondence between the feature points in the frame and the 3D points.
    //! Matched_lms_in_frm stores the 3D points (observed in the keyframe) corresponding to each feature point in the frame.
    //! NOTE: matched_lms_in_frm.size() is equal to the number of feature points in the frame
    unsigned int match_frame_and_keyframe(const std::shared_ptr<data::keyframe>& keyfrm, data::frame& frm, std::vector<std::shared_ptr<data::landmark>>& matched_lms_in_frm) const;

    //! Find the correspondence between the feature point observed in keyframe1 and the feature point observed in keyframe2,
    //! and obtain the correspondence between the feature point in keyframe1 and the 3D points.
    //! Matched_lms_in_keyfrm_1 stores the 3D points (observed in keyframe2) corresponding to each feature point in keyframe1
    //! NOTE: matched_lms_in_keyfrm_1.size() matches the number of feature points in keyframe1
    unsigned int match_keyframes(const std::shared_ptr<data::keyframe>& keyfrm_1, const std::shared_ptr<data::keyframe>& keyfrm_2, std::vector<std::shared_ptr<data::landmark>>& matched_lms_in_keyfrm_1) const;
};

} // namespace match
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MATCH_BOW_TREE_HPP_
