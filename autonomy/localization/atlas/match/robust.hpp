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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MATCH_ROBUST_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MATCH_ROBUST_HPP_

#include "autonomy/localization/atlas/type.hpp"
#include "autonomy/localization/atlas/match/base.hpp"

#include <memory>

namespace autonomy::localization::atlas {

namespace data {
class frame;
struct frame_observation;
class keyframe;
class landmark;
} // namespace data

namespace match {

class robust final : public base {
public:
    explicit robust(const float lowe_ratio, const bool check_orientation)
        : base(lowe_ratio, check_orientation) {}

    ~robust() final = default;

    unsigned int match_for_triangulation(const std::shared_ptr<data::keyframe>& keyfrm_1,
                                         const std::shared_ptr<data::keyframe>& keyfrm_2,
                                         const Mat33_t& E_12,
                                         std::vector<std::pair<unsigned int, unsigned int>>& matched_idx_pairs,
                                         const float residual_rad_thr) const;

    unsigned int match_keyframes(const std::shared_ptr<data::keyframe>& keyfrm1, const std::shared_ptr<data::keyframe>& keyfrm2,
                                 std::vector<std::shared_ptr<data::landmark>>& matched_lms_in_frm,
                                 bool validate_with_essential_solver = true, bool use_fixed_seed = false) const;

    unsigned int match_frame_and_keyframe(data::frame& frm, const std::shared_ptr<data::keyframe>& keyfrm,
                                          std::vector<std::shared_ptr<data::landmark>>& matched_lms_in_frm,
                                          bool use_fixed_seed = false) const;

    unsigned int brute_force_match(const data::frame_observation& frm_obs, const std::shared_ptr<data::keyframe>& keyfrm, std::vector<std::pair<int, int>>& matches) const;
};

} // namespace match
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MATCH_ROBUST_HPP_
