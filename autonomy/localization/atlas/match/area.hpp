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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MATCH_AREA_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MATCH_AREA_HPP_

#include "autonomy/localization/atlas/match/base.hpp"

namespace autonomy::localization::atlas {

namespace data {
class frame;
} // namespace data

namespace match {

class area final : public base {
public:
    area(const float lowe_ratio, const bool check_orientation)
        : base(lowe_ratio, check_orientation) {}

    ~area() final = default;

    unsigned int match_in_consistent_area(data::frame& frm_1, data::frame& frm_2, std::vector<cv::Point2f>& prev_matched_pts,
                                          std::vector<int>& matched_indices_2_in_frm_1, int margin = 10);
};

} // namespace match
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MATCH_AREA_HPP_
