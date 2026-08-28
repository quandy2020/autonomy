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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MATCH_FUSE_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MATCH_FUSE_HPP_

#include "autonomy/localization/atlas/type.hpp"
#include "autonomy/localization/atlas/match/base.hpp"

#include <memory>

namespace autonomy::localization::atlas {

namespace data {
class keyframe;
class landmark;
class landmark_line;
class map_database;
} // namespace data

namespace match {

class fuse final {
public:
    explicit fuse(float lowe_ratio)
        : lowe_ratio_(lowe_ratio) {}

    virtual ~fuse() = default;

    //! 3次元点(landmarks_to_check)をkeyframeに再投影し，keyframeで観測している3次元点と重複しているものを探す
    template<typename T>
    unsigned int detect_duplication(const std::shared_ptr<data::keyframe>& keyfrm,
                                    const Mat33_t& rot_cw,
                                    const Vec3_t& trans_cw,
                                    const T& landmarks_to_check,
                                    const float margin,
                                    std::unordered_map<std::shared_ptr<data::landmark>, std::shared_ptr<data::landmark>>& duplicated_lms_in_keyfrm,
                                    std::unordered_map<unsigned int, std::shared_ptr<data::landmark>>& new_connections,
                                    bool do_reprojection_matching = false) const;

    template<typename T>
    unsigned int replace_duplication_line(const std::shared_ptr<data::keyframe>& keyfrm,
                                          const T& landmarks_to_check,
                                          float margin) const;

protected:
    const float lowe_ratio_;
};

} // namespace match
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MATCH_FUSE_HPP_
