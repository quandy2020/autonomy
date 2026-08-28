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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_GLOBAL_BUNDLE_ADJUSTER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_GLOBAL_BUNDLE_ADJUSTER_HPP_

namespace autonomy::localization::atlas {

namespace data {
class map_database;
} // namespace data

namespace optimize {

class global_bundle_adjuster {
public:
    explicit global_bundle_adjuster(data::map_database* map_db = nullptr,
                                    unsigned int num_iter = 10,
                                    bool use_huber_kernel = true,
                                    bool verbose = false);

    virtual ~global_bundle_adjuster() = default;

    void optimize_for_initialization(const std::vector<std::shared_ptr<data::keyframe>>& keyfrms,
                                     const std::vector<std::shared_ptr<data::landmark>>& lms,
                                     const std::vector<std::shared_ptr<data::marker>>& markers,
                                     float gain_threshold,
                                     bool fix_markers,
                                     bool* const force_stop_flag = nullptr) const;

    bool optimize(const std::vector<std::shared_ptr<data::keyframe>>& keyfrms,
                  std::unordered_set<unsigned int>& optimized_keyfrm_ids,
                  std::unordered_set<unsigned int>& optimized_landmark_ids,
                  std::unordered_set<unsigned int>& optimized_landmark_line_ids,
                  std::unordered_set<unsigned int>& optimized_marker_ids,
                  eigen_alloc_unord_map<unsigned int, Vec3_t>& lm_to_pos_w_after_global_BA,
                  eigen_alloc_unord_map<unsigned int, Vec6_t>& lm_line_to_plucker_after_global_BA,
                  eigen_alloc_unord_map<unsigned int, Mat44_t>& keyfrm_to_pose_cw_after_global_BA,
                  eigen_alloc_unord_map<unsigned int, std::array<Vec3_t, 4>>& marker_to_pos_w_after_global_BA,
                  bool* const force_stop_flag = nullptr) const;

private:
    data::map_database* map_db_ = nullptr;
    unsigned int num_iter_;
    const bool use_huber_kernel_;
    const bool verbose_ = false;
};

} // namespace optimize
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_GLOBAL_BUNDLE_ADJUSTER_HPP_
