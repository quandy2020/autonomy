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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MODULE_RELOCALIZER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MODULE_RELOCALIZER_HPP_

#include "autonomy/localization/atlas/match/bow_tree.hpp"
#include "autonomy/localization/atlas/match/projection.hpp"
#include "autonomy/localization/atlas/match/robust.hpp"
#include "autonomy/localization/atlas/optimize/pose_optimizer.hpp"
#include "autonomy/localization/atlas/solve/pnp_solver.hpp"

#include <memory>

namespace autonomy::localization::atlas {

namespace data {
class frame;
class bow_database;
} // namespace data

namespace module {

class relocalizer {
public:
    //! Constructor
    explicit relocalizer(const std::shared_ptr<optimize::pose_optimizer>& pose_optimizer,
                         const double bow_match_lowe_ratio = 0.75, const double proj_match_lowe_ratio = 0.9,
                         const double robust_match_lowe_ratio = 0.8,
                         const unsigned int min_num_bow_matches = 20, const unsigned int min_num_valid_obs = 50,
                         const bool use_fixed_seed = false,
                         const bool search_neighbor = true,
                         const unsigned int top_n_covisibilities_to_search = 10,
                         const float num_common_words_thr_ratio = 0.8f,
                         const unsigned int max_num_ransac_iter = 30,
                         const unsigned int max_num_local_keyfrms = 60);

    explicit relocalizer(const std::shared_ptr<optimize::pose_optimizer>& pose_optimizer, const YAML::Node& yaml_node);

    //! Destructor
    virtual ~relocalizer();

    //! Relocalize the specified frame
    bool relocalize(data::bow_database* bow_db, data::frame& curr_frm);

    //! Relocalize the specified frame by given candidates list
    bool reloc_by_candidates(data::frame& curr_frm,
                             const std::vector<std::shared_ptr<autonomy::localization::atlas::data::keyframe>>& reloc_candidates,
                             bool use_robust_matcher = false);
    bool reloc_by_candidate(data::frame& curr_frm,
                            const std::shared_ptr<autonomy::localization::atlas::data::keyframe>& candidate_keyfrm,
                            bool use_robust_matcher);
    bool relocalize_by_pnp_solver(data::frame& curr_frm,
                                  const std::shared_ptr<autonomy::localization::atlas::data::keyframe>& candidate_keyfrm,
                                  bool use_robust_matcher,
                                  std::vector<unsigned int>& inlier_indices,
                                  std::vector<std::shared_ptr<data::landmark>>& matched_landmarks) const;
    bool optimize_pose(data::frame& curr_frm,
                       const std::shared_ptr<autonomy::localization::atlas::data::keyframe>& candidate_keyfrm,
                       std::vector<bool>& outlier_flags) const;
    bool refine_pose(data::frame& curr_frm,
                     const std::shared_ptr<autonomy::localization::atlas::data::keyframe>& candidate_keyfrm,
                     const std::set<std::shared_ptr<data::landmark>>& already_found_landmarks) const;
    bool refine_pose_by_local_map(data::frame& curr_frm,
                                  const std::shared_ptr<autonomy::localization::atlas::data::keyframe>& candidate_keyfrm) const;

private:
    //! Extract valid (non-deleted) landmarks from landmark vector
    std::vector<unsigned int> extract_valid_indices(const std::vector<std::shared_ptr<data::landmark>>& landmarks) const;

    //! Setup PnP solver with the specified 2D-3D matches
    std::unique_ptr<solve::pnp_solver> setup_pnp_solver(const std::vector<unsigned int>& valid_indices,
                                                        const eigen_alloc_vector<Vec3_t>& bearings,
                                                        const std::vector<cv::KeyPoint>& keypts,
                                                        const std::vector<std::shared_ptr<data::landmark>>& matched_landmarks,
                                                        const std::vector<float>& scale_factors) const;

    //! minimum threshold of the number of BoW matches
    const unsigned int min_num_bow_matches_;
    //! minimum threshold of the number of valid (= inlier after pose optimization) matches
    const unsigned int min_num_valid_obs_;

    //! BoW matcher
    const match::bow_tree bow_matcher_;
    //! projection matcher
    const match::projection proj_matcher_;
    //! robust matcher
    const match::robust robust_matcher_;
    //! pose optimizer
    std::shared_ptr<optimize::pose_optimizer> pose_optimizer_ = nullptr;

    //! Use fixed random seed for RANSAC if true
    const bool use_fixed_seed_ = false;

    //! If true, points used by the PnP solver are searched not only from candidate keyframes, but also from neighbor keyframes
    const bool search_neighbor_ = true;
    //! number of neighbor keyframes
    const unsigned int top_n_covisibilities_to_search_ = 10;

    const float num_common_words_thr_ratio_ = 0.8f;

    const unsigned int max_num_ransac_iter_ = 30;

    const unsigned int max_num_local_keyfrms_ = 60;
};

} // namespace module
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MODULE_RELOCALIZER_HPP_
