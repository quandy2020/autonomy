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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MATCH_PROJECTION_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MATCH_PROJECTION_HPP_

#include "autonomy/localization/atlas/type.hpp"
#include "autonomy/localization/atlas/match/base.hpp"

#include <set>
#include <memory>

#include "autonomy/localization/atlas/data/landmark_line.hpp"

namespace autonomy::localization::atlas {

namespace data {
class frame;
struct frame_observation;
class keyframe;
class landmark;
} // namespace data

namespace match {

class projection final : public base {
public:
    explicit projection(const float lowe_ratio = 0.6, const bool check_orientation = true)
        : base(lowe_ratio, check_orientation) {}

    ~projection() final = default;

    //! frameの2次元点と3次元の対応を求め，frame.landmarks_に対応情報を記録する
    unsigned int match_frame_and_landmarks(data::frame& frm,
                                           const std::vector<std::shared_ptr<data::landmark>>& local_landmarks,
                                           eigen_alloc_unord_map<unsigned int, Vec2_t>& lm_to_reproj,
                                           std::unordered_map<unsigned int, float>& lm_to_x_right,
                                           std::unordered_map<unsigned int, unsigned int>& lm_to_scale,
                                           const float margin = 5.0) const;

    //! last frameで観測している3次元点をcurrent frameに再投影し，frame.landmarks_に対応情報を記録する
    unsigned int match_current_and_last_frames(data::frame& curr_frm, const data::frame& last_frm, const float margin) const;

    //! keyfarmeで観測している3次元点をcurrent frameに再投影し，frame.landmarks_に対応情報を記録する
    //! current frameとすでに対応が取れているものは，already_matched_lmsに指定して再投影しないようにする
    unsigned int match_frame_and_keyframe(data::frame& curr_frm, const std::shared_ptr<data::keyframe>& keyfrm, const std::set<std::shared_ptr<data::landmark>>& already_matched_lms,
                                          const float margin, const unsigned int hamm_dist_thr) const;
    unsigned int match_frame_and_keyframe(const Mat44_t& cam_pose_cw,
                                          const camera::base* camera,
                                          const data::frame_observation& frm_obs,
                                          const feature::orb_params* orb_params,
                                          std::vector<std::shared_ptr<data::landmark>>& frm_landmarks,
                                          const std::shared_ptr<data::keyframe>& keyfrm,
                                          const std::set<std::shared_ptr<data::landmark>>& already_matched_lms,
                                          const float margin, const unsigned int hamm_dist_thr) const;

    //! 3次元点をSim3で座標変換したのちkeyframeに再投影し，matched_lms_in_keyfrmに対応情報を記録する
    //! matched_lms_in_keyfrmにすでに対応情報が記録されている場合は，探索の対象外とする
    //! (NOTE: keyframeの特徴点数とmatched_lms_in_keyfrm.size()は一致している)
    unsigned int match_by_Sim3_transform(const std::shared_ptr<data::keyframe>& keyfrm, const Mat44_t& Sim3_cw, const std::vector<std::shared_ptr<data::landmark>>& landmarks,
                                         std::vector<std::shared_ptr<data::landmark>>& matched_lms_in_keyfrm, const float margin) const;

    //! 指定されたSim3を用いて，各々のkeyframeで観測されている3次元点をもう片方のkeyframeへ変換・再投影し，対応点を求める
    //! matched_lms_in_keyfrm_1には，keyframe1の特徴点(index)と対応する，keyframe2で観測されている3次元点が記録される
    unsigned int match_keyframes_mutually(const std::shared_ptr<data::keyframe>& keyfrm_1, const std::shared_ptr<data::keyframe>& keyfrm_2, std::vector<std::shared_ptr<data::landmark>>& matched_lms_in_keyfrm_1,
                                          const float& s_12, const Mat33_t& rot_12, const Vec3_t& trans_12, const float margin) const;

    unsigned int match_frame_and_landmarks_line(data::frame& frm,
                                                const std::vector<std::shared_ptr<data::landmark_line>>& local_landmarks_line,
                                                float margin = 5.0f) const;

    unsigned int match_frame_and_keyframe_line(data::frame& curr_frm,
                                               const std::shared_ptr<data::keyframe>& keyfrm,
                                               const std::set<std::shared_ptr<data::landmark_line>>& already_matched_lms,
                                               float margin = 10.0f,
                                               unsigned int hamm_dist_thr = HAMMING_DIST_THR_HIGH) const;

    unsigned int match_current_and_last_frames_line(data::frame& curr_frm, const data::frame& last_frm, float margin) const;
};

} // namespace match
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MATCH_PROJECTION_HPP_
