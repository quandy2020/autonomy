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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_DATA_FRAME_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_DATA_FRAME_HPP_

#include "autonomy/localization/atlas/type.hpp"
#include "autonomy/localization/atlas/camera/base.hpp"
#include "autonomy/localization/atlas/feature/orb_params.hpp"
#include "autonomy/localization/atlas/util/converter.hpp"
#include "autonomy/localization/atlas/data/frame_observation.hpp"
#include "autonomy/localization/atlas/data/line_frame_observation.hpp"
#include "autonomy/localization/atlas/data/landmark_line.hpp"
#include "autonomy/localization/atlas/data/bow_vocabulary.hpp"
#include "autonomy/localization/atlas/data/marker2d.hpp"
#include "autonomy/localization/atlas/data/bow_vocabulary_fwd.hpp"

#include <vector>
#include <atomic>
#include <memory>
#include <unordered_set>

#include <Eigen/Core>

namespace autonomy::localization::atlas {

namespace camera {
class base;
} // namespace camera

namespace feature {
class orb_extractor;
struct orb_params;
} // namespace feature

namespace data {

class keyframe;
class landmark;

class frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    frame() = default;

    bool operator==(const frame& frm) { return this->id_ == frm.id_; }
    bool operator!=(const frame& frm) { return !(*this == frm); }

    /**
     * Constructor for monocular frame
     * @param frame_id
     * @param timestamp
     * @param camera
     * @param orb_params
     * @param frm_obs
     * @param markers_2d
     */
    frame(const unsigned int frame_id, const double timestamp, camera::base* camera, feature::orb_params* orb_params,
          const frame_observation frm_obs, const std::unordered_map<unsigned int, marker2d>& markers_2d);

    /**
     * Set camera pose and refresh rotation and translation
     * @param pose_cw
     */
    void set_pose_cw(const Mat44_t& pose_cw);

    /**
     * Get camera pose
     */
    Mat44_t get_pose_cw() const;

    /**
     * Get the inverse of the camera pose
     */
    Mat44_t get_pose_wc() const;

    /**
     * Get camera center
     * @return
     */
    Vec3_t get_trans_wc() const;

    /**
     * Get inverse of rotation
     * @return
     */
    Mat33_t get_rot_wc() const;

    /**
     * Get the translation of the camera pose
     */
    Vec3_t get_trans_cw() const {
        return trans_cw_;
    }

    /**
     * Get the rotation of the camera pose
     */
    Mat33_t get_rot_cw() const {
        return rot_cw_;
    }

    /**
     * Invalidate pose
     */
    void invalidate_pose() {
        pose_is_valid_ = false;
    }

    /**
     * Return true if pose is valid
     */
    bool pose_is_valid() const {
        return pose_is_valid_;
    }

    /**
     * Returns true if BoW has been computed.
     */
    bool bow_is_available() const;

    /**
     * Compute BoW representation
     */
    void compute_bow(bow_vocabulary* bow_vocab);

    /**
     * Check observability of the landmark
     */
    bool can_observe(const std::shared_ptr<landmark>& lm, const float ray_cos_thr,
                     Vec2_t& reproj, float& x_right, unsigned int& pred_scale_level) const;

    bool has_landmark(const std::shared_ptr<landmark>& lm) const;

    void add_landmark(const std::shared_ptr<landmark>&, const unsigned int idx);

    std::shared_ptr<landmark> get_landmark(const unsigned int idx) const;

    void erase_landmark_with_index(const unsigned int idx);

    void erase_landmark(const std::shared_ptr<landmark>& lm);

    std::vector<std::shared_ptr<landmark>> get_landmarks() const;

    void erase_landmarks();

    void set_landmarks(const std::vector<std::shared_ptr<landmark>>& landmarks);

    /** Initialize line landmark associations after line_obs_ is filled. */
    void init_line_tracking(unsigned int num_scale_levels, float scale_factor);

    bool can_observe_line(const std::shared_ptr<landmark_line>& lm_line,
                          Vec2_t& reproj_sp, float& x_right_sp,
                          Vec2_t& reproj_ep, float& x_right_ep,
                          unsigned int& pred_scale_level) const;

    void add_landmark_line(const std::shared_ptr<landmark_line>& lm_line, unsigned int idx);
    std::shared_ptr<landmark_line> get_landmark_line(unsigned int idx) const;
    void erase_landmark_line_with_index(unsigned int idx);
    std::vector<std::shared_ptr<landmark_line>> get_landmarks_line() const;
    void erase_landmarks_line();

    std::vector<unsigned int> get_keylines_in_cell(float ref_x1, float ref_y1,
                                                   float ref_x2, float ref_y2, float margin,
                                                   int min_level = -1, int max_level = -1) const;

    /**
     * Get keypoint indices in the cell which reference point is located
     * @param ref_x
     * @param ref_y
     * @param margin
     * @param min_level
     * @param max_level
     * @return
     */
    std::vector<unsigned int> get_keypoints_in_cell(const float ref_x, const float ref_y, const float margin, const int min_level = -1, const int max_level = -1) const;

    /**
     * Perform stereo triangulation of the keypoint
     * @param idx
     * @return
     */
    Vec3_t triangulate_stereo(const unsigned int idx) const;
    Vec6_t triangulate_stereo_for_line(unsigned int idx) const;

    //! current frame ID
    unsigned int id_;

    //! timestamp
    double timestamp_;

    //! camera model
    camera::base* camera_ = nullptr;

    //! ORB scale pyramid information
    const feature::orb_params* orb_params_ = nullptr;

    //! constant observations
    frame_observation frm_obs_;

    //! optional LSD/LBD line observations (Structure-PLP-SLAM)
    line_frame_observation line_obs_;

    /** Instance segmentation mask (CV_8UC3, RGB label per pixel). */
    cv::Mat img_seg_mask_;

    /** RGB-D depth map in meters (CV_32FC1), used for dense plane sampling. */
    cv::Mat depth_map_;

    //! line landmark associations (parallel to line_obs_.keylines)
    std::vector<std::shared_ptr<landmark_line>> landmarks_line_;
    std::vector<bool> outlier_flags_line_;
    std::vector<float> scale_factors_lsd_;
    std::vector<float> inv_level_sigma_sq_lsd_;
    float log_scale_factor_lsd_ = 0.f;
    unsigned int num_scale_levels_lsd_ = 1;

    //! markers 2D (ID to marker2d map)
    std::unordered_map<unsigned int, marker2d> markers_2d_;

    //! BoW features (DBoW2 or FBoW)
    bow_vector bow_vec_;
    bow_feature_vector bow_feat_vec_;

    //! reference keyframe for tracking
    std::shared_ptr<keyframe> ref_keyfrm_ = nullptr;

private:
    //! landmarks, whose nullptr indicates no-association
    std::vector<std::shared_ptr<landmark>> landmarks_;
    std::unordered_map<std::shared_ptr<landmark>, unsigned int> landmarks_idx_map_;

    //! camera pose: world -> camera
    bool pose_is_valid_ = false;
    Mat44_t pose_cw_;

    //! Camera pose
    //! rotation: world -> camera
    Mat33_t rot_cw_;
    //! translation: world -> camera
    Vec3_t trans_cw_;
    //! rotation: camera -> world
    Mat33_t rot_wc_;
    //! translation: camera -> world
    Vec3_t trans_wc_;
};

} // namespace data
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_DATA_FRAME_HPP_
