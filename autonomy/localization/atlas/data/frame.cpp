#include "autonomy/localization/atlas/data/common.hpp"
#include "autonomy/localization/atlas/data/frame.hpp"
#include "autonomy/localization/atlas/data/line_triangulation.hpp"
#include "autonomy/localization/atlas/data/keyframe.hpp"
#include "autonomy/localization/atlas/data/landmark.hpp"
#include "autonomy/localization/atlas/data/landmark_line.hpp"
#include "autonomy/localization/atlas/feature/orb_extractor.hpp"
#include "autonomy/localization/atlas/match/stereo.hpp"

#include <thread>
#include "autolink/common/log.hpp"

namespace autonomy::localization::atlas {
namespace data {

frame::frame(unsigned int frame_id, const double timestamp, camera::base* camera, feature::orb_params* orb_params,
             const frame_observation frm_obs, const std::unordered_map<unsigned int, marker2d>& markers_2d)
    : id_(frame_id), timestamp_(timestamp), camera_(camera), orb_params_(orb_params), frm_obs_(frm_obs),
      markers_2d_(markers_2d),
      // Initialize association with 3D points
      landmarks_(std::vector<std::shared_ptr<landmark>>(frm_obs_.undist_keypts_.size(), nullptr)) {}

void frame::set_pose_cw(const Mat44_t& pose_cw) {
    pose_is_valid_ = true;
    pose_cw_ = pose_cw;

    rot_cw_ = pose_cw_.block<3, 3>(0, 0);
    rot_wc_ = rot_cw_.transpose();
    trans_cw_ = pose_cw_.block<3, 1>(0, 3);
    trans_wc_ = -rot_cw_.transpose() * trans_cw_;
}

Mat44_t frame::get_pose_cw() const {
    return pose_cw_;
}

Mat44_t frame::get_pose_wc() const {
    Mat44_t pose_wc = Mat44_t::Identity();
    pose_wc.block<3, 3>(0, 0) = rot_wc_;
    pose_wc.block<3, 1>(0, 3) = trans_wc_;
    return pose_wc;
}

Vec3_t frame::get_trans_wc() const {
    return trans_wc_;
}

Mat33_t frame::get_rot_wc() const {
    return rot_wc_;
}

bool frame::bow_is_available() const {
    return !bow_vec_.empty() && !bow_feat_vec_.empty();
}

void frame::compute_bow(bow_vocabulary* bow_vocab) {
    bow_vocabulary_util::compute_bow(bow_vocab, frm_obs_.descriptors_, bow_vec_, bow_feat_vec_);
}

bool frame::can_observe(const std::shared_ptr<landmark>& lm, const float ray_cos_thr,
                        Vec2_t& reproj, float& x_right, unsigned int& pred_scale_level) const {
    const Vec3_t pos_w = lm->get_pos_in_world();

    const bool in_image = camera_->reproject_to_image(rot_cw_, trans_cw_, pos_w, reproj, x_right);
    if (!in_image) {
        return false;
    }

    const Vec3_t cam_to_lm_vec = pos_w - trans_wc_;
    const auto cam_to_lm_dist = cam_to_lm_vec.norm();
    const auto margin_far = 1.3;
    const auto margin_near = 1.0 / margin_far;
    if (!lm->is_inside_in_orb_scale(cam_to_lm_dist, margin_far, margin_near)) {
        return false;
    }

    const Vec3_t obs_mean_normal = lm->get_obs_mean_normal();
    const auto ray_cos = cam_to_lm_vec.dot(obs_mean_normal) / cam_to_lm_dist;
    if (ray_cos < ray_cos_thr) {
        return false;
    }

    pred_scale_level = lm->predict_scale_level(cam_to_lm_dist, this->orb_params_->num_levels_, this->orb_params_->log_scale_factor_);
    return true;
}

bool frame::has_landmark(const std::shared_ptr<landmark>& lm) const {
    return static_cast<bool>(landmarks_idx_map_.count(lm));
}

void frame::add_landmark(const std::shared_ptr<landmark>& lm, const unsigned int idx) {
    ADEBUG << "frame::add_landmark " << id_ << " " << lm->id_ << " " << idx;
    assert(!has_landmark(lm));
    landmarks_.at(idx) = lm;
    landmarks_idx_map_[lm] = idx;
}

std::shared_ptr<landmark> frame::get_landmark(const unsigned int idx) const {
    return landmarks_.at(idx);
}

void frame::erase_landmark_with_index(const unsigned int idx) {
    assert(landmarks_.at(idx));
    landmarks_idx_map_.erase(landmarks_.at(idx));
    landmarks_.at(idx) = nullptr;
}

void frame::erase_landmark(const std::shared_ptr<landmark>& lm) {
    assert(has_landmark(lm));
    auto idx = landmarks_idx_map_[lm];
    landmarks_idx_map_.erase(lm);
    landmarks_.at(idx) = nullptr;
}

std::vector<std::shared_ptr<landmark>> frame::get_landmarks() const {
    return landmarks_;
}

void frame::erase_landmarks() {
    std::fill(landmarks_.begin(), landmarks_.end(), nullptr);
    landmarks_idx_map_.clear();
}

void frame::set_landmarks(const std::vector<std::shared_ptr<landmark>>& landmarks) {
    erase_landmarks();
    for (unsigned int idx = 0; idx < landmarks.size(); ++idx) {
        const auto& lm = landmarks.at(idx);
        if (lm) {
            add_landmark(lm, idx);
        }
    }
}

void frame::init_line_tracking(unsigned int num_scale_levels, float scale_factor) {
    const auto n = line_obs_.size();
    landmarks_line_.assign(n, nullptr);
    outlier_flags_line_.assign(n, false);
    num_scale_levels_lsd_ = std::max(1u, num_scale_levels);
    log_scale_factor_lsd_ = std::log(scale_factor);
    scale_factors_lsd_.resize(num_scale_levels_lsd_);
    inv_level_sigma_sq_lsd_.resize(num_scale_levels_lsd_);
    float level_sum = 1.f;
    for (unsigned int level = 0; level < num_scale_levels_lsd_; ++level) {
        scale_factors_lsd_[level] = level_sum;
        inv_level_sigma_sq_lsd_[level] = 1.f / (level_sum * level_sum);
        level_sum *= scale_factor;
    }
}

bool frame::can_observe_line(const std::shared_ptr<landmark_line>& lm_line,
                             Vec2_t& reproj_sp, float& x_right_sp,
                             Vec2_t& reproj_ep, float& x_right_ep,
                             unsigned int& pred_scale_level) const {
    const Vec6_t pos_w = lm_line->get_pos_in_world();
    const Vec3_t pos_w_sp = pos_w.head<3>();
    const Vec3_t pos_w_ep = pos_w.tail<3>();

    const bool in_image_sp = camera_->reproject_to_image(rot_cw_, trans_cw_, pos_w_sp, reproj_sp, x_right_sp);
    const bool in_image_ep = camera_->reproject_to_image(rot_cw_, trans_cw_, pos_w_ep, reproj_ep, x_right_ep);
    if (!in_image_sp && !in_image_ep) {
        return false;
    }
    if (!in_image_sp || !in_image_ep) {
        const Vec3_t pos_w_mp = 0.5 * (pos_w_sp + pos_w_ep);
        Vec2_t reproj_mp;
        float x_right_mp;
        if (!camera_->reproject_to_image(rot_cw_, trans_cw_, pos_w_mp, reproj_mp, x_right_mp)) {
            return false;
        }
    }

    const Vec3_t cam_to_lm_vec = 0.5 * (pos_w_sp + pos_w_ep) - trans_wc_;
    const float cam_to_lm_dist = static_cast<float>(cam_to_lm_vec.norm());
    if (!lm_line->is_inside_in_feature_scale(cam_to_lm_dist)) {
        return false;
    }

    pred_scale_level = lm_line->predict_scale_level(cam_to_lm_dist, log_scale_factor_lsd_, num_scale_levels_lsd_);
    return true;
}

void frame::add_landmark_line(const std::shared_ptr<landmark_line>& lm_line, unsigned int idx) {
    landmarks_line_.at(idx) = lm_line;
}

std::shared_ptr<landmark_line> frame::get_landmark_line(unsigned int idx) const {
    return landmarks_line_.at(idx);
}

void frame::erase_landmark_line_with_index(unsigned int idx) {
    landmarks_line_.at(idx) = nullptr;
}

std::vector<std::shared_ptr<landmark_line>> frame::get_landmarks_line() const {
    return landmarks_line_;
}

void frame::erase_landmarks_line() {
    std::fill(landmarks_line_.begin(), landmarks_line_.end(), nullptr);
}

std::vector<unsigned int> frame::get_keylines_in_cell(const float ref_x1, const float ref_y1,
                                                      const float ref_x2, const float ref_y2, const float margin,
                                                      const int min_level, const int max_level) const {
    return data::get_keylines_in_cell(line_obs_.keylines, ref_x1, ref_y1, ref_x2, ref_y2, margin, min_level, max_level);
}

std::vector<unsigned int> frame::get_keypoints_in_cell(const float ref_x, const float ref_y, const float margin, const int min_level, const int max_level) const {
    return data::get_keypoints_in_cell(camera_, frm_obs_, ref_x, ref_y, margin, min_level, max_level);
}

Vec3_t frame::triangulate_stereo(const unsigned int idx) const {
    return data::triangulate_stereo(camera_, rot_wc_, trans_wc_, frm_obs_, idx);
}

Vec6_t frame::triangulate_stereo_for_line(const unsigned int idx) const {
    return triangulate_stereo_for_line_impl(camera_, rot_wc_, trans_wc_, line_obs_, idx);
}

} // namespace data
}  // namespace autonomy::localization::atlas
