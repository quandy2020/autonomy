#pragma once

#include "autonomy/localization/atlas/type.hpp"
#include "autonomy/localization/atlas/camera/base.hpp"
#include "autonomy/localization/atlas/feature/line_descriptor/line_descriptor_custom.hpp"

#include <memory>
#include <opencv2/core.hpp>

namespace autonomy::localization::atlas::data {
class keyframe;
}

#include "autonomy/localization/atlas/camera/perspective.hpp"

namespace autonomy::localization::atlas::module {

class two_view_triangulator_line {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    two_view_triangulator_line(const std::shared_ptr<data::keyframe>& keyfrm_1,
                               const std::shared_ptr<data::keyframe>& keyfrm_2,
                               float rays_parallax_deg_thr = 1.0f);

    bool triangulate(unsigned int idx_1, unsigned int idx_2, Vec6_t& pos_w) const;

private:
    bool check_depth_is_positive(const Vec3_t& pos_w, const Mat33_t& rot_cw, const Vec3_t& trans_cw) const;
    bool check_reprojection_error(const Vec3_t& pos_w, const Mat33_t& rot_cw, const Vec3_t& trans_cw,
                                  const Vec3_t& keyline_function, float x_right,
                                  float sigma_sq, bool is_stereo) const;
    bool check_reprojection_error(const Vec3_t& pos_w_middlepoint, const Mat33_t& rot_cw, const Vec3_t& trans_cw,
                                  const cv::line_descriptor::KeyLine& keyline, float sigma_sq) const;
    bool check_scale_factors(const Vec3_t& pos_w, float scale_factor_1, float scale_factor_2) const;
    Mat33_t skew(const Vec3_t& t) const;

    std::shared_ptr<data::keyframe> keyfrm_1_;
    std::shared_ptr<data::keyframe> keyfrm_2_;

    Mat33_t rot_1w_, rot_w1_, rot_2w_, rot_w2_;
    Vec3_t trans_1w_, trans_2w_, cam_center_1_, cam_center_2_;
    Mat44_t cam_pose_1w_, cam_pose_2w_;
    camera::base* camera_1_ = nullptr;
    camera::base* camera_2_ = nullptr;
    float ratio_factor_ = 2.f;
    float cos_rays_parallax_thr_ = 0.f;

    camera::perspective* camera_persp_ = nullptr;
    Mat33_t K_;
    camera::setup_type_t camera_type_;
};

inline bool two_view_triangulator_line::check_depth_is_positive(const Vec3_t& pos_w, const Mat33_t& rot_cw,
                                                                const Vec3_t& trans_cw) const {
    const auto pos_z = rot_cw.row(2).dot(pos_w) + trans_cw(2);
    return pos_z > 0;
}

inline bool two_view_triangulator_line::check_scale_factors(const Vec3_t& pos_w, float scale_factor_1,
                                                            float scale_factor_2) const {
    const auto cam_1_to_lm_dist = (pos_w - cam_center_1_).norm();
    const auto cam_2_to_lm_dist = (pos_w - cam_center_2_).norm();
    if (cam_1_to_lm_dist == 0 || cam_2_to_lm_dist == 0) {
        return false;
    }
    const auto ratio_dists = cam_2_to_lm_dist / cam_1_to_lm_dist;
    const auto ratio_octave = scale_factor_1 / scale_factor_2;
    return ratio_octave / ratio_dists < ratio_factor_ && ratio_dists / ratio_octave < ratio_factor_;
}

inline Mat33_t two_view_triangulator_line::skew(const Vec3_t& t) const {
    Mat33_t S;
    S << 0, -t.z(), t.y(), t.z(), 0, -t.x(), -t.y(), t.x(), 0;
    return S;
}

}  // namespace autonomy::localization::atlas::module
