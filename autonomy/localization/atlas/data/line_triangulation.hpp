#pragma once

#include "autonomy/localization/atlas/camera/perspective.hpp"
#include "autonomy/localization/atlas/data/line_frame_observation.hpp"
#include "autonomy/localization/atlas/type.hpp"

namespace autonomy::localization::atlas::data {

inline Vec6_t triangulate_stereo_for_line_impl(camera::base* camera,
                                               const Mat33_t& rot_wc,
                                               const Vec3_t& trans_wc,
                                               const line_frame_observation& line_obs,
                                               const unsigned int idx) {
    if (camera == nullptr || camera->setup_type_ == camera::setup_type_t::Monocular) {
        return Vec6_t::Zero();
    }
    if (idx >= line_obs.keylines.size() || idx >= line_obs.depths_start.size() ||
        idx >= line_obs.depths_end.size()) {
        return Vec6_t::Zero();
    }

    const float depth_sp = line_obs.depths_start.at(idx);
    const float depth_ep = line_obs.depths_end.at(idx);
    if (depth_sp <= 0.0f || depth_ep <= 0.0f) {
        return Vec6_t::Zero();
    }

    auto* cam = static_cast<camera::perspective*>(camera);
    const auto& keyline = line_obs.keylines.at(idx);
    const auto& sp = keyline.getStartPoint();
    const auto& ep = keyline.getEndPoint();

    const float unproj_x_sp = (sp.x - cam->cx_) * depth_sp * cam->fx_inv_;
    const float unproj_y_sp = (sp.y - cam->cy_) * depth_sp * cam->fy_inv_;
    const float unproj_x_ep = (ep.x - cam->cx_) * depth_ep * cam->fx_inv_;
    const float unproj_y_ep = (ep.y - cam->cy_) * depth_ep * cam->fy_inv_;

    const Vec3_t pos_c_sp(unproj_x_sp, unproj_y_sp, depth_sp);
    const Vec3_t pos_c_ep(unproj_x_ep, unproj_y_ep, depth_ep);
    const Vec3_t pos_w_sp = rot_wc * pos_c_sp + trans_wc;
    const Vec3_t pos_w_ep = rot_wc * pos_c_ep + trans_wc;

    Vec6_t pos_w_line;
    pos_w_line << pos_w_sp(0), pos_w_sp(1), pos_w_sp(2), pos_w_ep(0), pos_w_ep(1), pos_w_ep(2);
    return pos_w_line;
}

}  // namespace autonomy::localization::atlas::data
