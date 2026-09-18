/*
 * Copyright 2026 The Openbot Authors
 *
 * Per-frame line observations (Structure-PLP-SLAM LSD/LBD).
 */

#pragma once

#include "autonomy/localization/atlas/type.hpp"

#include <opencv2/core/mat.hpp>
#include <vector>

#include "autonomy/localization/atlas/frontend/feature/line_descriptor/line_descriptor_custom.hpp"

namespace autonomy::localization::atlas::data {

struct line_frame_observation {
    std::vector<cv::line_descriptor::KeyLine> keylines;
    cv::Mat lbd_descriptors;
    /** Homogeneous line coefficients ax+by+c=0 (normalized). */
    std::vector<Vec3_t> line_functions;
    /** Stereo/RGB-D: right image x or depth at endpoints (optional). */
    std::vector<float> stereo_x_right;
    std::vector<float> depths_start;
    std::vector<float> depths_end;

    bool empty() const { return keylines.empty(); }
    std::size_t size() const { return keylines.size(); }
};

}  // namespace autonomy::localization::atlas::data
