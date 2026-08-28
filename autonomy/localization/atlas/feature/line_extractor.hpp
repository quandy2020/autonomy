/*
 * Copyright 2026 The Openbot Authors
 *
 * LSD + LBD line feature extraction (Structure-PLP-SLAM LineFeatureTracker).
 */

#pragma once

#include <vector>

#include <opencv2/core/mat.hpp>

#include "autonomy/localization/atlas/camera/base.hpp"
#include "autonomy/localization/atlas/data/line_frame_observation.hpp"
#include "autonomy/localization/atlas/plp/plp_options.hpp"

namespace autonomy::localization::atlas::feature {

class line_extractor {
public:
    explicit line_extractor(camera::base* camera, plp::Options options);

    void extract(const cv::Mat& gray, data::line_frame_observation* out) const;

    unsigned int num_scale_levels() const { return num_levels_; }
    float scale_factor() const { return scale_factor_; }

private:
    void ensure_undistort_maps(const cv::Mat& img) const;

    camera::base* camera_ = nullptr;
    plp::Options options_;
    unsigned int num_levels_ = 1;
    float scale_factor_ = 2.f;
    mutable cv::Mat undist_map1_;
    mutable cv::Mat undist_map2_;
    mutable cv::Mat k_rect_;
    mutable bool maps_ready_ = false;
};

}  // namespace autonomy::localization::atlas::feature
