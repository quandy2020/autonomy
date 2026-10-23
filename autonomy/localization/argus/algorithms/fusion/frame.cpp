/*
 * Copyright 2026 The Openbot Authors
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

#include "argus/odometry/frame.hpp"

#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace argus {
namespace odometry {

int Frame::frame_counter_ = 0;

Frame::Frame(camera::AbstractCamera* cam, const cv::Mat& img)
    : id_(frame_counter_++), cam_(cam) {
    InitFrame(img);
}

Frame::~Frame() {
    for (map::Feature* f : features_) {
        delete f;
    }
    features_.clear();
}

void Frame::InitFrame(const cv::Mat& img) {
    if (img.empty()) {
        throw std::runtime_error("Frame: provided image is empty");
    }
    if (img.cols != cam_->width() || img.rows != cam_->height()) {
        throw std::runtime_error(
            "Frame: provided image has not the same size as the camera model");
    }
    if (img.type() != CV_8UC1) {
        throw std::runtime_error("Frame: provided image is not grayscale");
    }
    image_ = img;
}

void Frame::Reset(camera::AbstractCamera* cam, const cv::Mat& img) {
    for (map::Feature* f : features_) {
        delete f;
    }
    features_.clear();
    cam_ = cam;
    id_ = frame_counter_++;
    T_f_w_ = math::SE3();
    T_f_w_prior_ = math::SE3();
    InitFrame(img);
}

namespace frame_utils {

void CreateImgPyramid(const cv::Mat& img_level_0, int n_levels, ImagePyramid& pyr) {
    pyr.resize(static_cast<std::size_t>(n_levels));
    pyr[0] = img_level_0;
    for (int i = 1; i < n_levels; ++i) {
        cv::pyrDown(pyr[static_cast<std::size_t>(i - 1)],
                    pyr[static_cast<std::size_t>(i)]);
    }
}

}  // namespace frame_utils
}  // namespace odometry
}  // namespace argus
