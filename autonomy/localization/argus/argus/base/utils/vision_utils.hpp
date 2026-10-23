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

/**
 * @file vision_utils.hpp
 * @brief Minimal vision helpers (vikit-compatible, used by FAST-LIVO2 VIO).
 */

#ifndef ARGUS_COMMON_VISION_VISION_UTILS_HPP_
#define ARGUS_COMMON_VISION_VISION_UTILS_HPP_

#include <cmath>

#include <opencv2/core.hpp>

namespace argus {
namespace vision {

/**
 * @brief Bilinear sample of CV_8U image; caller ensures in-bounds.
 * @param mat Single-channel 8-bit image.
 * @param u Subpixel column coordinate.
 * @param v Subpixel row coordinate.
 * @return Interpolated intensity.
 */
inline float InterpolateMat8u(const cv::Mat& mat, float u, float v) {
    const int x = static_cast<int>(std::floor(u));
    const int y = static_cast<int>(std::floor(v));
    const float subpix_x = u - static_cast<float>(x);
    const float subpix_y = v - static_cast<float>(y);
    const float w00 = (1.0f - subpix_x) * (1.0f - subpix_y);
    const float w01 = (1.0f - subpix_x) * subpix_y;
    const float w10 = subpix_x * (1.0f - subpix_y);
    const float w11 = 1.0f - w00 - w01 - w10;
    const int stride = static_cast<int>(mat.step.p[0]);
    const unsigned char* ptr = mat.data + y * stride + x;
    return w00 * ptr[0] + w01 * ptr[stride] + w10 * ptr[1] +
           w11 * ptr[stride + 1];
}

/** @brief Legacy vikit name (same as InterpolateMat8u). */
inline float interpolateMat_8u(const cv::Mat& mat, float u, float v) {
    return InterpolateMat8u(mat, u, v);
}

/**
 * @brief Shi-Tomasi corner score (vikit formula, 8×8 box).
 * @param img Grayscale image (CV_8U).
 * @param u Integer column of the candidate corner.
 * @param v Integer row of the candidate corner.
 * @return Corner quality score; 0 when too close to the border.
 */
inline float ShiTomasiScore(const cv::Mat& img, int u, int v) {
    float dXX = 0.0f;
    float dYY = 0.0f;
    float dXY = 0.0f;
    constexpr int kHalfBox = 4;
    constexpr int kBoxSize = 2 * kHalfBox;
    constexpr int kBoxArea = kBoxSize * kBoxSize;
    const int x_min = u - kHalfBox;
    const int x_max = u + kHalfBox;
    const int y_min = v - kHalfBox;
    const int y_max = v + kHalfBox;
    if (x_min < 1 || x_max >= img.cols - 1 || y_min < 1 ||
        y_max >= img.rows - 1) {
        return 0.0f;
    }
    const int stride = static_cast<int>(img.step.p[0]);
    for (int y = y_min; y < y_max; ++y) {
        const uint8_t* ptr_left = img.data + stride * y + x_min - 1;
        const uint8_t* ptr_right = img.data + stride * y + x_min + 1;
        const uint8_t* ptr_top = img.data + stride * (y - 1) + x_min;
        const uint8_t* ptr_bottom = img.data + stride * (y + 1) + x_min;
        for (int x = 0; x < kBoxSize;
             ++x, ++ptr_left, ++ptr_right, ++ptr_top, ++ptr_bottom) {
            const float dx = static_cast<float>(*ptr_right - *ptr_left);
            const float dy = static_cast<float>(*ptr_bottom - *ptr_top);
            dXX += dx * dx;
            dYY += dy * dy;
            dXY += dx * dy;
        }
    }
    dXX = dXX / (2.0f * kBoxArea);
    dYY = dYY / (2.0f * kBoxArea);
    dXY = dXY / (2.0f * kBoxArea);
    return 0.5f * (dXX + dYY -
                   std::sqrt((dXX + dYY) * (dXX + dYY) -
                             4.0f * (dXX * dYY - dXY * dXY)));
}

/** @brief Legacy vikit name (same as ShiTomasiScore). */
inline float shiTomasiScore(const cv::Mat& img, int u, int v) {
    return ShiTomasiScore(img, u, v);
}

}  // namespace vision
}  // namespace argus

#endif  // ARGUS_COMMON_VISION_VISION_UTILS_HPP_
