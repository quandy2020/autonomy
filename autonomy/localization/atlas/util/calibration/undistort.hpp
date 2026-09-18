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

#pragma once

//! Lightweight camera undistort helpers (OpenCV), driven by CameraIntrinsics.
//! Full Atlas tracking still uses sensor/camera::*; use this for LIVO/tooling
//! when only the calib bundle is available.

#include "autonomy/localization/atlas/util/calibration/types.hpp"

#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace autonomy::localization::atlas {
namespace calibration {

inline cv::Mat CameraMatrixCv(const CameraIntrinsics& c) {
    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = c.fx;
    K.at<double>(1, 1) = c.fy;
    K.at<double>(0, 2) = c.cx;
    K.at<double>(1, 2) = c.cy;
    return K;
}

inline cv::Mat DistCoeffsCv(const CameraIntrinsics& c) {
    if (c.dist_coeffs.empty()) {
        return cv::Mat::zeros(5, 1, CV_64F);
    }
    cv::Mat D(static_cast<int>(c.dist_coeffs.size()), 1, CV_64F);
    for (int i = 0; i < D.rows; ++i) {
        D.at<double>(i, 0) = c.dist_coeffs[static_cast<std::size_t>(i)];
    }
    return D;
}

//! Undistort pixel keypoints → normalized or pixel (keep_pixel=true uses same K).
inline void UndistortPoints(const CameraIntrinsics& cam,
                            const std::vector<cv::Point2f>& distorted,
                            std::vector<cv::Point2f>* undistorted,
                            bool keep_pixel = true) {
    if (!undistorted) {
        return;
    }
    undistorted->clear();
    if (distorted.empty()) {
        return;
    }
    if (!cam.has_distortion()) {
        *undistorted = distorted;
        return;
    }
    const cv::Mat K = CameraMatrixCv(cam);
    const cv::Mat D = DistCoeffsCv(cam);
    if (cam.model == "fisheye") {
        if (keep_pixel) {
            cv::fisheye::undistortPoints(distorted, *undistorted, K, D, cv::Mat(),
                                         K);
        } else {
            cv::fisheye::undistortPoints(distorted, *undistorted, K, D);
        }
    } else {
        if (keep_pixel) {
            cv::undistortPoints(distorted, *undistorted, K, D, cv::Mat(), K);
        } else {
            cv::undistortPoints(distorted, *undistorted, K, D);
        }
    }
}

//! Optional lidar range scale/bias along ray (identity if scale=1, bias=0).
inline Vec3_t CorrectLidarRange(const LidarIntrinsics& lid, const Vec3_t& p) {
    const double r = p.norm();
    if (r < 1e-9) {
        return p;
    }
    const double r2 = lid.range_scale * r + lid.range_bias;
    return p * (r2 / r);
}

}  // namespace calibration
}  // namespace autonomy::localization::atlas
