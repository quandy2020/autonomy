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

//! util/calibration — multimodal intrinsics / distortion / SE3 extrinsics.
//! Camera optical undistort still runs via sensor/camera; lidar motion deskew
//! via frontend/lio (IMU traj). This package owns the YAML calib bundle.

#include "autonomy/localization/atlas/type.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace autonomy::localization::atlas {
namespace calibration {

//! p_dst = R * p_src + t  (stored as Mat44).
inline Mat44_t MakeSE3(const Mat33_t& R, const Vec3_t& t) {
    Mat44_t T = Mat44_t::Identity();
    T.block<3, 3>(0, 0) = R;
    T.block<3, 1>(0, 3) = t;
    return T;
}

inline Mat44_t InverseSE3(const Mat44_t& T) {
    Mat44_t inv = Mat44_t::Identity();
    const Mat33_t R = T.block<3, 3>(0, 0);
    const Vec3_t t = T.block<3, 1>(0, 3);
    inv.block<3, 3>(0, 0) = R.transpose();
    inv.block<3, 1>(0, 3) = -R.transpose() * t;
    return inv;
}

//! Row-major 9 or flat list → Mat33.
inline bool ParseMat33(const std::vector<double>& v, Mat33_t* out) {
    if (!out || v.size() != 9) {
        return false;
    }
    *out << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
    return true;
}

inline bool ParseVec3(const std::vector<double>& v, Vec3_t* out) {
    if (!out || v.size() != 3) {
        return false;
    }
    *out << v[0], v[1], v[2];
    return true;
}

//! Camera pinhole / fisheye intrinsics + distortion (OpenCV-compatible order).
struct CameraIntrinsics {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string model = "perspective";  // perspective | fisheye | equirectangular
    int width = 0;
    int height = 0;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    //! OpenCV: k1,k2,p1,p2[,k3] or fisheye k1..k4
    std::vector<double> dist_coeffs;
    double fps = 0.0;

    [[nodiscard]] Mat33_t K() const {
        Mat33_t m = Mat33_t::Identity();
        m(0, 0) = fx;
        m(1, 1) = fy;
        m(0, 2) = cx;
        m(1, 2) = cy;
        return m;
    }

    [[nodiscard]] bool has_distortion() const {
        for (double d : dist_coeffs) {
            if (std::abs(d) > 1e-12) {
                return true;
            }
        }
        return false;
    }
};

//! IMU noise / bias priors (for ESKF Q and init; not a second estimator).
struct ImuIntrinsics {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double gyro_noise = 1e-3;
    double acc_noise = 1e-2;
    double gyro_bias_noise = 1e-5;
    double acc_bias_noise = 1e-4;
    Vec3_t gravity = Vec3_t(0.0, 0.0, -9.81);
    //! Optional fixed bias priors (zeros = unknown / estimate online).
    Vec3_t ba0 = Vec3_t::Zero();
    Vec3_t bg0 = Vec3_t::Zero();
};

//! Lidar optical / model params (motion deskew is separate: lio::ImuProcess).
struct LidarIntrinsics {
    std::string model = "generic";  // generic | velodyne | ouster | livox
    double min_range = 0.5;
    double max_range = 80.0;
    double scan_period = 0.1;  // s; used when no per-point time
    //! Optional range scale / bias (p' = scale * p + bias * dir) — usually 1/0.
    double range_scale = 1.0;
    double range_bias = 0.0;
};

//! Named SE3 extrinsic: p_to = R * p_from + t.
struct ExtrinsicSE3 {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Mat33_t R = Mat33_t::Identity();
    Vec3_t t = Vec3_t::Zero();

    [[nodiscard]] Mat44_t T() const { return MakeSE3(R, t); }
    void SetT(const Mat44_t& T) {
        R = T.block<3, 3>(0, 0);
        t = T.block<3, 1>(0, 3);
    }
};

}  // namespace calibration
}  // namespace autonomy::localization::atlas
