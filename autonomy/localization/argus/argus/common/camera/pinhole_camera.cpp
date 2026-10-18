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

#include "argus/common/camera/pinhole_camera.hpp"

#include <cmath>
#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include "Eigen/LU"

namespace argus {
namespace camera {

PinholeCamera::PinholeCamera(double width, double height, double scale,
                             double fx, double fy, double cx, double cy,
                             double d0, double d1, double d2, double d3,
                             double d4)
    : AbstractCamera(static_cast<int>(width * scale),
                     static_cast<int>(height * scale), scale),
      fx_(fx * scale),
      fy_(fy * scale),
      cx_(cx * scale),
      cy_(cy * scale),
      distortion_(std::fabs(d0) > 1e-7),
      undist_map1_(height_, width_, CV_16SC2),
      undist_map2_(height_, width_, CV_16SC2) {
    d_[0] = d0;
    d_[1] = d1;
    d_[2] = d2;
    d_[3] = d3;
    d_[4] = d4;
    cvK_ = (cv::Mat_<float>(3, 3) << static_cast<float>(fx_), 0.0f,
            static_cast<float>(cx_), 0.0f, static_cast<float>(fy_),
            static_cast<float>(cy_), 0.0f, 0.0f, 1.0f);
    cvD_ = (cv::Mat_<float>(1, 5) << static_cast<float>(d_[0]),
            static_cast<float>(d_[1]), static_cast<float>(d_[2]),
            static_cast<float>(d_[3]), static_cast<float>(d_[4]));
    cv::initUndistortRectifyMap(cvK_, cvD_, cv::Mat_<double>::eye(3, 3), cvK_,
                                cv::Size(width_, height_), CV_16SC2,
                                undist_map1_, undist_map2_);
    K_ << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0;
    K_inv_ = K_.inverse();
}

Eigen::Vector3d PinholeCamera::cam2world(const double& u,
                                         const double& v) const {
    Eigen::Vector3d xyz;
    if (!distortion_) {
        xyz[0] = (u - cx_) / fx_;
        xyz[1] = (v - cy_) / fy_;
        xyz[2] = 1.0;
    } else {
        cv::Point2f uv(static_cast<float>(u), static_cast<float>(v));
        cv::Point2f px;
        const cv::Mat src_pt(1, 1, CV_32FC2, &uv.x);
        cv::Mat dst_pt(1, 1, CV_32FC2, &px.x);
        cv::undistortPoints(src_pt, dst_pt, cvK_, cvD_);
        xyz[0] = px.x;
        xyz[1] = px.y;
        xyz[2] = 1.0;
    }
    return xyz.normalized();
}

Eigen::Vector3d PinholeCamera::cam2world(const Eigen::Vector2d& uv) const {
    return cam2world(uv[0], uv[1]);
}

Eigen::Vector2d PinholeCamera::world2cam(const Eigen::Vector3d& xyz) const {
    return world2cam(Project2d(xyz));
}

Eigen::Vector2d PinholeCamera::world2cam(const Eigen::Vector2d& uv) const {
    Eigen::Vector2d px;
    if (!distortion_) {
        px[0] = fx_ * uv[0] + cx_;
        px[1] = fy_ * uv[1] + cy_;
    } else {
        const double x = uv[0];
        const double y = uv[1];
        const double r2 = x * x + y * y;
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;
        const double a1 = 2 * x * y;
        const double a2 = r2 + 2 * x * x;
        const double a3 = r2 + 2 * y * y;
        const double cdist = 1 + d_[0] * r2 + d_[1] * r4 + d_[4] * r6;
        const double xd = x * cdist + d_[2] * a1 + d_[3] * a2;
        const double yd = y * cdist + d_[2] * a3 + d_[3] * a1;
        px[0] = xd * fx_ + cx_;
        px[1] = yd * fy_ + cy_;
    }
    return px;
}

void PinholeCamera::undistortImage(const cv::Mat& raw,
                                   cv::Mat& rectified) const {
    if (distortion_) {
        cv::remap(raw, rectified, undist_map1_, undist_map2_, cv::INTER_LINEAR);
    } else {
        rectified = raw.clone();
    }
}

}  // namespace camera
}  // namespace argus
