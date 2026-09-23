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
 * @file geometric_tools.cpp
 * @brief `GeometricTools` static method implementations (F matrix, DLT, epipolar).
 */

#include "autonomy/localization/atlas/common/geometric_tools.hpp"

#include <cmath>

namespace autonomy {
namespace localization {
namespace atlas {
namespace {

Mat33 Skew(const Vec3& v) {
    Mat33 m;
    m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return m;
}

Mat33 CameraK(const std::shared_ptr<KeyFrame>& keyframe) {
    Mat33 K = Mat33::Identity();
    K(0, 0) = keyframe->fx;
    K(1, 1) = keyframe->fy;
    K(0, 2) = keyframe->cx;
    K(1, 2) = keyframe->cy;
    return K;
}

Eigen::Matrix<double, 3, 4> PoseMatrix(const SE3& Tcw) {
    Eigen::Matrix<double, 3, 4> P;
    P.block<3, 3>(0, 0) = Tcw.rotation();
    P.col(3) = Tcw.translation();
    return P;
}

}  // namespace

Mat33 GeometricTools::ComputeF12(const std::shared_ptr<KeyFrame>& keyframe1,
                                 const std::shared_ptr<KeyFrame>& keyframe2) {
    if (!keyframe1 || !keyframe2) {
        return Mat33::Identity();
    }
    const SE3 Tc1w = keyframe1->GetPose();
    const SE3 Tc2w = keyframe2->GetPose();
    const Mat33 Rc1c2 = Tc1w.rotation() * Tc2w.rotation().transpose();
    const Vec3 tc1c2 =
        -Rc1c2 * Tc2w.translation() + Tc1w.translation();
    const Mat33 K1 = CameraK(keyframe1);
    const Mat33 K2 = CameraK(keyframe2);
    return K1.transpose().inverse() * Skew(tc1c2) * Rc1c2 * K2.inverse();
}

bool GeometricTools::Triangulate(const Vec3& x_c1, const Vec3& x_c2,
                                 const Eigen::Matrix<double, 3, 4>& Tc1w,
                                 const Eigen::Matrix<double, 3, 4>& Tc2w,
                                 Vec3* x3d) {
    if (x3d == nullptr) {
        return false;
    }
    Eigen::Matrix4d A;
    A.row(0) = x_c1.x() * Tc1w.row(2) - Tc1w.row(0);
    A.row(1) = x_c1.y() * Tc1w.row(2) - Tc1w.row(1);
    A.row(2) = x_c2.x() * Tc2w.row(2) - Tc2w.row(0);
    A.row(3) = x_c2.y() * Tc2w.row(2) - Tc2w.row(1);

    const Eigen::JacobiSVD<Eigen::Matrix4d> svd(
        A, Eigen::ComputeFullV);
    const Eigen::Vector4d xh = svd.matrixV().col(3);
    if (std::abs(xh.w()) < 1e-12) {
        return false;
    }
    *x3d = xh.head<3>() / xh.w();
    return true;
}

bool GeometricTools::TriangulatePixels(
    const cv::Point2f& uv1, const cv::Point2f& uv2,
    const std::shared_ptr<KeyFrame>& keyframe1,
    const std::shared_ptr<KeyFrame>& keyframe2, Vec3* x3d_world) {
    if (!keyframe1 || !keyframe2 || x3d_world == nullptr) {
        return false;
    }
    return TriangulatePixels(
        uv1, uv2, keyframe1->camera.get(), keyframe2->camera.get(),
        keyframe1->GetPose(), keyframe2->GetPose(), keyframe1->fx,
        keyframe1->fy, keyframe1->cx, keyframe1->cy, keyframe2->fx,
        keyframe2->fy, keyframe2->cx, keyframe2->cy, x3d_world);
}

bool GeometricTools::TriangulatePixels(
    const cv::Point2f& uv1, const cv::Point2f& uv2,
    const sensor::GeometricCamera* cam1, const sensor::GeometricCamera* cam2,
    const SE3& Tcw1, const SE3& Tcw2, float fx1, float fy1, float cx1,
    float cy1, float fx2, float fy2, float cx2, float cy2, Vec3* x3d_world) {
    if (x3d_world == nullptr) {
        return false;
    }
    Vec3 x1;
    Vec3 x2;
    if (cam1) {
        x1 = cam1->Unproject(Vec2(uv1.x, uv1.y), 1.0);
        if (std::abs(x1.z()) > 1e-9) {
            x1 /= x1.z();
        }
    } else {
        x1 = Vec3((uv1.x - cx1) / fx1, (uv1.y - cy1) / fy1, 1.0);
    }
    if (cam2) {
        x2 = cam2->Unproject(Vec2(uv2.x, uv2.y), 1.0);
        if (std::abs(x2.z()) > 1e-9) {
            x2 /= x2.z();
        }
    } else {
        x2 = Vec3((uv2.x - cx2) / fx2, (uv2.y - cy2) / fy2, 1.0);
    }
    return Triangulate(x1, x2, PoseMatrix(Tcw1), PoseMatrix(Tcw2), x3d_world);
}

bool GeometricTools::EpipolarConstrain(const sensor::GeometricCamera* cam1,
                                       const sensor::GeometricCamera* cam2,
                                       const cv::KeyPoint& kp1,
                                       const cv::KeyPoint& kp2,
                                       const Mat33& R12, const Vec3& t12,
                                       float sigma_level, float unc) {
    if (!cam1 && !cam2) {
        return true;
    }
    // Kannala / non-pinhole: TriangulateMatches (ORB KB epipolarConstrain).
    if (cam1 && cam2 &&
        (cam1->type() == sensor::GeometricCamera::Type::kKannalaBrandt ||
         cam2->type() == sensor::GeometricCamera::Type::kKannalaBrandt ||
         cam1->type() != sensor::GeometricCamera::Type::kPinhole ||
         cam2->type() != sensor::GeometricCamera::Type::kPinhole)) {
        Vec3 p3d;
        return TriangulateMatches(*cam1, *cam2, kp1, kp2, R12, t12,
                                  sigma_level, unc, &p3d) > 0.0001f;
    }

    // Pinhole F-matrix distance (ORB Pinhole::epipolarConstrain).
    const double fx1 = cam1 ? cam1->fx() : (cam2 ? cam2->fx() : 1.0);
    const double fy1 = cam1 ? cam1->fy() : (cam2 ? cam2->fy() : 1.0);
    const double cx1 = cam1 ? cam1->cx() : (cam2 ? cam2->cx() : 0.0);
    const double cy1 = cam1 ? cam1->cy() : (cam2 ? cam2->cy() : 0.0);
    const double fx2 = cam2 ? cam2->fx() : fx1;
    const double fy2 = cam2 ? cam2->fy() : fy1;
    const double cx2 = cam2 ? cam2->cx() : cx1;
    const double cy2 = cam2 ? cam2->cy() : cy1;
    Mat33 K1 = Mat33::Identity();
    K1(0, 0) = fx1;
    K1(1, 1) = fy1;
    K1(0, 2) = cx1;
    K1(1, 2) = cy1;
    Mat33 K2 = Mat33::Identity();
    K2(0, 0) = fx2;
    K2(1, 1) = fy2;
    K2(0, 2) = cx2;
    K2(1, 2) = cy2;
    const Mat33 F12 =
        K1.transpose().inverse() * Skew(t12) * R12 * K2.inverse();
    const float a = static_cast<float>(kp1.pt.x * F12(0, 0) +
                                       kp1.pt.y * F12(1, 0) + F12(2, 0));
    const float b = static_cast<float>(kp1.pt.x * F12(0, 1) +
                                       kp1.pt.y * F12(1, 1) + F12(2, 1));
    const float c = static_cast<float>(kp1.pt.x * F12(0, 2) +
                                       kp1.pt.y * F12(1, 2) + F12(2, 2));
    const float num = a * kp2.pt.x + b * kp2.pt.y + c;
    const float den = a * a + b * b;
    if (den < 1e-12f) {
        return false;
    }
    const float dsqr = (num * num) / den;
    return dsqr < 3.84f * unc;
}

float GeometricTools::TriangulateMatches(
    const sensor::GeometricCamera& cam1, const sensor::GeometricCamera& cam2,
    const cv::KeyPoint& kp1, const cv::KeyPoint& kp2, const Mat33& R12,
    const Vec3& t12, float sigma_level, float unc, Vec3* p3d_cam1) {
    if (p3d_cam1 == nullptr) {
        return -1.f;
    }
    const Vec3 r1 =
        cam1.Unproject(Vec2(kp1.pt.x, kp1.pt.y), 1.0);
    const Vec3 r2 =
        cam2.Unproject(Vec2(kp2.pt.x, kp2.pt.y), 1.0);
    const Vec3 r21 = R12 * r2;
    const double cos_parallax =
        r1.dot(r21) / (r1.norm() * r21.norm() + 1e-12);
    if (cos_parallax > 0.9998) {
        return -1.f;
    }

    // Normalized plane coords (ORB KannalaBrandt8 uses (x/z, y/z)).
    auto ToPlane = [](const Vec3& r) -> Vec3 {
        if (std::abs(r.z()) > 1e-9) {
            return Vec3(r.x() / r.z(), r.y() / r.z(), 1.0);
        }
        return Vec3(r.x(), r.y(), 1.0);
    };
    const Vec3 x1 = ToPlane(r1);
    const Vec3 x2 = ToPlane(r2);

    Eigen::Matrix<double, 3, 4> Tcw1;
    Tcw1 << Mat33::Identity(), Vec3::Zero();
    const Mat33 R21 = R12.transpose();
    Eigen::Matrix<double, 3, 4> Tcw2;
    Tcw2 << R21, -R21 * t12;

    Vec3 x3d;
    if (!Triangulate(x1, x2, Tcw1, Tcw2, &x3d)) {
        return -1.f;
    }
    const float z1 = static_cast<float>(x3d.z());
    if (z1 <= 0.f) {
        return -2.f;
    }
    const float z2 =
        static_cast<float>(R21.row(2).dot(x3d) + Tcw2(2, 3));
    if (z2 <= 0.f) {
        return -3.f;
    }

    const Vec2 uv1 = cam1.Project(x3d);
    const float err1 = static_cast<float>(
        (uv1.x() - kp1.pt.x) * (uv1.x() - kp1.pt.x) +
        (uv1.y() - kp1.pt.y) * (uv1.y() - kp1.pt.y));
    if (err1 > 5.991f * sigma_level) {
        return -4.f;
    }

    const Vec3 x3d2 = R21 * x3d + Tcw2.col(3);
    const Vec2 uv2 = cam2.Project(x3d2);
    const float err2 = static_cast<float>(
        (uv2.x() - kp2.pt.x) * (uv2.x() - kp2.pt.x) +
        (uv2.y() - kp2.pt.y) * (uv2.y() - kp2.pt.y));
    if (err2 > 5.991f * unc) {
        return -5.f;
    }

    *p3d_cam1 = x3d;
    return z1;
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
