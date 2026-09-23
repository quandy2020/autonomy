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
 * @file two_view_reconstruction.cpp
 * @brief Monocular two-view reconstruction (OpenCV essential matrix + triangulation).
 */

#include "autonomy/localization/atlas/frontend/tracking/two_view_reconstruction.hpp"

#include <opencv2/calib3d.hpp>

namespace autonomy {
namespace localization {
namespace atlas {
namespace tracking {

TwoViewReconstruction::TwoViewReconstruction(const Mat33& K, float sigma,
                                             int iterations)
    : K_(K), sigma_(sigma), iterations_(iterations) {}

bool TwoViewReconstruction::Reconstruct(
    const std::vector<cv::KeyPoint>& keys1,
    const std::vector<cv::KeyPoint>& keys2, const std::vector<int>& matches12,
    SE3* T21, std::vector<cv::Point3f>* points3d,
    std::vector<bool>* triangulated) const {
    if (T21 == nullptr || points3d == nullptr || triangulated == nullptr) {
        return false;
    }
    std::vector<cv::Point2f> pts1;
    std::vector<cv::Point2f> pts2;
    std::vector<int> valid_indices;
    pts1.reserve(matches12.size());
    pts2.reserve(matches12.size());
    for (size_t i = 0; i < matches12.size(); ++i) {
        const int j = matches12[i];
        if (j < 0 || j >= static_cast<int>(keys2.size()) ||
            i >= keys1.size()) {
            continue;
        }
        pts1.push_back(keys1[i].pt);
        pts2.push_back(keys2[static_cast<size_t>(j)].pt);
        valid_indices.push_back(static_cast<int>(i));
    }
    if (pts1.size() < 8) {
        return false;
    }

    cv::Mat K =
        (cv::Mat_<double>(3, 3) << K_(0, 0), K_(0, 1), K_(0, 2), K_(1, 0),
         K_(1, 1), K_(1, 2), K_(2, 0), K_(2, 1), K_(2, 2));
    cv::Mat E;
    cv::Mat inliers;
    // OpenCV 5: maxIters comes before the inlier mask.
    E = cv::findEssentialMat(pts1, pts2, K, cv::RANSAC, 0.999,
                             static_cast<double>(sigma_), iterations_,
                             inliers);
    if (E.empty()) {
        return false;
    }

    cv::Mat R;
    cv::Mat t;
    const int inlier_count =
        cv::recoverPose(E, pts1, pts2, K, R, t, inliers);
    if (inlier_count < 8) {
        return false;
    }

    *T21 = SE3Identity();
    T21->linear() << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
    T21->translation() << t.at<double>(0), t.at<double>(1), t.at<double>(2);

    cv::Mat P1 = K * cv::Mat::eye(3, 4, CV_64F);
    cv::Mat P2 = cv::Mat::zeros(3, 4, CV_64F);
    R.copyTo(P2(cv::Rect(0, 0, 3, 3)));
    t.copyTo(P2(cv::Rect(3, 0, 1, 3)));
    P2 = K * P2;

    cv::Mat points4d;
    cv::triangulatePoints(P1, P2, pts1, pts2, points4d);

    points3d->assign(keys1.size(), cv::Point3f(0.f, 0.f, 0.f));
    triangulated->assign(keys1.size(), false);
    int good = 0;
    for (size_t k = 0; k < valid_indices.size(); ++k) {
        if (inliers.at<uchar>(static_cast<int>(k)) == 0) {
            continue;
        }
        cv::Mat col = points4d.col(static_cast<int>(k));
        if (std::fabs(col.at<float>(3)) < 1e-6f) {
            // triangulatePoints may return CV_32F or CV_64F.
        }
        double w = 0.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (points4d.type() == CV_32F) {
            w = col.at<float>(3);
            x = col.at<float>(0);
            y = col.at<float>(1);
            z = col.at<float>(2);
        } else {
            w = col.at<double>(3);
            x = col.at<double>(0);
            y = col.at<double>(1);
            z = col.at<double>(2);
        }
        if (std::fabs(w) < 1e-8) {
            continue;
        }
        x /= w;
        y /= w;
        z /= w;
        if (z <= 0.0) {
            continue;
        }
        const int idx = valid_indices[k];
        (*points3d)[static_cast<size_t>(idx)] =
            cv::Point3f(static_cast<float>(x), static_cast<float>(y),
                        static_cast<float>(z));
        (*triangulated)[static_cast<size_t>(idx)] = true;
        ++good;
    }
    return good >= 50;
}

}  // namespace tracking
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
