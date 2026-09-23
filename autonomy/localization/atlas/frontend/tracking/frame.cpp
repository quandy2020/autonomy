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
 *
 * Structure adapted from ORB-SLAM3 Frame.cc (RGB-D path).
 */

/**
 * @file frame.cpp
 * @brief Frame implementation: ORB extract, stereo/RGB-D depth, BoW, grid, IMU pose helpers.
 */

#include "autonomy/localization/atlas/frontend/tracking/frame.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "autonomy/localization/atlas/common/geometric_tools.hpp"
#include "autonomy/localization/atlas/frontend/match/orb_matcher.hpp"
#include "autonomy/localization/atlas/sensor/imu/pose.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace tracking {

long unsigned int Frame::next_id = 0;

Frame::Frame() = default;

Frame::Frame(const cv::Mat& gray, const cv::Mat& depth, double timestamp_sec,
             feature::OrbExtractor* extractor, float fx_in, float fy_in,
             float cx_in, float cy_in, float bf, float th_depth)
    : timestamp(timestamp_sec),
      fx(fx_in),
      fy(fy_in),
      cx(cx_in),
      cy(cy_in),
      inv_fx(1.f / fx_in),
      inv_fy(1.f / fy_in),
      baseline_times_fx(bf),
      baseline_meters(bf / fx_in),
      depth_threshold(th_depth),
      orb_extractor_left(extractor) {
    id = next_id++;
    ExtractOrb(0, gray);
    num_keypoints = static_cast<int>(keypoints.size());
    keypoints_undistorted = keypoints;
    right_coordinate.assign(num_keypoints, -1.f);
    depths.assign(num_keypoints, -1.f);
    map_points.assign(num_keypoints, nullptr);
    outliers.assign(num_keypoints, false);

    if (extractor != nullptr) {
        scale_levels = extractor->GetLevels();
        scale_factor = extractor->GetScaleFactor();
        log_scale_factor = std::log(std::max(1.0001f, scale_factor));
        scale_factors = extractor->GetScaleFactors();
        inverse_scale_factors = extractor->GetInverseScaleFactors();
        level_sigma2 = extractor->GetScaleSigmaSquares();
        inverse_level_sigma2 = extractor->GetInverseScaleSigmaSquares();
    }

    ComputeStereoFromRgbd(depth);
    AssignFeaturesToGrid();
}

Frame::Frame(const cv::Mat& left_gray, const cv::Mat& right_gray,
             double timestamp_sec, feature::OrbExtractor* left_extractor,
             feature::OrbExtractor* right_extractor, float fx_in, float fy_in,
             float cx_in, float cy_in, float bf, float th_depth,
             const std::vector<int>& lapping_l,
             const std::vector<int>& lapping_r)
    : timestamp(timestamp_sec),
      fx(fx_in),
      fy(fy_in),
      cx(cx_in),
      cy(cy_in),
      inv_fx(1.f / fx_in),
      inv_fy(1.f / fy_in),
      baseline_times_fx(bf),
      baseline_meters(bf / fx_in),
      depth_threshold(th_depth),
      orb_extractor_left(left_extractor),
      orb_extractor_right(right_extractor) {
    if (lapping_l.size() >= 2) {
        lapping_left = {lapping_l[0], lapping_l[1]};
    }
    if (lapping_r.size() >= 2) {
        lapping_right = {lapping_r[0], lapping_r[1]};
    }
    id = next_id++;
    ExtractOrb(0, left_gray);
    ExtractOrb(1, right_gray);
    num_keypoints = static_cast<int>(keypoints.size());
    keypoints_undistorted = keypoints;
    right_coordinate.assign(num_keypoints, -1.f);
    depths.assign(num_keypoints, -1.f);
    map_points.assign(num_keypoints, nullptr);
    outliers.assign(num_keypoints, false);

    if (left_extractor != nullptr) {
        scale_levels = left_extractor->GetLevels();
        scale_factor = left_extractor->GetScaleFactor();
        log_scale_factor = std::log(std::max(1.0001f, scale_factor));
        scale_factors = left_extractor->GetScaleFactors();
        inverse_scale_factors = left_extractor->GetInverseScaleFactors();
        level_sigma2 = left_extractor->GetScaleSigmaSquares();
        inverse_level_sigma2 = left_extractor->GetInverseScaleSigmaSquares();
    }

    ComputeStereoMatches();
    AssignFeaturesToGrid();
}

Frame::Frame(const cv::Mat& gray, double timestamp_sec,
             feature::OrbExtractor* extractor, float fx_in, float fy_in,
             float cx_in, float cy_in, float bf, float th_depth)
    : timestamp(timestamp_sec),
      fx(fx_in),
      fy(fy_in),
      cx(cx_in),
      cy(cy_in),
      inv_fx(1.f / fx_in),
      inv_fy(1.f / fy_in),
      baseline_times_fx(bf),
      baseline_meters(bf / fx_in),
      depth_threshold(th_depth),
      orb_extractor_left(extractor) {
    id = next_id++;
    ExtractOrb(0, gray);
    num_keypoints = static_cast<int>(keypoints.size());
    keypoints_undistorted = keypoints;
    right_coordinate.assign(num_keypoints, -1.f);
    depths.assign(num_keypoints, -1.f);
    map_points.assign(num_keypoints, nullptr);
    outliers.assign(num_keypoints, false);

    if (extractor != nullptr) {
        scale_levels = extractor->GetLevels();
        scale_factor = extractor->GetScaleFactor();
        log_scale_factor = std::log(std::max(1.0001f, scale_factor));
        scale_factors = extractor->GetScaleFactors();
        inverse_scale_factors = extractor->GetInverseScaleFactors();
        level_sigma2 = extractor->GetScaleSigmaSquares();
        inverse_level_sigma2 = extractor->GetInverseScaleSigmaSquares();
    }
    AssignFeaturesToGrid();
}

void Frame::ExtractOrb(int flag, const cv::Mat& image) {
    std::vector<int> lapping_area =
        (flag == 0) ? lapping_left : lapping_right;
    if (lapping_area.size() < 2) {
        lapping_area = {0, 1000};
    }
    if (flag == 0) {
        mono_left = (*orb_extractor_left)(image, cv::Mat(), keypoints,
                                          descriptors, lapping_area);
    } else {
        mono_right = (*orb_extractor_right)(
            image, cv::Mat(), keypoints_right, descriptors_right, lapping_area);
    }
}

void Frame::ComputeStereoFromRgbd(const cv::Mat& depth) {
    for (int i = 0; i < num_keypoints; ++i) {
        const cv::KeyPoint& keypoint = keypoints[static_cast<size_t>(i)];
        const int row = cvRound(keypoint.pt.y);
        const int col = cvRound(keypoint.pt.x);
        if (row < 0 || col < 0 || row >= depth.rows || col >= depth.cols) {
            continue;
        }
        const float depth_value = depth.at<float>(row, col);
        if (depth_value > 0.f) {
            depths[static_cast<size_t>(i)] = depth_value;
            const float u_undist =
                keypoints_undistorted[static_cast<size_t>(i)].pt.x;
            right_coordinate[static_cast<size_t>(i)] =
                u_undist - baseline_times_fx / depth_value;
        }
    }
}

void Frame::ComputeStereoMatches() {
    right_coordinate.assign(static_cast<size_t>(num_keypoints), -1.f);
    depths.assign(static_cast<size_t>(num_keypoints), -1.f);
    if (num_keypoints <= 0 || orb_extractor_left == nullptr ||
        orb_extractor_right == nullptr || descriptors.empty() ||
        descriptors_right.empty() || keypoints_right.empty() ||
        orb_extractor_left->mvImagePyramid.empty() ||
        orb_extractor_right->mvImagePyramid.empty()) {
        return;
    }

    constexpr int kThHigh = feature::OrbMatcher::kThHigh;
    constexpr int kThLow = feature::OrbMatcher::kThLow;
    const int th_orb_dist = (kThHigh + kThLow) / 2;

    const int n_rows = orb_extractor_left->mvImagePyramid[0].rows;
    if (n_rows <= 0) {
        return;
    }
    std::vector<std::vector<size_t>> row_indices(static_cast<size_t>(n_rows));
    for (int i = 0; i < n_rows; ++i) {
        row_indices[static_cast<size_t>(i)].reserve(200);
    }

    const int n_right = static_cast<int>(keypoints_right.size());
    for (int i_r = 0; i_r < n_right; ++i_r) {
        const cv::KeyPoint& kp = keypoints_right[static_cast<size_t>(i_r)];
        const float kp_y = kp.pt.y;
        const float r =
            2.0f * scale_factors[static_cast<size_t>(std::max(0, kp.octave))];
        const int max_r = static_cast<int>(std::ceil(kp_y + r));
        const int min_r = static_cast<int>(std::floor(kp_y - r));
        for (int yi = min_r; yi <= max_r; ++yi) {
            if (yi < 0 || yi >= n_rows) {
                continue;
            }
            row_indices[static_cast<size_t>(yi)].push_back(
                static_cast<size_t>(i_r));
        }
    }

    const float min_z = baseline_meters;
    const float min_d = 0.f;
    const float max_d =
        (min_z > 1e-6f) ? (baseline_times_fx / min_z) : baseline_times_fx;

    std::vector<std::pair<int, int>> dist_idx;
    dist_idx.reserve(static_cast<size_t>(num_keypoints));

    for (int i_l = 0; i_l < num_keypoints; ++i_l) {
        const cv::KeyPoint& kp_l = keypoints[static_cast<size_t>(i_l)];
        const int level_l = kp_l.octave;
        const float v_l = kp_l.pt.y;
        const float u_l = kp_l.pt.x;
        const int row = cvRound(v_l);
        if (row < 0 || row >= n_rows) {
            continue;
        }
        const auto& candidates = row_indices[static_cast<size_t>(row)];
        if (candidates.empty()) {
            continue;
        }

        const float min_u = u_l - max_d;
        const float max_u = u_l - min_d;
        if (max_u < 0.f) {
            continue;
        }

        int best_dist = kThHigh;
        size_t best_idx_r = 0;
        const cv::Mat d_l = descriptors.row(i_l);

        for (const size_t i_r : candidates) {
            const cv::KeyPoint& kp_r = keypoints_right[i_r];
            if (kp_r.octave < level_l - 1 || kp_r.octave > level_l + 1) {
                continue;
            }
            const float u_r = kp_r.pt.x;
            if (u_r < min_u || u_r > max_u) {
                continue;
            }
            const cv::Mat d_r =
                descriptors_right.row(static_cast<int>(i_r));
            const int dist = feature::OrbMatcher::DescriptorDistance(d_l, d_r);
            if (dist < best_dist) {
                best_dist = dist;
                best_idx_r = i_r;
            }
        }

        if (best_dist >= th_orb_dist) {
            continue;
        }

        // Subpixel match by correlation (ORB-SLAM3).
        const float u_r0 = keypoints_right[best_idx_r].pt.x;
        const float scale_factor_inv =
            inverse_scale_factors[static_cast<size_t>(
                std::max(0, std::min(level_l,
                                     static_cast<int>(inverse_scale_factors
                                                          .size()) -
                                         1)))];
        const float scaled_u_l = std::round(kp_l.pt.x * scale_factor_inv);
        const float scaled_v_l = std::round(kp_l.pt.y * scale_factor_inv);
        const float scaled_u_r0 = std::round(u_r0 * scale_factor_inv);

        constexpr int w = 5;
        constexpr int L = 5;
        if (level_l < 0 ||
            level_l >=
                static_cast<int>(orb_extractor_left->mvImagePyramid.size()) ||
            level_l >= static_cast<int>(
                           orb_extractor_right->mvImagePyramid.size())) {
            continue;
        }
        const cv::Mat& pyr_l = orb_extractor_left->mvImagePyramid[static_cast<
            size_t>(level_l)];
        const cv::Mat& pyr_r = orb_extractor_right->mvImagePyramid[static_cast<
            size_t>(level_l)];
        if (scaled_v_l - w < 0 || scaled_v_l + w + 1 > pyr_l.rows ||
            scaled_u_l - w < 0 || scaled_u_l + w + 1 > pyr_l.cols) {
            continue;
        }
        cv::Mat IL =
            pyr_l.rowRange(static_cast<int>(scaled_v_l - w),
                           static_cast<int>(scaled_v_l + w + 1))
                .colRange(static_cast<int>(scaled_u_l - w),
                          static_cast<int>(scaled_u_l + w + 1));

        int best_corr = std::numeric_limits<int>::max();
        int best_inc_r = 0;
        std::vector<float> v_dists(static_cast<size_t>(2 * L + 1));

        const float ini_u = scaled_u_r0 + L - w;
        const float end_u = scaled_u_r0 + L + w + 1;
        if (ini_u < 0 || end_u >= pyr_r.cols) {
            continue;
        }

        for (int inc_r = -L; inc_r <= L; ++inc_r) {
            if (scaled_v_l - w < 0 || scaled_v_l + w + 1 > pyr_r.rows ||
                scaled_u_r0 + inc_r - w < 0 ||
                scaled_u_r0 + inc_r + w + 1 > pyr_r.cols) {
                continue;
            }
            cv::Mat IR =
                pyr_r
                    .rowRange(static_cast<int>(scaled_v_l - w),
                              static_cast<int>(scaled_v_l + w + 1))
                    .colRange(static_cast<int>(scaled_u_r0 + inc_r - w),
                              static_cast<int>(scaled_u_r0 + inc_r + w + 1));
            const float dist =
                static_cast<float>(cv::norm(IL, IR, cv::NORM_L1));
            if (dist < best_corr) {
                best_corr = static_cast<int>(dist);
                best_inc_r = inc_r;
            }
            v_dists[static_cast<size_t>(L + inc_r)] = dist;
        }

        if (best_inc_r == -L || best_inc_r == L) {
            continue;
        }

        const float dist1 = v_dists[static_cast<size_t>(L + best_inc_r - 1)];
        const float dist2 = v_dists[static_cast<size_t>(L + best_inc_r)];
        const float dist3 = v_dists[static_cast<size_t>(L + best_inc_r + 1)];
        const float denom = 2.0f * (dist1 + dist3 - 2.0f * dist2);
        if (std::fabs(denom) < 1e-6f) {
            continue;
        }
        const float delta_r = (dist1 - dist3) / denom;
        if (delta_r < -1.f || delta_r > 1.f) {
            continue;
        }

        const float scale =
            scale_factors[static_cast<size_t>(std::max(0, level_l))];
        float best_u_r =
            scale * (scaled_u_r0 + static_cast<float>(best_inc_r) + delta_r);
        float disparity = u_l - best_u_r;
        if (disparity >= min_d && disparity < max_d) {
            if (disparity <= 0.f) {
                disparity = 0.01f;
                best_u_r = u_l - 0.01f;
            }
            depths[static_cast<size_t>(i_l)] = baseline_times_fx / disparity;
            right_coordinate[static_cast<size_t>(i_l)] = best_u_r;
            dist_idx.emplace_back(best_corr, i_l);
        }
    }

    if (dist_idx.empty()) {
        return;
    }
    std::sort(dist_idx.begin(), dist_idx.end());
    const float median =
        static_cast<float>(dist_idx[dist_idx.size() / 2].first);
    const float th_dist = 1.5f * 1.4f * median;
    for (int i = static_cast<int>(dist_idx.size()) - 1; i >= 0; --i) {
        if (dist_idx[static_cast<size_t>(i)].first < th_dist) {
            break;
        }
        const int idx = dist_idx[static_cast<size_t>(i)].second;
        right_coordinate[static_cast<size_t>(idx)] = -1.f;
        depths[static_cast<size_t>(idx)] = -1.f;
    }
}

void Frame::ComputeStereoFishEyeMatches() {
    const size_t n_left = static_cast<size_t>(num_keypoints);
    const size_t n_right = keypoints_right.size();
    left_to_right_match.assign(n_left, -1);
    right_to_left_match.assign(n_right, -1);
    depths.assign(n_left, -1.f);
    right_coordinate.assign(n_left, -1.f);
    stereo_3d_points.assign(n_left, Vec3::Zero());

    if (!camera || !camera2 || descriptors.empty() ||
        descriptors_right.empty() || n_left == 0 || n_right == 0) {
        return;
    }

    // Match only overlap (stereo) keypoints — ORB monoLeft / monoRight.
    const int ml = std::max(0, std::min(mono_left, num_keypoints));
    const int mr =
        std::max(0, std::min(mono_right, static_cast<int>(n_right)));
    if (ml >= num_keypoints || mr >= static_cast<int>(n_right)) {
        return;
    }
    const cv::Mat stereo_desc_l = descriptors.rowRange(ml, descriptors.rows);
    const cv::Mat stereo_desc_r =
        descriptors_right.rowRange(mr, descriptors_right.rows);
    if (stereo_desc_l.rows <= 0 || stereo_desc_r.rows <= 0) {
        return;
    }

    std::vector<std::vector<cv::DMatch>> matches;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.knnMatch(stereo_desc_l, stereo_desc_r, matches, 2);

    const Mat33 R12 = T_c1_c2.linear();
    const Vec3 t12 = T_c1_c2.translation();

    for (const auto& knn : matches) {
        if (knn.size() < 2 || knn[0].distance >= knn[1].distance * 0.7f) {
            continue;
        }
        const int i_l = knn[0].queryIdx + ml;
        const int i_r = knn[0].trainIdx + mr;
        if (i_l < 0 || i_l >= num_keypoints || i_r < 0 ||
            i_r >= static_cast<int>(n_right)) {
            continue;
        }
        const int oct_l = keypoints[static_cast<size_t>(i_l)].octave;
        const int oct_r = keypoints_right[static_cast<size_t>(i_r)].octave;
        float sigma1 = 1.f;
        float sigma2 = 1.f;
        if (!level_sigma2.empty()) {
            const int max_oct =
                static_cast<int>(level_sigma2.size()) - 1;
            sigma1 = level_sigma2[static_cast<size_t>(
                std::max(0, std::min(oct_l, max_oct)))];
            sigma2 = level_sigma2[static_cast<size_t>(
                std::max(0, std::min(oct_r, max_oct)))];
        }
        Vec3 p3d;
        const float depth = GeometricTools::TriangulateMatches(
            *camera, *camera2, keypoints[static_cast<size_t>(i_l)],
            keypoints_right[static_cast<size_t>(i_r)], R12, t12, sigma1,
            sigma2, &p3d);
        if (depth <= 1e-4f) {
            continue;
        }
        left_to_right_match[static_cast<size_t>(i_l)] = i_r;
        right_to_left_match[static_cast<size_t>(i_r)] = i_l;
        depths[static_cast<size_t>(i_l)] = depth;
        stereo_3d_points[static_cast<size_t>(i_l)] = p3d;
        if (baseline_times_fx > 1e-6f && depth > 1e-6f) {
            right_coordinate[static_cast<size_t>(i_l)] =
                keypoints_undistorted[static_cast<size_t>(i_l)].pt.x -
                baseline_times_fx / depth;
        }
    }
}

void Frame::FinalizeFisheyeStereo() {
    if (!camera2 || keypoints_right.empty() || descriptors_right.empty() ||
        descriptors.empty() || num_keypoints <= 0) {
        num_left = -1;
        num_right = 0;
        return;
    }
    num_left = num_keypoints;
    num_right = static_cast<int>(keypoints_right.size());
    cv::vconcat(descriptors, descriptors_right, descriptors);
    const size_t n_total =
        static_cast<size_t>(num_left) + static_cast<size_t>(num_right);
    map_points.resize(n_total, nullptr);
    outliers.resize(n_total, false);
    AssignFeaturesToGrid();
}

cv::KeyPoint Frame::GetKeyPoint(int index) const {
    if (num_left >= 0 && index >= num_left) {
        const int ri = index - num_left;
        if (ri >= 0 && ri < static_cast<int>(keypoints_right.size())) {
            return keypoints_right[static_cast<size_t>(ri)];
        }
        return {};
    }
    if (index >= 0 &&
        index < static_cast<int>(keypoints_undistorted.size())) {
        return keypoints_undistorted[static_cast<size_t>(index)];
    }
    if (index >= 0 && index < static_cast<int>(keypoints.size())) {
        return keypoints[static_cast<size_t>(index)];
    }
    return {};
}

bool Frame::UnprojectStereo(int index, Vec3* point_world) const {
    if (point_world == nullptr || index < 0 || index >= num_keypoints) {
        return false;
    }
    const float depth = depths[static_cast<size_t>(index)];
    if (depth <= 0.f) {
        return false;
    }
    Vec3 point_camera;
    if (index < static_cast<int>(stereo_3d_points.size()) &&
        index < static_cast<int>(left_to_right_match.size()) &&
        left_to_right_match[static_cast<size_t>(index)] >= 0) {
        point_camera = stereo_3d_points[static_cast<size_t>(index)];
    } else {
        const float u = keypoints_undistorted[static_cast<size_t>(index)].pt.x;
        const float v = keypoints_undistorted[static_cast<size_t>(index)].pt.y;
        const float x = (u - cx) * depth * inv_fx;
        const float y = (v - cy) * depth * inv_fy;
        point_camera = Vec3(x, y, depth);
    }
    // ORB-SLAM3 stores T_cw; world = T_cw.inverse() * X_c
    *point_world = pose_camera_world_.inverse() * point_camera;
    return true;
}

void Frame::SetPose(const SE3& pose_camera_world) {
    pose_camera_world_ = pose_camera_world;
    has_pose_ = true;
}

Vec3 Frame::GetCameraCenter() const {
    return pose_camera_world_.inverse().translation();
}

SE3 Frame::PoseWorldBody() const {
    return GetImuPose();
}

SE3 Frame::GetImuPose() const {
    return sensor::imu::CameraPoseToImuPose(
        pose_camera_world_, sensor::imu::DefaultCalibOr(imu_calib));
}

void Frame::SetImuPose(const SE3& T_wb) {
    pose_camera_world_ = sensor::imu::ImuPoseToCameraPose(
        T_wb, sensor::imu::DefaultCalibOr(imu_calib));
    has_pose_ = true;
}

void Frame::SetVocabulary(
    const std::shared_ptr<feature::OrbVocabulary>& vocabulary) {
    vocabulary_ = vocabulary;
}

void Frame::ComputeBoW() {
    if (bow_ready_ || !vocabulary_ || !vocabulary_->is_valid() ||
        descriptors.empty()) {
        return;
    }
    vocabulary_->Transform(descriptors, &bow_vector_, &feat_vector_);
    bow_ready_ = !bow_vector_.empty();
}

void Frame::AssignFeaturesToGrid() {
    // Preserve bounds from ComputeImageBounds when available.
    if (max_x_ <= min_x_ || max_y_ <= min_y_) {
        min_x_ = 0.f;
        min_y_ = 0.f;
        max_x_ = 0.f;
        max_y_ = 0.f;
        for (const auto& keypoint : keypoints_undistorted) {
            max_x_ = std::max(max_x_, keypoint.pt.x);
            max_y_ = std::max(max_y_, keypoint.pt.y);
        }
        if (max_x_ <= min_x_ || max_y_ <= min_y_) {
            max_x_ =
                static_cast<float>(std::max(1, static_cast<int>(max_x_) + 1));
            max_y_ =
                static_cast<float>(std::max(1, static_cast<int>(max_y_) + 1));
        }
    }
    grid_element_width_inv_ =
        static_cast<float>(kFrameGridCols) / (max_x_ - min_x_);
    grid_element_height_inv_ =
        static_cast<float>(kFrameGridRows) / (max_y_ - min_y_);

    for (int i = 0; i < kFrameGridCols; ++i) {
        for (int j = 0; j < kFrameGridRows; ++j) {
            grid_[i][j].clear();
            grid_right_[i][j].clear();
        }
    }
    const int n_left =
        HasDualCameraIndex() ? num_left : num_keypoints;
    for (int i = 0; i < n_left; ++i) {
        int pos_x = 0;
        int pos_y = 0;
        const cv::KeyPoint& kp =
            (i < static_cast<int>(keypoints_undistorted.size()))
                ? keypoints_undistorted[static_cast<size_t>(i)]
                : keypoints[static_cast<size_t>(i)];
        if (PositionInGrid(kp, &pos_x, &pos_y)) {
            grid_[pos_x][pos_y].push_back(static_cast<size_t>(i));
        }
    }
    if (HasDualCameraIndex()) {
        for (int i = 0; i < num_right; ++i) {
            int pos_x = 0;
            int pos_y = 0;
            if (PositionInGrid(keypoints_right[static_cast<size_t>(i)],
                               &pos_x, &pos_y)) {
                grid_right_[pos_x][pos_y].push_back(static_cast<size_t>(i));
            }
        }
    }
}

bool Frame::PositionInGrid(const cv::KeyPoint& keypoint, int* pos_x,
                           int* pos_y) const {
    *pos_x = static_cast<int>(std::floor((keypoint.pt.x - min_x_) *
                                         grid_element_width_inv_));
    *pos_y = static_cast<int>(std::floor((keypoint.pt.y - min_y_) *
                                         grid_element_height_inv_));
    if (*pos_x < 0 || *pos_x >= kFrameGridCols || *pos_y < 0 ||
        *pos_y >= kFrameGridRows) {
        return false;
    }
    return true;
}

std::vector<size_t> Frame::GetFeaturesInArea(float x, float y, float radius,
                                             int min_level, int max_level,
                                             bool right) const {
    std::vector<size_t> indices;
    const int min_cell_x = std::max(
        0, static_cast<int>(std::floor((x - radius - min_x_) *
                                       grid_element_width_inv_)));
    const int max_cell_x = std::min(
        kFrameGridCols - 1,
        static_cast<int>(std::floor((x + radius - min_x_) *
                                    grid_element_width_inv_)));
    const int min_cell_y = std::max(
        0, static_cast<int>(std::floor((y - radius - min_y_) *
                                       grid_element_height_inv_)));
    const int max_cell_y = std::min(
        kFrameGridRows - 1,
        static_cast<int>(std::floor((y + radius - min_y_) *
                                    grid_element_height_inv_)));
    for (int ix = min_cell_x; ix <= max_cell_x; ++ix) {
        for (int iy = min_cell_y; iy <= max_cell_y; ++iy) {
            const auto& cell = right ? grid_right_[ix][iy] : grid_[ix][iy];
            for (const size_t index : cell) {
                const cv::KeyPoint& keypoint =
                    right ? keypoints_right[index]
                          : ((index < keypoints_undistorted.size())
                                 ? keypoints_undistorted[index]
                                 : keypoints[index]);
                if (min_level >= 0 && keypoint.octave < min_level) {
                    continue;
                }
                if (max_level >= 0 && keypoint.octave > max_level) {
                    continue;
                }
                const float dx = keypoint.pt.x - x;
                const float dy = keypoint.pt.y - y;
                if (dx * dx + dy * dy < radius * radius) {
                    indices.push_back(index);
                }
            }
        }
    }
    return indices;
}

bool Frame::IsInFrustum(const std::shared_ptr<MapPoint>& map_point,
                        float viewing_cos_limit, bool right) {
    if (!map_point || !has_pose_) {
        return false;
    }
    if (right) {
        if (!HasDualCameraIndex() || !camera2) {
            map_point->track_in_view_r = false;
            return false;
        }
        map_point->track_in_view_r = false;
        map_point->track_proj_xr = -1.f;
        map_point->track_proj_yr = -1.f;
        map_point->track_scale_level_r = -1;
    } else {
        map_point->track_in_view = false;
        map_point->track_proj_x = -1.f;
        map_point->track_proj_y = -1.f;
    }

    const Vec3 Pw = map_point->GetWorldPos();
    // Left: Tcw; right: Trl * Tcw (ORB isInFrustumChecks).
    const SE3 T_view =
        right ? (T_c1_c2.inverse() * pose_camera_world_) : pose_camera_world_;
    const Vec3 Pc = T_view * Pw;

    const sensor::GeometricCamera* cam =
        right ? camera2.get() : camera.get();
    float u = 0.f;
    float v = 0.f;
    if (cam) {
        if (!cam->IsInValidRange(Pc)) {
            return false;
        }
        const Vec2 uv = cam->Project(Pc);
        u = static_cast<float>(uv.x());
        v = static_cast<float>(uv.y());
    } else {
        if (Pc.z() <= 0.0) {
            return false;
        }
        const float invz = static_cast<float>(1.0 / Pc.z());
        u = fx * static_cast<float>(Pc.x()) * invz + cx;
        v = fy * static_cast<float>(Pc.y()) * invz + cy;
    }
    if (u < min_x_ || u > max_x_ || v < min_y_ || v > max_y_) {
        return false;
    }

    const float invz =
        (Pc.z() > 1e-6) ? static_cast<float>(1.0 / Pc.z()) : 0.f;

    Vec3 Ow;
    if (right) {
        const SE3 Twc = pose_camera_world_.inverse();
        Ow = Twc * T_c1_c2.translation();
    } else {
        Ow = GetCameraCenter();
    }
    const Vec3 PO = Pw - Ow;
    const float dist = static_cast<float>(PO.norm());
    if (dist < map_point->GetMinDistanceInvariance() ||
        dist > map_point->GetMaxDistanceInvariance()) {
        return false;
    }

    const Vec3 Pn = map_point->GetNormal();
    const float view_cos =
        static_cast<float>(PO.dot(Pn) / std::max(1e-6, PO.norm()));
    if (view_cos < viewing_cos_limit) {
        return false;
    }

    const int level =
        map_point->PredictScale(dist, scale_levels, log_scale_factor);
    if (right) {
        map_point->track_in_view_r = true;
        map_point->track_proj_xr = u;
        map_point->track_proj_yr = v;
        map_point->track_depth_r = static_cast<float>(Pc.norm());
        map_point->track_view_cos_r = view_cos;
        map_point->track_scale_level_r = level;
    } else {
        map_point->track_in_view = true;
        map_point->track_proj_x = u;
        map_point->track_proj_y = v;
        map_point->track_proj_xr =
            HasDualCameraIndex() ? map_point->track_proj_xr
                                 : (u - baseline_times_fx * invz);
        map_point->track_depth = static_cast<float>(Pc.norm());
        map_point->track_view_cos = view_cos;
        map_point->track_scale_level = level;
    }
    return true;
}

void Frame::ApplyDistortion(const cv::Mat& dist) {
    if (dist.empty()) {
        dist_coef = cv::Mat::zeros(4, 1, CV_32F);
    } else {
        dist.convertTo(dist_coef, CV_32F);
    }
    UndistortKeyPoints();
    // Prefer camera image size; else infer from keypoint extents after undistort.
    int w = 0;
    int h = 0;
    if (camera) {
        w = camera->width();
        h = camera->height();
    }
    if (w > 0 && h > 0) {
        ComputeImageBounds(w, h);
    }
    AssignFeaturesToGrid();
}

void Frame::ComputeImageBounds(int image_width, int image_height) {
    if (image_width <= 0 || image_height <= 0) {
        return;
    }
    const bool zero_dist =
        dist_coef.empty() ||
        (dist_coef.total() >= 1 && std::fabs(dist_coef.at<float>(0)) < 1e-12f);
    if (zero_dist) {
        min_x_ = 0.f;
        min_y_ = 0.f;
        max_x_ = static_cast<float>(image_width);
        max_y_ = static_cast<float>(image_height);
        return;
    }
    cv::Mat K = (cv::Mat_<float>(3, 3) << fx, 0.f, cx, 0.f, fy, cy, 0.f, 0.f,
                 1.f);
    cv::Mat mat(4, 2, CV_32F);
    mat.at<float>(0, 0) = 0.f;
    mat.at<float>(0, 1) = 0.f;
    mat.at<float>(1, 0) = static_cast<float>(image_width);
    mat.at<float>(1, 1) = 0.f;
    mat.at<float>(2, 0) = 0.f;
    mat.at<float>(2, 1) = static_cast<float>(image_height);
    mat.at<float>(3, 0) = static_cast<float>(image_width);
    mat.at<float>(3, 1) = static_cast<float>(image_height);
    mat = mat.reshape(2);
    cv::undistortPoints(mat, mat, K, dist_coef, cv::Mat(), K);
    mat = mat.reshape(1);
    min_x_ = std::min(mat.at<float>(0, 0), mat.at<float>(2, 0));
    max_x_ = std::max(mat.at<float>(1, 0), mat.at<float>(3, 0));
    min_y_ = std::min(mat.at<float>(0, 1), mat.at<float>(1, 1));
    max_y_ = std::max(mat.at<float>(2, 1), mat.at<float>(3, 1));
}

void Frame::UndistortKeyPoints() {
    if (num_keypoints <= 0 || keypoints.empty()) {
        keypoints_undistorted = keypoints;
        return;
    }
    const bool zero_dist =
        dist_coef.empty() ||
        (dist_coef.total() >= 1 && std::fabs(dist_coef.at<float>(0)) < 1e-12f);
    if (zero_dist) {
        keypoints_undistorted = keypoints;
        return;
    }

    cv::Mat K = (cv::Mat_<float>(3, 3) << fx, 0.f, cx, 0.f, fy, cy, 0.f, 0.f,
                 1.f);
    cv::Mat mat(num_keypoints, 2, CV_32F);
    for (int i = 0; i < num_keypoints; ++i) {
        mat.at<float>(i, 0) = keypoints[static_cast<size_t>(i)].pt.x;
        mat.at<float>(i, 1) = keypoints[static_cast<size_t>(i)].pt.y;
    }
    mat = mat.reshape(2);
    cv::undistortPoints(mat, mat, K, dist_coef, cv::Mat(), K);
    mat = mat.reshape(1);

    keypoints_undistorted.resize(static_cast<size_t>(num_keypoints));
    for (int i = 0; i < num_keypoints; ++i) {
        cv::KeyPoint kp = keypoints[static_cast<size_t>(i)];
        kp.pt.x = mat.at<float>(i, 0);
        kp.pt.y = mat.at<float>(i, 1);
        keypoints_undistorted[static_cast<size_t>(i)] = kp;
    }
}

}  // namespace tracking
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
