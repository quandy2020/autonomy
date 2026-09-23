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
 * Adapted from ORB-SLAM3 ORBmatcher.cc (projection + DescriptorDistance).
 */

/**
 * @file orb_matcher.cpp
 * @brief OrbMatcher implementation: projection/BoW/fusion/triangulation/init matching.
 */

#include "autonomy/localization/atlas/frontend/match/orb_matcher.hpp"

#include <climits>
#include <cmath>
#include <limits>
#include <numeric>
#include <algorithm>
#include <set>

#include "autonomy/localization/atlas/common/geometric_tools.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace feature {

OrbMatcher::OrbMatcher(float nn_ratio, bool check_orientation)
    : nn_ratio_(nn_ratio), check_orientation_(check_orientation) {}

int OrbMatcher::DescriptorDistance(const cv::Mat& a, const cv::Mat& b) {
    const int* pa = a.ptr<int32_t>();
    const int* pb = b.ptr<int32_t>();
    int dist = 0;
    for (int i = 0; i < 8; ++i, ++pa, ++pb) {
        unsigned int v = static_cast<unsigned int>(*pa) ^
                         static_cast<unsigned int>(*pb);
        v = v - ((v >> 1) & 0x55555555u);
        v = (v & 0x33333333u) + ((v >> 2) & 0x33333333u);
        dist += static_cast<int>((((v + (v >> 4)) & 0xF0F0F0Fu) * 0x01010101u) >>
                                 24);
    }
    return dist;
}

float OrbMatcher::RadiusByViewingCos(float view_cos) const {
    return view_cos > 0.998f ? 2.5f : 4.0f;
}

void OrbMatcher::ComputeThreeMaxima(std::vector<int>* histo, int length,
                                    int* ind1, int* ind2, int* ind3) const {
    int max1 = 0;
    int max2 = 0;
    int max3 = 0;
    *ind1 = -1;
    *ind2 = -1;
    *ind3 = -1;
    for (int i = 0; i < length; ++i) {
        const int count = static_cast<int>(histo[i].size());
        if (count > max1) {
            max3 = max2;
            max2 = max1;
            max1 = count;
            *ind3 = *ind2;
            *ind2 = *ind1;
            *ind1 = i;
        } else if (count > max2) {
            max3 = max2;
            max2 = count;
            *ind3 = *ind2;
            *ind2 = i;
        } else if (count > max3) {
            max3 = count;
            *ind3 = i;
        }
    }
    if (*ind1 >= 0 && max2 < 0.1f * max1) {
        *ind2 = -1;
        *ind3 = -1;
    } else if (*ind2 >= 0 && max3 < 0.1f * max1) {
        *ind3 = -1;
    }
}

namespace {

Vec2 ProjectFramePoint(const tracking::Frame& frame, const Vec3& x3Dc,
                       bool right) {
    const auto& cam =
        (right && frame.camera2) ? frame.camera2 : frame.camera;
    if (cam) {
        return cam->Project(x3Dc);
    }
    if (x3Dc.z() <= 1e-8) {
        return Vec2(std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN());
    }
    const float invz = static_cast<float>(1.0 / x3Dc.z());
    return Vec2(frame.fx * x3Dc.x() * invz + frame.cx,
                frame.fy * x3Dc.y() * invz + frame.cy);
}

int LastFrameOctave(const tracking::Frame& last, int i) {
    if (last.HasDualCameraIndex() && i >= last.num_left) {
        const int ri = i - last.num_left;
        if (ri >= 0 && ri < static_cast<int>(last.keypoints_right.size())) {
            return last.keypoints_right[static_cast<size_t>(ri)].octave;
        }
    }
    if (i >= 0 && i < static_cast<int>(last.keypoints.size())) {
        return last.keypoints[static_cast<size_t>(i)].octave;
    }
    return 0;
}

cv::KeyPoint LastFrameKeyPoint(const tracking::Frame& last, int i) {
    if (last.HasDualCameraIndex()) {
        if (i < last.num_left &&
            i < static_cast<int>(last.keypoints.size())) {
            return last.keypoints[static_cast<size_t>(i)];
        }
        const int ri = i - last.num_left;
        if (ri >= 0 && ri < static_cast<int>(last.keypoints_right.size())) {
            return last.keypoints_right[static_cast<size_t>(ri)];
        }
    }
    if (i >= 0 && i < static_cast<int>(last.keypoints_undistorted.size())) {
        return last.keypoints_undistorted[static_cast<size_t>(i)];
    }
    if (i >= 0 && i < static_cast<int>(last.keypoints.size())) {
        return last.keypoints[static_cast<size_t>(i)];
    }
    return {};
}

}  // namespace

int OrbMatcher::SearchByProjection(tracking::Frame& current,
                                   const tracking::Frame& last, float th,
                                   bool monocular) {
    int nmatches = 0;
    std::vector<int> rot_hist[kHistoLength];
    for (int i = 0; i < kHistoLength; ++i) {
        rot_hist[i].reserve(500);
    }
    const float factor = 1.0f / kHistoLength;

    if (!current.has_pose() || !last.has_pose()) {
        return 0;
    }

    const SE3 Tcw = current.GetPose();
    const Vec3 twc = Tcw.inverse().translation();
    const SE3 Tlw = last.GetPose();
    const Vec3 tlc = Tlw * twc;
    const bool forward = tlc.z() > current.baseline_meters && !monocular;
    const bool backward = -tlc.z() > current.baseline_meters && !monocular;
    const bool dual = current.HasDualCameraIndex();
    const int n_last = last.TotalFeatures();
    const SE3 Trl = current.T_c1_c2.inverse();  // ORB GetRelativePoseTrl.

    for (int i = 0; i < n_last; ++i) {
        if (i >= static_cast<int>(last.map_points.size())) {
            break;
        }
        const auto& map_point = last.map_points[static_cast<size_t>(i)];
        if (!map_point ||
            (i < static_cast<int>(last.outliers.size()) &&
             last.outliers[static_cast<size_t>(i)]) ||
            map_point->isBad()) {
            continue;
        }

        const Vec3 x3Dw = map_point->GetWorldPos();
        const Vec3 x3Dc = Tcw * x3Dw;
        if (x3Dc.z() <= 0.0) {
            continue;
        }
        const float invzc = static_cast<float>(1.0 / x3Dc.z());
        const Vec2 uv = ProjectFramePoint(current, x3Dc, false);
        const float u = static_cast<float>(uv.x());
        const float v = static_cast<float>(uv.y());
        // ORB: left out-of-bounds skips the whole MP (including right search).
        if (!std::isfinite(u) || !std::isfinite(v) || u < current.min_x() ||
            u > current.max_x() || v < current.min_y() ||
            v > current.max_y()) {
            continue;
        }

        const int last_octave = LastFrameOctave(last, i);
        float radius = th;
        if (last_octave >= 0 &&
            last_octave < static_cast<int>(current.scale_factors.size())) {
            radius *=
                current.scale_factors[static_cast<size_t>(last_octave)];
        }

        std::vector<size_t> indices;
        if (forward) {
            indices = current.GetFeaturesInArea(u, v, radius, last_octave, -1,
                                                false);
        } else if (backward) {
            indices = current.GetFeaturesInArea(u, v, radius, 0, last_octave,
                                                false);
        } else {
            indices = current.GetFeaturesInArea(
                u, v, radius, last_octave - 1, last_octave + 1, false);
        }

        const cv::Mat dMP = map_point->GetDescriptor();
        if (!indices.empty() && !dMP.empty() && !current.descriptors.empty()) {
            int best_dist = 256;
            int best_idx = -1;
            for (const size_t i2 : indices) {
                if (i2 >= current.map_points.size()) {
                    continue;
                }
                if (current.map_points[i2] &&
                    current.map_points[i2]->Observations() > 0) {
                    continue;
                }
                // Rectified stereo epipolar (not fisheye dual).
                if (!dual && i2 < current.right_coordinate.size() &&
                    current.right_coordinate[i2] > 0.f) {
                    const float ur = u - current.baseline_times_fx * invzc;
                    if (std::fabs(ur - current.right_coordinate[i2]) >
                        radius) {
                        continue;
                    }
                }
                if (static_cast<int>(i2) >= current.descriptors.rows) {
                    continue;
                }
                const int dist = DescriptorDistance(
                    dMP, current.descriptors.row(static_cast<int>(i2)));
                if (dist < best_dist) {
                    best_dist = dist;
                    best_idx = static_cast<int>(i2);
                }
            }

            if (best_dist <= kThHigh && best_idx >= 0) {
                current.map_points[static_cast<size_t>(best_idx)] = map_point;
                ++nmatches;
                if (check_orientation_) {
                    const cv::KeyPoint kp_lf = LastFrameKeyPoint(last, i);
                    const cv::KeyPoint kp_cf = current.GetKeyPoint(best_idx);
                    float rot = kp_lf.angle - kp_cf.angle;
                    if (rot < 0.f) {
                        rot += 360.f;
                    }
                    int bin = static_cast<int>(std::round(rot * factor));
                    if (bin == kHistoLength) {
                        bin = 0;
                    }
                    if (bin >= 0 && bin < kHistoLength) {
                        rot_hist[bin].push_back(best_idx);
                    }
                }
            }
        }

        // Fisheye dual: also project into right camera (ORB Nleft path).
        if (!dual) {
            continue;
        }
        const Vec3 x3Dr = Trl * x3Dc;
        if (x3Dr.z() <= 0.0) {
            continue;
        }
        const Vec2 uv_r = ProjectFramePoint(current, x3Dr, true);
        const float ur = static_cast<float>(uv_r.x());
        const float vr = static_cast<float>(uv_r.y());
        if (!std::isfinite(ur) || !std::isfinite(vr)) {
            continue;
        }

        const int last_octave_right = LastFrameOctave(last, i);
        float radius_right = th;
        if (last_octave_right >= 0 &&
            last_octave_right <
                static_cast<int>(current.scale_factors.size())) {
            radius_right *=
                current.scale_factors[static_cast<size_t>(last_octave_right)];
        }

        std::vector<size_t> indices_r;
        if (forward) {
            indices_r = current.GetFeaturesInArea(
                ur, vr, radius_right, last_octave_right, -1, true);
        } else if (backward) {
            indices_r = current.GetFeaturesInArea(ur, vr, radius_right, 0,
                                                  last_octave_right, true);
        } else {
            indices_r = current.GetFeaturesInArea(
                ur, vr, radius_right, last_octave_right - 1,
                last_octave_right + 1, true);
        }
        if (indices_r.empty()) {
            continue;
        }

        const cv::Mat descriptor_right = map_point->GetDescriptor();
        if (descriptor_right.empty() || current.descriptors.empty()) {
            continue;
        }

        int best_dist = 256;
        int best_idx = -1;
        for (const size_t i2 : indices_r) {
            const int global = current.num_left + static_cast<int>(i2);
            if (global < 0 ||
                global >= static_cast<int>(current.map_points.size())) {
                continue;
            }
            if (current.map_points[static_cast<size_t>(global)] &&
                current.map_points[static_cast<size_t>(global)]
                        ->Observations() > 0) {
                continue;
            }
            if (global >= current.descriptors.rows) {
                continue;
            }
            const int dist = DescriptorDistance(
                descriptor_right, current.descriptors.row(global));
            if (dist < best_dist) {
                best_dist = dist;
                best_idx = static_cast<int>(i2);
            }
        }

        if (best_dist <= kThHigh && best_idx >= 0) {
            const int global = current.num_left + best_idx;
            current.map_points[static_cast<size_t>(global)] = map_point;
            ++nmatches;
            if (check_orientation_) {
                const cv::KeyPoint kp_lf = LastFrameKeyPoint(last, i);
                const cv::KeyPoint kp_cf =
                    current.keypoints_right[static_cast<size_t>(best_idx)];
                float rot = kp_lf.angle - kp_cf.angle;
                if (rot < 0.f) {
                    rot += 360.f;
                }
                int bin = static_cast<int>(std::round(rot * factor));
                if (bin == kHistoLength) {
                    bin = 0;
                }
                if (bin >= 0 && bin < kHistoLength) {
                    rot_hist[bin].push_back(global);
                }
            }
        }
    }

    if (check_orientation_) {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;
        ComputeThreeMaxima(rot_hist, kHistoLength, &ind1, &ind2, &ind3);
        for (int i = 0; i < kHistoLength; ++i) {
            if (i == ind1 || i == ind2 || i == ind3) {
                continue;
            }
            for (const int idx : rot_hist[i]) {
                if (idx >= 0 &&
                    idx < static_cast<int>(current.map_points.size())) {
                    current.map_points[static_cast<size_t>(idx)].reset();
                    --nmatches;
                }
            }
        }
    }
    return nmatches;
}

int OrbMatcher::SearchByProjection(
    tracking::Frame& frame,
    const std::vector<std::shared_ptr<MapPoint>>& map_points, float th,
    bool far_points, float th_far_points) {
    int nmatches = 0;
    const bool scale_th = std::fabs(th - 1.f) > 1e-6f;
    const bool dual = frame.HasDualCameraIndex();

    for (const auto& map_point : map_points) {
        if (!map_point || map_point->isBad()) {
            continue;
        }
        if (!map_point->track_in_view && !map_point->track_in_view_r) {
            continue;
        }
        if (far_points && map_point->track_depth > th_far_points) {
            continue;
        }

        if (map_point->track_in_view) {
            const int predicted_level = map_point->track_scale_level;
            float r = RadiusByViewingCos(map_point->track_view_cos);
            if (scale_th) {
                r *= th;
            }
            float search_radius = r;
            if (predicted_level >= 0 &&
                predicted_level <
                    static_cast<int>(frame.scale_factors.size())) {
                search_radius *= frame.scale_factors[predicted_level];
            }

            const std::vector<size_t> indices = frame.GetFeaturesInArea(
                map_point->track_proj_x, map_point->track_proj_y,
                search_radius, predicted_level - 1, predicted_level, false);
            if (!indices.empty() && !frame.descriptors.empty()) {
                const cv::Mat descriptor = map_point->GetDescriptor();
                if (!descriptor.empty()) {
                    int best_dist = 256;
                    int best_dist2 = 256;
                    int best_level = -1;
                    int best_level2 = -1;
                    int best_idx = -1;

                    for (const size_t idx : indices) {
                        if (idx >= frame.map_points.size()) {
                            continue;
                        }
                        if (frame.map_points[idx] &&
                            frame.map_points[idx]->Observations() > 0) {
                            continue;
                        }
                        // Rectified stereo epipolar check (not fisheye dual).
                        if (!dual && idx < frame.right_coordinate.size() &&
                            frame.right_coordinate[idx] > 0.f) {
                            const float er = std::fabs(
                                map_point->track_proj_xr -
                                frame.right_coordinate[idx]);
                            if (er > search_radius) {
                                continue;
                            }
                        }
                        if (static_cast<int>(idx) >= frame.descriptors.rows) {
                            continue;
                        }
                        const int dist = DescriptorDistance(
                            descriptor,
                            frame.descriptors.row(static_cast<int>(idx)));
                        const int octave = frame.GetKeyPoint(
                                                     static_cast<int>(idx))
                                               .octave;
                        if (dist < best_dist) {
                            best_dist2 = best_dist;
                            best_dist = dist;
                            best_level2 = best_level;
                            best_level = octave;
                            best_idx = static_cast<int>(idx);
                        } else if (dist < best_dist2) {
                            best_dist2 = dist;
                            best_level2 = octave;
                        }
                    }

                    if (best_dist <= kThHigh && best_idx >= 0) {
                        if (!(best_level == best_level2 &&
                              best_dist > nn_ratio_ * best_dist2)) {
                            frame.map_points[static_cast<size_t>(best_idx)] =
                                map_point;
                            ++nmatches;
                            if (dual &&
                                best_idx < static_cast<int>(
                                               frame.left_to_right_match
                                                   .size()) &&
                                frame.left_to_right_match[static_cast<size_t>(
                                    best_idx)] >= 0) {
                                const int ri =
                                    frame.num_left +
                                    frame.left_to_right_match
                                        [static_cast<size_t>(best_idx)];
                                if (ri >= 0 &&
                                    ri < static_cast<int>(
                                             frame.map_points.size())) {
                                    frame.map_points[static_cast<size_t>(ri)] =
                                        map_point;
                                    ++nmatches;
                                }
                            }
                        }
                    }
                }
            }
        }

        if (dual && map_point->track_in_view_r &&
            map_point->track_scale_level_r >= 0) {
            const int predicted_level = map_point->track_scale_level_r;
            float r = RadiusByViewingCos(map_point->track_view_cos_r);
            if (scale_th) {
                r *= th;
            }
            float search_radius = r;
            if (predicted_level >= 0 &&
                predicted_level <
                    static_cast<int>(frame.scale_factors.size())) {
                search_radius *= frame.scale_factors[predicted_level];
            }

            const std::vector<size_t> indices = frame.GetFeaturesInArea(
                map_point->track_proj_xr, map_point->track_proj_yr,
                search_radius, predicted_level - 1, predicted_level, true);
            if (indices.empty() || frame.descriptors.empty()) {
                continue;
            }
            const cv::Mat descriptor = map_point->GetDescriptor();
            if (descriptor.empty()) {
                continue;
            }

            int best_dist = 256;
            int best_dist2 = 256;
            int best_level = -1;
            int best_level2 = -1;
            int best_idx = -1;

            for (const size_t idx : indices) {
                const int global = frame.num_left + static_cast<int>(idx);
                if (global < 0 ||
                    global >= static_cast<int>(frame.map_points.size())) {
                    continue;
                }
                if (frame.map_points[static_cast<size_t>(global)] &&
                    frame.map_points[static_cast<size_t>(global)]
                            ->Observations() > 0) {
                    continue;
                }
                if (global >= frame.descriptors.rows) {
                    continue;
                }
                const int dist = DescriptorDistance(
                    descriptor, frame.descriptors.row(global));
                const int octave =
                    frame.keypoints_right[idx].octave;
                if (dist < best_dist) {
                    best_dist2 = best_dist;
                    best_dist = dist;
                    best_level2 = best_level;
                    best_level = octave;
                    best_idx = static_cast<int>(idx);
                } else if (dist < best_dist2) {
                    best_dist2 = dist;
                    best_level2 = octave;
                }
            }

            if (best_dist <= kThHigh && best_idx >= 0) {
                if (best_level == best_level2 &&
                    best_dist > nn_ratio_ * best_dist2) {
                    continue;
                }
                if (best_idx <
                        static_cast<int>(frame.right_to_left_match.size()) &&
                    frame.right_to_left_match[static_cast<size_t>(best_idx)] >=
                        0) {
                    const int li =
                        frame.right_to_left_match[static_cast<size_t>(
                            best_idx)];
                    if (li >= 0 &&
                        li < static_cast<int>(frame.map_points.size())) {
                        frame.map_points[static_cast<size_t>(li)] = map_point;
                        ++nmatches;
                    }
                }
                const int global = frame.num_left + best_idx;
                if (global >= 0 &&
                    global < static_cast<int>(frame.map_points.size())) {
                    frame.map_points[static_cast<size_t>(global)] = map_point;
                    ++nmatches;
                }
            }
        }
    }
    return nmatches;
}

int OrbMatcher::SearchByProjection(
    tracking::Frame& current, const std::shared_ptr<KeyFrame>& keyframe,
    const std::set<std::shared_ptr<MapPoint>>& already_found, float th,
    int orb_dist) {
    if (!keyframe || keyframe->isBad() || !current.has_pose()) {
        return 0;
    }

    int nmatches = 0;
    std::vector<int> rot_hist[kHistoLength];
    for (int i = 0; i < kHistoLength; ++i) {
        rot_hist[i].reserve(500);
    }
    const float factor = 1.0f / kHistoLength;

    const SE3 Tcw = current.GetPose();
    const Vec3 Ow = Tcw.inverse().translation();
    const auto map_points = keyframe->GetMapPointMatches();

    for (size_t i = 0; i < map_points.size(); ++i) {
        const auto& map_point = map_points[i];
        if (!map_point || map_point->isBad() ||
            already_found.count(map_point) > 0) {
            continue;
        }

        const Vec3 x3Dw = map_point->GetWorldPos();
        const Vec3 x3Dc = Tcw * x3Dw;
        if (x3Dc.z() <= 0.0) {
            continue;
        }

        float u = 0.f;
        float v = 0.f;
        if (current.camera) {
            const Vec2 uv = current.camera->Project(x3Dc);
            u = static_cast<float>(uv.x());
            v = static_cast<float>(uv.y());
        } else {
            const float invz = static_cast<float>(1.0 / x3Dc.z());
            u = current.fx * static_cast<float>(x3Dc.x()) * invz + current.cx;
            v = current.fy * static_cast<float>(x3Dc.y()) * invz + current.cy;
        }
        if (!std::isfinite(u) || !std::isfinite(v) || u < current.min_x() ||
            u > current.max_x() || v < current.min_y() ||
            v > current.max_y()) {
            continue;
        }

        const Vec3 PO = x3Dw - Ow;
        const float dist3d = static_cast<float>(PO.norm());
        if (dist3d < map_point->GetMinDistanceInvariance() ||
            dist3d > map_point->GetMaxDistanceInvariance()) {
            continue;
        }

        const int predicted_level = map_point->PredictScale(
            dist3d, current.scale_levels, current.log_scale_factor);
        float radius = th;
        if (predicted_level >= 0 &&
            predicted_level <
                static_cast<int>(current.scale_factors.size())) {
            radius *=
                current.scale_factors[static_cast<size_t>(predicted_level)];
        }

        const auto indices = current.GetFeaturesInArea(
            u, v, radius, predicted_level - 1, predicted_level + 1, false);
        if (indices.empty() || current.descriptors.empty()) {
            continue;
        }

        const cv::Mat dMP = map_point->GetDescriptor();
        if (dMP.empty()) {
            continue;
        }

        int best_dist = 256;
        int best_idx = -1;
        for (const size_t i2 : indices) {
            if (i2 >= current.map_points.size() || current.map_points[i2]) {
                continue;
            }
            if (static_cast<int>(i2) >= current.descriptors.rows) {
                continue;
            }
            const int dist = DescriptorDistance(
                dMP, current.descriptors.row(static_cast<int>(i2)));
            if (dist < best_dist) {
                best_dist = dist;
                best_idx = static_cast<int>(i2);
            }
        }

        if (best_dist <= orb_dist && best_idx >= 0) {
            current.map_points[static_cast<size_t>(best_idx)] = map_point;
            ++nmatches;
            if (check_orientation_) {
                const cv::KeyPoint kp_kf =
                    keyframe->GetKeyPoint(static_cast<int>(i));
                const cv::KeyPoint kp_cf = current.GetKeyPoint(best_idx);
                float rot = kp_kf.angle - kp_cf.angle;
                if (rot < 0.f) {
                    rot += 360.f;
                }
                int bin = static_cast<int>(std::round(rot * factor));
                if (bin == kHistoLength) {
                    bin = 0;
                }
                if (bin >= 0 && bin < kHistoLength) {
                    rot_hist[bin].push_back(best_idx);
                }
            }
        }
    }

    if (check_orientation_) {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;
        ComputeThreeMaxima(rot_hist, kHistoLength, &ind1, &ind2, &ind3);
        for (int i = 0; i < kHistoLength; ++i) {
            if (i == ind1 || i == ind2 || i == ind3) {
                continue;
            }
            for (const int idx : rot_hist[i]) {
                if (idx >= 0 &&
                    idx < static_cast<int>(current.map_points.size())) {
                    current.map_points[static_cast<size_t>(idx)].reset();
                    --nmatches;
                }
            }
        }
    }
    return nmatches;
}

int OrbMatcher::SearchByProjection(
    const std::shared_ptr<KeyFrame>& keyframe, const backend::Sim3& Scw,
    const std::vector<std::shared_ptr<MapPoint>>& map_points,
    std::vector<std::shared_ptr<MapPoint>>* matched, float th,
    float ratio_hamming) {
    if (!keyframe || keyframe->isBad() || matched == nullptr) {
        return 0;
    }
    const size_t n_kp = keyframe->GetKeyPoints().size();
    if (matched->size() != n_kp) {
        matched->assign(n_kp, nullptr);
    }

    // Tcw from Sim3: R, t/s (ORB-SLAM3 SearchByProjection).
    SE3 Tcw = SE3Identity();
    Tcw.linear() = Scw.rotation;
    const double inv_s = (std::abs(Scw.scale) > 1e-12) ? (1.0 / Scw.scale) : 1.0;
    Tcw.translation() = Scw.translation * inv_s;
    const Vec3 Ow = Tcw.inverse().translation();

    std::set<std::shared_ptr<MapPoint>> already_found;
    for (const auto& mp : *matched) {
        if (mp) {
            already_found.insert(mp);
        }
    }

    const cv::Mat descriptors = keyframe->GetDescriptors();
    const auto& keypoints = keyframe->GetKeyPoints();
    const int thr = static_cast<int>(kThLow * ratio_hamming);
    int nmatches = 0;

    for (const auto& map_point : map_points) {
        if (!map_point || map_point->isBad() ||
            already_found.count(map_point) > 0) {
            continue;
        }
        const Vec3 Pw = map_point->GetWorldPos();
        const Vec3 Pc = Tcw * Pw;
        if (Pc.z() <= 0.0) {
            continue;
        }
        const float invz = static_cast<float>(1.0 / Pc.z());
        const float u =
            keyframe->fx * static_cast<float>(Pc.x()) * invz + keyframe->cx;
        const float v =
            keyframe->fy * static_cast<float>(Pc.y()) * invz + keyframe->cy;
        if (!keyframe->IsInImage(u, v)) {
            continue;
        }

        const Vec3 PO = Pw - Ow;
        const float dist = static_cast<float>(PO.norm());
        if (dist < map_point->GetMinDistanceInvariance() ||
            dist > map_point->GetMaxDistanceInvariance()) {
            continue;
        }
        if (PO.dot(map_point->GetNormal()) < 0.5 * dist) {
            continue;
        }

        const int predicted_level = map_point->PredictScale(
            dist, keyframe->scale_levels, keyframe->log_scale_factor);
        float radius = th;
        if (predicted_level >= 0 &&
            predicted_level < static_cast<int>(keyframe->scale_factors.size())) {
            radius *= keyframe->scale_factors[predicted_level];
        }
        const auto indices = keyframe->GetFeaturesInArea(u, v, radius);
        if (indices.empty() || descriptors.empty()) {
            continue;
        }

        const cv::Mat dMP = map_point->GetDescriptor();
        if (dMP.empty()) {
            continue;
        }
        int best_dist = 256;
        int best_idx = -1;
        for (const size_t idx : indices) {
            if (idx >= matched->size() || (*matched)[idx]) {
                continue;
            }
            if (static_cast<int>(idx) >= descriptors.rows) {
                continue;
            }
            const int kp_level = keypoints[idx].octave;
            if (kp_level < predicted_level - 1 || kp_level > predicted_level) {
                continue;
            }
            const int d =
                DescriptorDistance(dMP, descriptors.row(static_cast<int>(idx)));
            if (d < best_dist) {
                best_dist = d;
                best_idx = static_cast<int>(idx);
            }
        }
        if (best_dist <= thr && best_idx >= 0) {
            (*matched)[static_cast<size_t>(best_idx)] = map_point;
            already_found.insert(map_point);
            ++nmatches;
        }
    }
    return nmatches;
}

int OrbMatcher::SearchByBruteForce(
    const std::shared_ptr<KeyFrame>& keyframe, tracking::Frame& frame,
    std::vector<std::shared_ptr<MapPoint>>* matches) {
    if (!keyframe || matches == nullptr || frame.descriptors.empty()) {
        return 0;
    }
    matches->assign(frame.TotalFeatures() > 0
                        ? static_cast<size_t>(frame.TotalFeatures())
                        : static_cast<size_t>(frame.num_keypoints),
                    nullptr);
    const cv::Mat kf_desc = keyframe->GetDescriptors();
    if (kf_desc.empty()) {
        return 0;
    }

    const auto map_points = keyframe->GetMapPoints();
    const int n_frame = static_cast<int>(matches->size());
    int nmatches = 0;
    for (size_t i = 0; i < map_points.size(); ++i) {
        const auto& map_point = map_points[i];
        if (!map_point || map_point->isBad()) {
            continue;
        }
        if (static_cast<int>(i) >= kf_desc.rows) {
            continue;
        }
        const cv::Mat dMP = kf_desc.row(static_cast<int>(i));
        int best_dist = 256;
        int best_dist2 = 256;
        int best_idx = -1;
        for (int j = 0; j < n_frame; ++j) {
            if ((*matches)[static_cast<size_t>(j)]) {
                continue;
            }
            if (j >= frame.descriptors.rows) {
                continue;
            }
            const int dist =
                DescriptorDistance(dMP, frame.descriptors.row(j));
            if (dist < best_dist) {
                best_dist2 = best_dist;
                best_dist = dist;
                best_idx = j;
            } else if (dist < best_dist2) {
                best_dist2 = dist;
            }
        }
        if (best_idx >= 0 && best_dist <= kThLow &&
            best_dist < nn_ratio_ * best_dist2) {
            (*matches)[static_cast<size_t>(best_idx)] = map_point;
            ++nmatches;
        }
    }
    return nmatches;
}

int OrbMatcher::SearchByBoW(
    const std::shared_ptr<KeyFrame>& keyframe, tracking::Frame& frame,
    std::vector<std::shared_ptr<MapPoint>>* matches) {
    if (!keyframe || matches == nullptr) {
        return 0;
    }
    // Prefer FBoW node-constrained matching when both sides have FeatVectors.
    if (keyframe->HasBoW() && frame.HasBoW()) {
        const int n_frame = frame.TotalFeatures();
        matches->assign(static_cast<size_t>(n_frame), nullptr);
        const auto map_points = keyframe->GetMapPoints();
        const cv::Mat kf_desc = keyframe->GetDescriptors();
        if (kf_desc.empty() || frame.descriptors.empty() || n_frame <= 0) {
            return 0;
        }

        const bool dual = frame.HasDualCameraIndex();
        int nmatches = 0;
        std::vector<int> rot_hist[kHistoLength];
        for (int i = 0; i < kHistoLength; ++i) {
            rot_hist[i].reserve(500);
        }
        const float factor = 1.0f / kHistoLength;

        auto push_orient = [&](int idx_kf, int idx_f) {
            if (!check_orientation_ || idx_f < 0) {
                return;
            }
            const cv::KeyPoint kp_kf = keyframe->GetKeyPoint(idx_kf);
            const cv::KeyPoint kp_f = frame.GetKeyPoint(idx_f);
            float rot = kp_kf.angle - kp_f.angle;
            if (rot < 0.f) {
                rot += 360.f;
            }
            int bin = static_cast<int>(std::round(rot * factor));
            if (bin == kHistoLength) {
                bin = 0;
            }
            if (bin >= 0 && bin < kHistoLength) {
                rot_hist[bin].push_back(idx_f);
            }
        };

        auto kf_it = keyframe->feat_vector().begin();
        auto f_it = frame.feat_vector().begin();
        const auto kf_end = keyframe->feat_vector().end();
        const auto f_end = frame.feat_vector().end();

        while (kf_it != kf_end && f_it != f_end) {
            if (kf_it->first == f_it->first) {
                const auto& indices_kf = kf_it->second;
                const auto& indices_f = f_it->second;
                for (const uint32_t idx_kf : indices_kf) {
                    if (idx_kf >= map_points.size()) {
                        continue;
                    }
                    const auto& map_point = map_points[idx_kf];
                    if (!map_point || map_point->isBad()) {
                        continue;
                    }
                    if (static_cast<int>(idx_kf) >= kf_desc.rows) {
                        continue;
                    }
                    const cv::Mat dKF = kf_desc.row(static_cast<int>(idx_kf));

                    int best_dist1 = 256;
                    int best_dist2 = 256;
                    int best_idx = -1;
                    int best_dist1_r = 256;
                    int best_dist2_r = 256;
                    int best_idx_r = -1;

                    for (const uint32_t idx_f : indices_f) {
                        if (static_cast<int>(idx_f) >= n_frame ||
                            (*matches)[idx_f]) {
                            continue;
                        }
                        if (static_cast<int>(idx_f) >= frame.descriptors.rows) {
                            continue;
                        }
                        const int dist = DescriptorDistance(
                            dKF, frame.descriptors.row(static_cast<int>(idx_f)));
                        if (!dual) {
                            if (dist < best_dist1) {
                                best_dist2 = best_dist1;
                                best_dist1 = dist;
                                best_idx = static_cast<int>(idx_f);
                            } else if (dist < best_dist2) {
                                best_dist2 = dist;
                            }
                            continue;
                        }
                        // Fisheye dual: separate left / right NN (ORB Nleft).
                        if (static_cast<int>(idx_f) < frame.num_left) {
                            if (dist < best_dist1) {
                                best_dist2 = best_dist1;
                                best_dist1 = dist;
                                best_idx = static_cast<int>(idx_f);
                            } else if (dist < best_dist2) {
                                best_dist2 = dist;
                            }
                        } else {
                            if (dist < best_dist1_r) {
                                best_dist2_r = best_dist1_r;
                                best_dist1_r = dist;
                                best_idx_r = static_cast<int>(idx_f);
                            } else if (dist < best_dist2_r) {
                                best_dist2_r = dist;
                            }
                        }
                    }

                    if (best_idx >= 0 && best_dist1 <= kThLow &&
                        static_cast<float>(best_dist1) <
                            nn_ratio_ * static_cast<float>(best_dist2)) {
                        (*matches)[static_cast<size_t>(best_idx)] = map_point;
                        ++nmatches;
                        push_orient(static_cast<int>(idx_kf), best_idx);
                    }
                    // ORB: right accept when bestDist1R<=TH_LOW (ratio || true).
                    if (dual && best_idx_r >= 0 && best_dist1_r <= kThLow) {
                        (*matches)[static_cast<size_t>(best_idx_r)] =
                            map_point;
                        ++nmatches;
                        push_orient(static_cast<int>(idx_kf), best_idx_r);
                    }
                }
                ++kf_it;
                ++f_it;
            } else if (kf_it->first < f_it->first) {
                kf_it = keyframe->feat_vector().lower_bound(f_it->first);
            } else {
                f_it = frame.feat_vector().lower_bound(kf_it->first);
            }
        }

        if (check_orientation_) {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;
            ComputeThreeMaxima(rot_hist, kHistoLength, &ind1, &ind2, &ind3);
            for (int i = 0; i < kHistoLength; ++i) {
                if (i == ind1 || i == ind2 || i == ind3) {
                    continue;
                }
                for (const int idx : rot_hist[i]) {
                    if (idx >= 0 && idx < n_frame) {
                        (*matches)[static_cast<size_t>(idx)].reset();
                        --nmatches;
                    }
                }
            }
        }
        return nmatches;
    }
    // Fallback when vocabulary is not loaded.
    return SearchByBruteForce(keyframe, frame, matches);
}

int OrbMatcher::SearchByBoW(
    const std::shared_ptr<KeyFrame>& keyframe1,
    const std::shared_ptr<KeyFrame>& keyframe2,
    std::vector<std::shared_ptr<MapPoint>>* matches12) {
    if (!keyframe1 || !keyframe2 || matches12 == nullptr) {
        return 0;
    }
    const auto map_points1 = keyframe1->GetMapPoints();
    const auto map_points2 = keyframe2->GetMapPoints();
    matches12->assign(map_points1.size(), nullptr);
    std::vector<bool> matched2(map_points2.size(), false);
    const cv::Mat desc1 = keyframe1->GetDescriptors();
    const cv::Mat desc2 = keyframe2->GetDescriptors();
    if (desc1.empty() || desc2.empty()) {
        return 0;
    }

    int nmatches = 0;
    auto match_pair = [&](size_t idx1, size_t idx2) {
        if (idx1 >= map_points1.size() || idx2 >= map_points2.size()) {
            return;
        }
        if (!map_points1[idx1] || map_points1[idx1]->isBad() ||
            !map_points2[idx2] || map_points2[idx2]->isBad() || matched2[idx2]) {
            return;
        }
        (*matches12)[idx1] = map_points2[idx2];
        matched2[idx2] = true;
        ++nmatches;
    };

    if (keyframe1->HasBoW() && keyframe2->HasBoW()) {
        auto it1 = keyframe1->feat_vector().begin();
        auto it2 = keyframe2->feat_vector().begin();
        const auto end1 = keyframe1->feat_vector().end();
        const auto end2 = keyframe2->feat_vector().end();
        // ORB KF↔KF BoW uses left keypoints only when dual (skip idx ≥ NLeft).
        const int n_left1 = keyframe1->HasDualCameraIndex()
                                ? keyframe1->num_left
                                : static_cast<int>(map_points1.size());
        const int n_left2 = keyframe2->HasDualCameraIndex()
                                ? keyframe2->num_left
                                : static_cast<int>(map_points2.size());
        while (it1 != end1 && it2 != end2) {
            if (it1->first == it2->first) {
                for (const uint32_t i1 : it1->second) {
                    if (static_cast<int>(i1) >= n_left1 ||
                        i1 >= map_points1.size() || !map_points1[i1] ||
                        map_points1[i1]->isBad()) {
                        continue;
                    }
                    if (static_cast<int>(i1) >= desc1.rows) {
                        continue;
                    }
                    const cv::Mat d1 = desc1.row(static_cast<int>(i1));
                    int best_dist1 = 256;
                    int best_dist2 = 256;
                    int best_idx2 = -1;
                    for (const uint32_t i2 : it2->second) {
                        if (static_cast<int>(i2) >= n_left2 ||
                            i2 >= map_points2.size() || matched2[i2] ||
                            !map_points2[i2] || map_points2[i2]->isBad()) {
                            continue;
                        }
                        if (static_cast<int>(i2) >= desc2.rows) {
                            continue;
                        }
                        const int dist = DescriptorDistance(
                            d1, desc2.row(static_cast<int>(i2)));
                        if (dist < best_dist1) {
                            best_dist2 = best_dist1;
                            best_dist1 = dist;
                            best_idx2 = static_cast<int>(i2);
                        } else if (dist < best_dist2) {
                            best_dist2 = dist;
                        }
                    }
                    if (best_idx2 >= 0 && best_dist1 <= kThLow &&
                        static_cast<float>(best_dist1) <
                            nn_ratio_ * static_cast<float>(best_dist2)) {
                        match_pair(i1, static_cast<size_t>(best_idx2));
                    }
                }
                ++it1;
                ++it2;
            } else if (it1->first < it2->first) {
                it1 = keyframe1->feat_vector().lower_bound(it2->first);
            } else {
                it2 = keyframe2->feat_vector().lower_bound(it1->first);
            }
        }
    } else {
        for (size_t i1 = 0; i1 < map_points1.size(); ++i1) {
            if (!map_points1[i1] || map_points1[i1]->isBad()) {
                continue;
            }
            int best_dist1 = 256;
            int best_dist2 = 256;
            int best_idx2 = -1;
            for (size_t i2 = 0; i2 < map_points2.size(); ++i2) {
                if (matched2[i2] || !map_points2[i2] ||
                    map_points2[i2]->isBad()) {
                    continue;
                }
                if (static_cast<int>(i1) >= desc1.rows ||
                    static_cast<int>(i2) >= desc2.rows) {
                    continue;
                }
                const int dist = DescriptorDistance(
                    desc1.row(static_cast<int>(i1)),
                    desc2.row(static_cast<int>(i2)));
                if (dist < best_dist1) {
                    best_dist2 = best_dist1;
                    best_dist1 = dist;
                    best_idx2 = static_cast<int>(i2);
                } else if (dist < best_dist2) {
                    best_dist2 = dist;
                }
            }
            if (best_idx2 >= 0 && best_dist1 <= kThLow &&
                static_cast<float>(best_dist1) <
                    nn_ratio_ * static_cast<float>(best_dist2)) {
                match_pair(i1, static_cast<size_t>(best_idx2));
            }
        }
    }
    return nmatches;
}

int OrbMatcher::Fuse(
    const std::shared_ptr<KeyFrame>& keyframe,
    const std::vector<std::shared_ptr<MapPoint>>& map_points, float th,
    bool right) {
    if (!keyframe || keyframe->isBad()) {
        return 0;
    }
    if (right && !keyframe->HasDualCameraIndex()) {
        return 0;
    }

    const SE3 Tcw = right ? keyframe->GetRightPose() : keyframe->GetPose();
    const Vec3 Ow =
        right ? keyframe->GetRightCameraCenter() : keyframe->GetCameraCenter();
    const sensor::GeometricCamera* cam =
        right ? keyframe->camera2.get() : keyframe->camera.get();
    int fused = 0;

    for (const auto& map_point : map_points) {
        if (!map_point || map_point->isBad()) {
            continue;
        }
        if (map_point->IsInKeyFrame(keyframe)) {
            continue;
        }

        const Vec3 Pw = map_point->GetWorldPos();
        const Vec3 Pc = Tcw * Pw;
        if (Pc.z() <= 0.0) {
            continue;
        }
        const float invz = static_cast<float>(1.0 / Pc.z());

        float u = 0.f;
        float v = 0.f;
        if (cam) {
            const Vec2 uv = cam->Project(Pc);
            u = static_cast<float>(uv.x());
            v = static_cast<float>(uv.y());
        } else {
            u = keyframe->fx * static_cast<float>(Pc.x()) * invz +
                keyframe->cx;
            v = keyframe->fy * static_cast<float>(Pc.y()) * invz +
                keyframe->cy;
        }
        if (!std::isfinite(u) || !std::isfinite(v) ||
            !keyframe->IsInImage(u, v)) {
            continue;
        }

        const float ur = u - keyframe->baseline_times_fx * invz;
        const Vec3 PO = Pw - Ow;
        const float dist = static_cast<float>(PO.norm());
        if (dist < map_point->GetMinDistanceInvariance() ||
            dist > map_point->GetMaxDistanceInvariance()) {
            continue;
        }
        if (PO.dot(map_point->GetNormal()) < 0.5 * dist) {
            continue;
        }

        const int predicted_level = map_point->PredictScale(
            dist, keyframe->scale_levels, keyframe->log_scale_factor);
        float radius = th;
        if (predicted_level >= 0 &&
            predicted_level <
                static_cast<int>(keyframe->scale_factors.size())) {
            radius *= keyframe->scale_factors[static_cast<size_t>(
                predicted_level)];
        }

        const auto indices =
            keyframe->GetFeaturesInArea(u, v, radius, right);
        if (indices.empty()) {
            continue;
        }

        const cv::Mat dMP = map_point->GetDescriptor();
        if (dMP.empty()) {
            continue;
        }
        const cv::Mat descriptors = keyframe->GetDescriptors();
        const auto& right_u = keyframe->GetRightCoordinates();
        int best_dist = 256;
        int best_idx = -1;
        for (size_t local : indices) {
            const cv::KeyPoint kp =
                right ? keyframe->GetKeyPointsRight()[local]
                      : keyframe->GetKeyPoints()[local];
            const int kp_level = kp.octave;
            if (kp_level < predicted_level - 1 ||
                kp_level > predicted_level) {
                continue;
            }

            // Rectified stereo chi2 (ORB); skip for fisheye dual right.
            if (!right && local < right_u.size() && right_u[local] >= 0.f) {
                const float ex = u - kp.pt.x;
                const float ey = v - kp.pt.y;
                const float er = ur - right_u[local];
                const float e2 = ex * ex + ey * ey + er * er;
                float inv_sigma2 = 1.f;
                if (kp_level >= 0 &&
                    kp_level <
                        static_cast<int>(keyframe->inverse_level_sigma2.size())) {
                    inv_sigma2 =
                        keyframe->inverse_level_sigma2[static_cast<size_t>(
                            kp_level)];
                }
                if (e2 * inv_sigma2 > 7.8f) {
                    continue;
                }
            } else {
                const float ex = u - kp.pt.x;
                const float ey = v - kp.pt.y;
                const float e2 = ex * ex + ey * ey;
                float inv_sigma2 = 1.f;
                if (kp_level >= 0 &&
                    kp_level <
                        static_cast<int>(keyframe->inverse_level_sigma2.size())) {
                    inv_sigma2 =
                        keyframe->inverse_level_sigma2[static_cast<size_t>(
                            kp_level)];
                }
                if (e2 * inv_sigma2 > 5.99f) {
                    continue;
                }
            }

            int global = static_cast<int>(local);
            if (right) {
                global = keyframe->num_left + static_cast<int>(local);
            }
            if (global < 0 || global >= descriptors.rows) {
                continue;
            }
            const int dist_desc =
                DescriptorDistance(dMP, descriptors.row(global));
            if (dist_desc < best_dist) {
                best_dist = dist_desc;
                best_idx = global;
            }
        }
        if (best_dist > kThLow || best_idx < 0) {
            continue;
        }

        auto existing = keyframe->GetMapPoint(best_idx);
        if (existing && !existing->isBad()) {
            if (existing->Observations() > map_point->Observations()) {
                map_point->Replace(existing);
            } else {
                existing->Replace(map_point);
            }
        } else {
            keyframe->AddMapPoint(map_point, best_idx);
            map_point->AddObservation(keyframe, best_idx);
        }
        ++fused;
    }
    return fused;
}

int OrbMatcher::Fuse(
    const std::shared_ptr<KeyFrame>& keyframe, const backend::Sim3& Scw,
    const std::vector<std::shared_ptr<MapPoint>>& map_points, float th,
    std::vector<std::shared_ptr<MapPoint>>* replace_points) {
    if (!keyframe || keyframe->isBad()) {
        return 0;
    }
    if (replace_points != nullptr &&
        replace_points->size() != map_points.size()) {
        replace_points->assign(map_points.size(), nullptr);
    }

    // Tcw from Sim3: R, t/s (ORB Fuse Scw).
    SE3 Tcw = SE3Identity();
    Tcw.linear() = Scw.rotation;
    const double inv_s = (std::abs(Scw.scale) > 1e-12) ? (1.0 / Scw.scale) : 1.0;
    Tcw.translation() = Scw.translation * inv_s;
    const Vec3 Ow = Tcw.inverse().translation();

    std::set<std::shared_ptr<MapPoint>> already_found;
    for (const auto& mp : keyframe->GetMapPointMatches()) {
        if (mp) {
            already_found.insert(mp);
        }
    }

    const sensor::GeometricCamera* cam = keyframe->camera.get();
    int fused = 0;

    for (size_t i = 0; i < map_points.size(); ++i) {
        const auto& map_point = map_points[i];
        if (!map_point || map_point->isBad() ||
            already_found.count(map_point) > 0) {
            continue;
        }

        const Vec3 Pw = map_point->GetWorldPos();
        const Vec3 Pc = Tcw * Pw;
        if (Pc.z() <= 0.0) {
            continue;
        }

        float u = 0.f;
        float v = 0.f;
        if (cam) {
            const Vec2 uv = cam->Project(Pc);
            u = static_cast<float>(uv.x());
            v = static_cast<float>(uv.y());
        } else {
            const float invz = static_cast<float>(1.0 / Pc.z());
            u = keyframe->fx * static_cast<float>(Pc.x()) * invz + keyframe->cx;
            v = keyframe->fy * static_cast<float>(Pc.y()) * invz + keyframe->cy;
        }
        if (!std::isfinite(u) || !std::isfinite(v) ||
            !keyframe->IsInImage(u, v)) {
            continue;
        }

        const Vec3 PO = Pw - Ow;
        const float dist = static_cast<float>(PO.norm());
        if (dist < map_point->GetMinDistanceInvariance() ||
            dist > map_point->GetMaxDistanceInvariance()) {
            continue;
        }
        if (PO.dot(map_point->GetNormal()) < 0.5 * dist) {
            continue;
        }

        const int predicted_level = map_point->PredictScale(
            dist, keyframe->scale_levels, keyframe->log_scale_factor);
        float radius = th;
        if (predicted_level >= 0 &&
            predicted_level <
                static_cast<int>(keyframe->scale_factors.size())) {
            radius *= keyframe->scale_factors[static_cast<size_t>(
                predicted_level)];
        }

        const auto indices = keyframe->GetFeaturesInArea(u, v, radius, false);
        if (indices.empty()) {
            continue;
        }

        const cv::Mat dMP = map_point->GetDescriptor();
        if (dMP.empty()) {
            continue;
        }
        const cv::Mat descriptors = keyframe->GetDescriptors();
        int best_dist = INT_MAX;
        int best_idx = -1;
        for (const size_t idx : indices) {
            const cv::KeyPoint kp = keyframe->GetKeyPoint(static_cast<int>(idx));
            if (kp.octave < predicted_level - 1 ||
                kp.octave > predicted_level) {
                continue;
            }
            if (static_cast<int>(idx) >= descriptors.rows) {
                continue;
            }
            const int dist_desc =
                DescriptorDistance(dMP, descriptors.row(static_cast<int>(idx)));
            if (dist_desc < best_dist) {
                best_dist = dist_desc;
                best_idx = static_cast<int>(idx);
            }
        }
        if (best_dist > kThLow || best_idx < 0) {
            continue;
        }

        auto existing = keyframe->GetMapPoint(best_idx);
        if (existing && !existing->isBad()) {
            if (replace_points != nullptr) {
                (*replace_points)[i] = existing;
            } else if (existing->Observations() > map_point->Observations()) {
                map_point->Replace(existing);
            } else {
                existing->Replace(map_point);
            }
        } else {
            keyframe->AddMapPoint(map_point, best_idx);
            map_point->AddObservation(keyframe, best_idx);
        }
        ++fused;
    }
    return fused;
}

namespace {

int IndexInKeyFrame(const std::shared_ptr<MapPoint>& map_point,
                    const std::shared_ptr<KeyFrame>& keyframe) {
    if (!map_point || !keyframe) {
        return -1;
    }
    for (const auto& [weak_kf, idx] : map_point->GetObservations()) {
        auto kf = weak_kf.lock();
        if (kf && kf.get() == keyframe.get()) {
            return idx;
        }
    }
    return -1;
}

}  // namespace

int OrbMatcher::SearchBySim3(
    const std::shared_ptr<KeyFrame>& keyframe1,
    const std::shared_ptr<KeyFrame>& keyframe2,
    std::vector<std::shared_ptr<MapPoint>>* matches12, const backend::Sim3& S12,
    float th) {
    if (!keyframe1 || !keyframe2 || matches12 == nullptr) {
        return 0;
    }

    const auto map_points1 = keyframe1->GetMapPointMatches();
    const auto map_points2 = keyframe2->GetMapPointMatches();
    const int n1 = static_cast<int>(map_points1.size());
    const int n2 = static_cast<int>(map_points2.size());
    if (static_cast<int>(matches12->size()) != n1) {
        matches12->assign(static_cast<size_t>(n1), nullptr);
    }

    const SE3 T1w = keyframe1->GetPose();
    const SE3 T2w = keyframe2->GetPose();
    const backend::Sim3 S21 = S12.Inverse();

    std::vector<bool> already1(static_cast<size_t>(n1), false);
    std::vector<bool> already2(static_cast<size_t>(n2), false);
    for (int i = 0; i < n1; ++i) {
        if (!(*matches12)[static_cast<size_t>(i)]) {
            continue;
        }
        already1[static_cast<size_t>(i)] = true;
        const int idx2 = IndexInKeyFrame((*matches12)[static_cast<size_t>(i)],
                                         keyframe2);
        if (idx2 >= 0 && idx2 < n2) {
            already2[static_cast<size_t>(idx2)] = true;
        }
    }

    std::vector<int> match1(static_cast<size_t>(n1), -1);
    std::vector<int> match2(static_cast<size_t>(n2), -1);

    auto project_search =
        [&](const std::shared_ptr<KeyFrame>& kf_src,
            const std::shared_ptr<KeyFrame>& kf_dst, const SE3& Tw_src,
            const backend::Sim3& S_dst_from_src, int i_src,
            const std::vector<std::shared_ptr<MapPoint>>& mps_src,
            const std::vector<bool>& already_src, std::vector<int>* out_match) {
            if (i_src < 0 || i_src >= static_cast<int>(mps_src.size()) ||
                already_src[static_cast<size_t>(i_src)] ||
                !mps_src[static_cast<size_t>(i_src)] ||
                mps_src[static_cast<size_t>(i_src)]->isBad()) {
                return;
            }
            const auto& map_point = mps_src[static_cast<size_t>(i_src)];
            const Vec3 p3Dw = map_point->GetWorldPos();
            const Vec3 p3Dc_src = Tw_src * p3Dw;
            // Apply Sim3 to camera-frame point: s*R*x + t
            Vec3 p3Dc_dst =
                S_dst_from_src.scale * (S_dst_from_src.rotation * p3Dc_src) +
                S_dst_from_src.translation;
            if (p3Dc_dst.z() <= 0.0) {
                return;
            }
            float u = 0.f;
            float v = 0.f;
            if (kf_dst->camera) {
                const Vec2 uv = kf_dst->camera->Project(p3Dc_dst);
                u = static_cast<float>(uv.x());
                v = static_cast<float>(uv.y());
            } else {
                const float invz = static_cast<float>(1.0 / p3Dc_dst.z());
                u = kf_dst->fx * static_cast<float>(p3Dc_dst.x()) * invz +
                    kf_dst->cx;
                v = kf_dst->fy * static_cast<float>(p3Dc_dst.y()) * invz +
                    kf_dst->cy;
            }
            if (!std::isfinite(u) || !std::isfinite(v) ||
                !kf_dst->IsInImage(u, v)) {
                return;
            }
            const float dist3d = static_cast<float>(p3Dc_dst.norm());
            if (dist3d < map_point->GetMinDistanceInvariance() ||
                dist3d > map_point->GetMaxDistanceInvariance()) {
                return;
            }
            const int predicted_level = map_point->PredictScale(
                dist3d, kf_dst->scale_levels, kf_dst->log_scale_factor);
            float radius = th;
            if (predicted_level >= 0 &&
                predicted_level <
                    static_cast<int>(kf_dst->scale_factors.size())) {
                radius *= kf_dst->scale_factors[static_cast<size_t>(
                    predicted_level)];
            }
            const auto indices =
                kf_dst->GetFeaturesInArea(u, v, radius, false);
            if (indices.empty()) {
                return;
            }
            const cv::Mat dMP = map_point->GetDescriptor();
            if (dMP.empty()) {
                return;
            }
            const cv::Mat descriptors = kf_dst->GetDescriptors();
            int best_dist = INT_MAX;
            int best_idx = -1;
            for (const size_t idx : indices) {
                const cv::KeyPoint kp =
                    kf_dst->GetKeyPoint(static_cast<int>(idx));
                if (kp.octave < predicted_level - 1 ||
                    kp.octave > predicted_level) {
                    continue;
                }
                if (static_cast<int>(idx) >= descriptors.rows) {
                    continue;
                }
                const int dist = DescriptorDistance(
                    dMP, descriptors.row(static_cast<int>(idx)));
                if (dist < best_dist) {
                    best_dist = dist;
                    best_idx = static_cast<int>(idx);
                }
            }
            if (best_dist <= kThHigh && best_idx >= 0) {
                (*out_match)[static_cast<size_t>(i_src)] = best_idx;
            }
            (void)kf_src;
        };

    for (int i1 = 0; i1 < n1; ++i1) {
        project_search(keyframe1, keyframe2, T1w, S21, i1, map_points1,
                       already1, &match1);
    }
    for (int i2 = 0; i2 < n2; ++i2) {
        project_search(keyframe2, keyframe1, T2w, S12, i2, map_points2,
                       already2, &match2);
    }

    // Check agreement (ORB SearchBySim3).
    int nmatches = 0;
    for (int i1 = 0; i1 < n1; ++i1) {
        const int i2 = match1[static_cast<size_t>(i1)];
        if (i2 < 0) {
            continue;
        }
        if (match2[static_cast<size_t>(i2)] == i1) {
            (*matches12)[static_cast<size_t>(i1)] =
                map_points2[static_cast<size_t>(i2)];
            ++nmatches;
        }
    }
    return nmatches;
}

int OrbMatcher::SearchForTriangulation(
    const std::shared_ptr<KeyFrame>& keyframe1,
    const std::shared_ptr<KeyFrame>& keyframe2,
    std::vector<std::pair<size_t, size_t>>* matched_pairs, bool only_stereo) {
    if (!keyframe1 || !keyframe2 || matched_pairs == nullptr) {
        return 0;
    }
    matched_pairs->clear();

    const cv::Mat desc1 = keyframe1->GetDescriptors();
    const cv::Mat desc2 = keyframe2->GetDescriptors();
    if (desc1.empty() || desc2.empty()) {
        return 0;
    }

    const int n1 = keyframe1->TotalFeatures();
    const int n2 = keyframe2->TotalFeatures();
    if (n1 <= 0 || n2 <= 0 || desc1.rows < n1 || desc2.rows < n2) {
        return 0;
    }

    const auto map_points1 = keyframe1->GetMapPoints();
    const auto map_points2 = keyframe2->GetMapPoints();

    const SE3 T1w = keyframe1->GetPose();
    const SE3 T2w = keyframe2->GetPose();
    const SE3 Tw2 = keyframe2->GetPoseInverse();
    const Vec3 C2 = T2w * keyframe1->GetCameraCenter();
    Vec2 ep(0, 0);
    if (keyframe2->camera) {
        ep = keyframe2->camera->Project(C2);
    } else if (C2.z() > 1e-8) {
        const float invz = static_cast<float>(1.0 / C2.z());
        ep = Vec2(keyframe2->fx * C2.x() * invz + keyframe2->cx,
                  keyframe2->fy * C2.y() * invz + keyframe2->cy);
    }

    const bool dual =
        keyframe1->HasDualCameraIndex() && keyframe2->HasDualCameraIndex() &&
        keyframe1->camera2 && keyframe2->camera2;

    SE3 Tll = T1w * Tw2;
    SE3 Tlr;
    SE3 Trl;
    SE3 Trr;
    if (dual) {
        const SE3 Tr1w = keyframe1->GetRightPose();
        const SE3 Twr2 = keyframe2->GetRightPoseInverse();
        Tll = T1w * Tw2;
        Tlr = T1w * Twr2;
        Trl = Tr1w * Tw2;
        Trr = Tr1w * Twr2;
    }
    const Mat33 Rll = Tll.rotation();
    const Mat33 Rlr =
        dual ? Mat33(Tlr.rotation()) : Mat33(Mat33::Identity());
    const Mat33 Rrl =
        dual ? Mat33(Trl.rotation()) : Mat33(Mat33::Identity());
    const Mat33 Rrr =
        dual ? Mat33(Trr.rotation()) : Mat33(Mat33::Identity());
    const Vec3 tll = Tll.translation();
    const Vec3 tlr = dual ? Vec3(Tlr.translation()) : Vec3(Vec3::Zero());
    const Vec3 trl = dual ? Vec3(Trl.translation()) : Vec3(Vec3::Zero());
    const Vec3 trr = dual ? Vec3(Trr.translation()) : Vec3(Vec3::Zero());

    const auto& right_u1 = keyframe1->GetRightCoordinates();
    const auto& right_u2 = keyframe2->GetRightCoordinates();

    std::vector<bool> matched2(static_cast<size_t>(n2), false);
    std::vector<int> matches12(static_cast<size_t>(n1), -1);
    int nmatches = 0;

    std::vector<int> rot_hist[kHistoLength];
    for (int i = 0; i < kHistoLength; ++i) {
        rot_hist[i].reserve(500);
    }
    const float factor = 1.0f / kHistoLength;

    auto sigma2 = [](const std::shared_ptr<KeyFrame>& kf, int octave) -> float {
        if (octave >= 0 &&
            octave < static_cast<int>(kf->level_sigma2.size())) {
            return kf->level_sigma2[static_cast<size_t>(octave)];
        }
        return 1.f;
    };

    auto is_stereo = [](const std::shared_ptr<KeyFrame>& kf,
                        const std::vector<float>& right_u, int idx) {
        return !kf->camera2 && idx >= 0 &&
               static_cast<size_t>(idx) < right_u.size() &&
               right_u[static_cast<size_t>(idx)] >= 0.f;
    };

    auto find_best = [&](int idx1, const std::vector<uint32_t>& cands) {
        if (idx1 < 0 || idx1 >= n1) {
            return;
        }
        if (static_cast<size_t>(idx1) < map_points1.size() &&
            map_points1[static_cast<size_t>(idx1)]) {
            return;
        }
        const bool stereo1 = is_stereo(keyframe1, right_u1, idx1);
        if (only_stereo && !stereo1) {
            return;
        }

        int best_dist = kThLow;
        int best_idx2 = -1;
        cv::KeyPoint best_kp2;
        const cv::KeyPoint kp1 = keyframe1->GetKeyPoint(idx1);
        const bool right1 =
            keyframe1->HasDualCameraIndex() && idx1 >= keyframe1->num_left;

        for (const uint32_t i2u : cands) {
            const int idx2 = static_cast<int>(i2u);
            if (idx2 < 0 || idx2 >= n2 || matched2[static_cast<size_t>(idx2)] ||
                (static_cast<size_t>(idx2) < map_points2.size() &&
                 map_points2[static_cast<size_t>(idx2)])) {
                continue;
            }
            const bool stereo2 = is_stereo(keyframe2, right_u2, idx2);
            if (only_stereo && !stereo2) {
                continue;
            }
            const int dist =
                DescriptorDistance(desc1.row(idx1), desc2.row(idx2));
            if (dist > best_dist) {
                continue;
            }
            const cv::KeyPoint kp2 = keyframe2->GetKeyPoint(idx2);
            const bool right2 =
                keyframe2->HasDualCameraIndex() && idx2 >= keyframe2->num_left;

            if (!stereo1 && !stereo2 && !keyframe1->camera2) {
                const float dx = static_cast<float>(ep.x()) - kp2.pt.x;
                const float dy = static_cast<float>(ep.y()) - kp2.pt.y;
                float scale = 1.f;
                if (kp2.octave >= 0 &&
                    kp2.octave <
                        static_cast<int>(keyframe2->scale_factors.size())) {
                    scale = keyframe2
                                ->scale_factors[static_cast<size_t>(kp2.octave)];
                }
                if (dx * dx + dy * dy < 100.f * scale) {
                    continue;
                }
            }

            Mat33 R = Rll;
            Vec3 t = tll;
            const sensor::GeometricCamera* cam1 = keyframe1->camera.get();
            const sensor::GeometricCamera* cam2 = keyframe2->camera.get();
            if (dual) {
                if (right1 && right2) {
                    R = Rrr;
                    t = trr;
                    cam1 = keyframe1->camera2.get();
                    cam2 = keyframe2->camera2.get();
                } else if (right1 && !right2) {
                    R = Rrl;
                    t = trl;
                    cam1 = keyframe1->camera2.get();
                    cam2 = keyframe2->camera.get();
                } else if (!right1 && right2) {
                    R = Rlr;
                    t = tlr;
                    cam1 = keyframe1->camera.get();
                    cam2 = keyframe2->camera2.get();
                }
            }

            if (!GeometricTools::EpipolarConstrain(
                    cam1, cam2, kp1, kp2, R, t, sigma2(keyframe1, kp1.octave),
                    sigma2(keyframe2, kp2.octave))) {
                continue;
            }
            best_dist = dist;
            best_idx2 = idx2;
            best_kp2 = kp2;
        }

        if (best_idx2 < 0) {
            return;
        }
        matches12[static_cast<size_t>(idx1)] = best_idx2;
        matched2[static_cast<size_t>(best_idx2)] = true;
        ++nmatches;
        if (check_orientation_) {
            float rot = kp1.angle - best_kp2.angle;
            if (rot < 0.f) {
                rot += 360.f;
            }
            int bin = static_cast<int>(std::round(rot * factor));
            if (bin == kHistoLength) {
                bin = 0;
            }
            if (bin >= 0 && bin < kHistoLength) {
                rot_hist[bin].push_back(idx1);
            }
        }
    };

    if (keyframe1->HasBoW() && keyframe2->HasBoW()) {
        auto it1 = keyframe1->feat_vector().begin();
        auto it2 = keyframe2->feat_vector().begin();
        const auto end1 = keyframe1->feat_vector().end();
        const auto end2 = keyframe2->feat_vector().end();
        while (it1 != end1 && it2 != end2) {
            if (it1->first == it2->first) {
                for (const uint32_t i1 : it1->second) {
                    find_best(static_cast<int>(i1), it2->second);
                }
                ++it1;
                ++it2;
            } else if (it1->first < it2->first) {
                it1 = keyframe1->feat_vector().lower_bound(it2->first);
            } else {
                it2 = keyframe2->feat_vector().lower_bound(it1->first);
            }
        }
    } else {
        std::vector<uint32_t> all2(static_cast<size_t>(n2));
        std::iota(all2.begin(), all2.end(), 0u);
        for (int i1 = 0; i1 < n1; ++i1) {
            find_best(i1, all2);
        }
    }

    if (check_orientation_) {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;
        ComputeThreeMaxima(rot_hist, kHistoLength, &ind1, &ind2, &ind3);
        for (int i = 0; i < kHistoLength; ++i) {
            if (i == ind1 || i == ind2 || i == ind3) {
                continue;
            }
            for (const int idx1 : rot_hist[i]) {
                if (idx1 >= 0 && idx1 < n1 &&
                    matches12[static_cast<size_t>(idx1)] >= 0) {
                    matches12[static_cast<size_t>(idx1)] = -1;
                    --nmatches;
                }
            }
        }
    }

    matched_pairs->reserve(static_cast<size_t>(std::max(0, nmatches)));
    for (int i = 0; i < n1; ++i) {
        if (matches12[static_cast<size_t>(i)] < 0) {
            continue;
        }
        matched_pairs->emplace_back(
            static_cast<size_t>(i),
            static_cast<size_t>(matches12[static_cast<size_t>(i)]));
    }
    return static_cast<int>(matched_pairs->size());
}

int OrbMatcher::SearchForInitialization(tracking::Frame& f1,
                                        tracking::Frame& f2,
                                        std::vector<cv::Point2f>* prev_matched,
                                        std::vector<int>* matches12,
                                        int window_size) {
    if (!prev_matched || !matches12) {
        return 0;
    }
    matches12->assign(static_cast<size_t>(f1.num_keypoints), -1);
    if (prev_matched->size() != static_cast<size_t>(f1.num_keypoints)) {
        prev_matched->resize(static_cast<size_t>(f1.num_keypoints));
        for (int i = 0; i < f1.num_keypoints; ++i) {
            (*prev_matched)[static_cast<size_t>(i)] =
                f1.keypoints_undistorted[static_cast<size_t>(i)].pt;
        }
    }

    std::vector<int> rot_hist[kHistoLength];
    for (int i = 0; i < kHistoLength; ++i) {
        rot_hist[i].reserve(500);
    }
    const float factor = 1.f / kHistoLength;
    std::vector<int> matched_distance(static_cast<size_t>(f2.num_keypoints),
                                      std::numeric_limits<int>::max());
    std::vector<int> matches21(static_cast<size_t>(f2.num_keypoints), -1);

    int nmatches = 0;
    for (int i1 = 0; i1 < f1.num_keypoints; ++i1) {
        const cv::KeyPoint& kp1 = f1.keypoints_undistorted[static_cast<size_t>(i1)];
        if (kp1.octave > 0) {
            continue;
        }
        const auto& indices = f2.GetFeaturesInArea(
            (*prev_matched)[static_cast<size_t>(i1)].x,
            (*prev_matched)[static_cast<size_t>(i1)].y,
            static_cast<float>(window_size), 0, 0);
        if (indices.empty()) {
            continue;
        }
        const cv::Mat d1 = f1.descriptors.row(i1);
        int best_dist = std::numeric_limits<int>::max();
        int best_dist2 = std::numeric_limits<int>::max();
        int best_idx2 = -1;
        for (const size_t i2 : indices) {
            const int dist =
                DescriptorDistance(d1, f2.descriptors.row(static_cast<int>(i2)));
            if (matched_distance[i2] <= dist) {
                continue;
            }
            if (dist < best_dist) {
                best_dist2 = best_dist;
                best_dist = dist;
                best_idx2 = static_cast<int>(i2);
            } else if (dist < best_dist2) {
                best_dist2 = dist;
            }
        }
        if (best_idx2 < 0 || best_dist > kThLow) {
            continue;
        }
        if (static_cast<float>(best_dist) >=
            nn_ratio_ * static_cast<float>(best_dist2)) {
            continue;
        }
        (*matches12)[static_cast<size_t>(i1)] = best_idx2;
        matched_distance[static_cast<size_t>(best_idx2)] = best_dist;
        matches21[static_cast<size_t>(best_idx2)] = i1;
        ++nmatches;

        if (check_orientation_) {
            float rot = kp1.angle -
                        f2.keypoints_undistorted[static_cast<size_t>(best_idx2)]
                            .angle;
            if (rot < 0.f) {
                rot += 360.f;
            }
            int bin = static_cast<int>(std::round(rot * factor));
            if (bin == kHistoLength) {
                bin = 0;
            }
            if (bin >= 0 && bin < kHistoLength) {
                rot_hist[bin].push_back(i1);
            }
        }
    }

    if (check_orientation_) {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;
        ComputeThreeMaxima(rot_hist, kHistoLength, &ind1, &ind2, &ind3);
        for (int i = 0; i < kHistoLength; ++i) {
            if (i == ind1 || i == ind2 || i == ind3) {
                continue;
            }
            for (const int idx : rot_hist[i]) {
                if ((*matches12)[static_cast<size_t>(idx)] >= 0) {
                    (*matches12)[static_cast<size_t>(idx)] = -1;
                    --nmatches;
                }
            }
        }
    }

    for (int i1 = 0; i1 < f1.num_keypoints; ++i1) {
        if ((*matches12)[static_cast<size_t>(i1)] >= 0) {
            (*prev_matched)[static_cast<size_t>(i1)] =
                f2.keypoints_undistorted[static_cast<size_t>(
                                             (*matches12)[static_cast<size_t>(
                                                 i1)])]
                    .pt;
        }
    }
    return nmatches;
}

}  // namespace feature
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
