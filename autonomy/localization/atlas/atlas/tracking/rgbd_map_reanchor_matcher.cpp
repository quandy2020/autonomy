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

#include "atlas/tracking/rgbd_map_reanchor_matcher.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <numeric>
#include <random>
#include <set>
#include <tuple>
#include <utility>

#include <opencv2/calib3d.hpp>

#include "atlas/match/orb_matcher.hpp"
#include "atlas/tracking/recovery_policy.hpp"

namespace autonomy::localization::atlas {
namespace {

constexpr int kGridColumns = 4;
constexpr int kGridRows = 4;

bool IsFiniteCandidate(const internal::RgbdMapReanchorCandidate& candidate) {
    return candidate.feature_index >= 0 &&
           candidate.descriptor_distance >= 0 &&
           std::isfinite(candidate.squared_reprojection_error_pixels) &&
           candidate.squared_reprojection_error_pixels >= 0.0 &&
           std::isfinite(candidate.squared_xyz_error_meters) &&
           candidate.squared_xyz_error_meters >= 0.0;
}

bool CandidateRankLess(const internal::RgbdMapReanchorCandidate* left,
                       const internal::RgbdMapReanchorCandidate* right) {
    return std::tie(left->descriptor_distance,
                    left->squared_reprojection_error_pixels,
                    left->squared_xyz_error_meters, left->map_point_id,
                    left->feature_index, left->map_point_index) <
           std::tie(right->descriptor_distance,
                    right->squared_reprojection_error_pixels,
                    right->squared_xyz_error_meters, right->map_point_id,
                    right->feature_index, right->map_point_index);
}

bool GeometryCandidateRankLess(
    const internal::RgbdMapReanchorCandidate* left,
    const internal::RgbdMapReanchorCandidate* right) {
    return std::tie(left->squared_xyz_error_meters,
                    left->squared_reprojection_error_pixels,
                    left->descriptor_distance, left->map_point_id,
                    left->feature_index, left->map_point_index) <
           std::tie(right->squared_xyz_error_meters,
                    right->squared_reprojection_error_pixels,
                    right->descriptor_distance, right->map_point_id,
                    right->feature_index, right->map_point_index);
}

bool PassesDescriptorRatio(
    const std::vector<const internal::RgbdMapReanchorCandidate*>& ranked,
    double descriptor_ratio, int maximum_descriptor_distance) {
    if (ranked.empty() ||
        ranked.front()->descriptor_distance > maximum_descriptor_distance) {
        return false;
    }
    if (ranked.size() == 1U) {
        return true;
    }
    return static_cast<double>(ranked[0]->descriptor_distance) <
           descriptor_ratio * ranked[1]->descriptor_distance;
}

bool HasValidDescriptorBank(const cv::Mat& descriptor_bank,
                            int observation_descriptor_rows) {
    return descriptor_bank.rows > 0 &&
           descriptor_bank.rows <=
               internal::kRgbdMapReanchorDescriptorRowsPerMapPoint &&
           descriptor_bank.cols == 32 && descriptor_bank.type() == CV_8U &&
           observation_descriptor_rows >= 0 &&
           observation_descriptor_rows <= descriptor_bank.rows;
}

bool HasValidGeometryFrameStorage(
    const RgbdMapReanchorMatcher::FrameView& frame) {
    if (!frame.keypoints || !frame.depths || frame.feature_count <= 0 ||
        frame.keypoints->size() <
            static_cast<std::size_t>(frame.feature_count) ||
        frame.depths->size() < static_cast<std::size_t>(frame.feature_count)) {
        return false;
    }
    return frame.intrinsics.array().isFinite().all() &&
           frame.intrinsics(0, 0) > 0.0F &&
           frame.intrinsics(1, 1) > 0.0F &&
           std::isfinite(frame.minimum_x) &&
           std::isfinite(frame.maximum_x) &&
           std::isfinite(frame.minimum_y) &&
           std::isfinite(frame.maximum_y) &&
           frame.maximum_x > frame.minimum_x &&
           frame.maximum_y > frame.minimum_y;
}

bool HasValidFrameStorage(const RgbdMapReanchorMatcher::FrameView& frame) {
    return HasValidGeometryFrameStorage(frame) && frame.descriptors &&
           !frame.descriptors->empty() &&
           frame.descriptors->type() == CV_8U &&
           frame.descriptors->cols == 32 &&
           frame.descriptors->rows >= frame.feature_count;
}

int CountOccupiedGridCells(
    const RgbdMapReanchorMatcher::FrameView& frame,
    const std::vector<internal::RgbdMapReanchorCandidate>& matches) {
    std::array<bool, kGridColumns * kGridRows> occupied_grid_cells{};
    const double image_width = frame.maximum_x - frame.minimum_x;
    const double image_height = frame.maximum_y - frame.minimum_y;
    if (!frame.keypoints || image_width <= 0.0 || image_height <= 0.0) {
        return 0;
    }
    for (const internal::RgbdMapReanchorCandidate& match : matches) {
        if (match.feature_index < 0 ||
            match.feature_index >= frame.feature_count ||
            static_cast<std::size_t>(match.feature_index) >=
                frame.keypoints->size()) {
            continue;
        }
        const cv::Point2f& point = (*frame.keypoints)[match.feature_index].pt;
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            point.x < frame.minimum_x || point.x > frame.maximum_x ||
            point.y < frame.minimum_y || point.y > frame.maximum_y) {
            continue;
        }
        const int column = std::clamp(
            static_cast<int>((point.x - frame.minimum_x) / image_width *
                             kGridColumns),
            0, kGridColumns - 1);
        const int row = std::clamp(
            static_cast<int>((point.y - frame.minimum_y) / image_height *
                             kGridRows),
            0, kGridRows - 1);
        occupied_grid_cells[row * kGridColumns + column] = true;
    }
    return static_cast<int>(std::count(occupied_grid_cells.begin(),
                                       occupied_grid_cells.end(), true));
}

struct ValidMapPoint {
    std::size_t view_index = 0;
    MapPoint* map_point = nullptr;
    Eigen::Vector3f world_point = Eigen::Vector3f::Zero();
    cv::Mat descriptor_bank;
    int observation_descriptor_rows = 0;
};

struct PnpCorrespondence {
    cv::Point3f world_point;
    cv::Point2f image_point;
    std::uint64_t map_point_id = 0;
    int feature_index = -1;
};

struct PnpEstimate {
    bool success = false;
    int inlier_count = 0;
    autonomy::common::math::SE3f camera_from_world;
};

cv::Mat CameraMatrix(const Eigen::Matrix3f& intrinsics) {
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            camera_matrix.at<double>(row, column) =
                intrinsics(row, column);
        }
    }
    return camera_matrix;
}

bool NormalizePoseVector(const cv::Mat& input, cv::Mat* output) {
    if (!output || input.empty() || input.total() != 3U ||
        input.channels() != 1) {
        return false;
    }
    try {
        input.reshape(1, 3).convertTo(*output, CV_64F);
    } catch (const cv::Exception&) {
        return false;
    }
    return output->rows == 3 && output->cols == 1 &&
           cv::checkRange(*output, true);
}

std::vector<int> ScorePnpPose(
    const std::vector<PnpCorrespondence>& correspondences,
    const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation,
    const Eigen::Matrix3f& intrinsics, double* squared_error) {
    std::vector<int> inliers;
    inliers.reserve(correspondences.size());
    *squared_error = 0.0;
    for (int index = 0; index < static_cast<int>(correspondences.size());
         ++index) {
        const PnpCorrespondence& correspondence = correspondences[index];
        const Eigen::Vector3d world_point(correspondence.world_point.x,
                                          correspondence.world_point.y,
                                          correspondence.world_point.z);
        const Eigen::Vector3d camera_point =
            rotation * world_point + translation;
        if (!camera_point.allFinite() || camera_point.z() <= 0.0) {
            continue;
        }
        const double projected_x =
            intrinsics(0, 0) * camera_point.x() / camera_point.z() +
            intrinsics(0, 2);
        const double projected_y =
            intrinsics(1, 1) * camera_point.y() / camera_point.z() +
            intrinsics(1, 2);
        const double residual_x =
            projected_x - correspondence.image_point.x;
        const double residual_y =
            projected_y - correspondence.image_point.y;
        const double candidate_squared_error =
            residual_x * residual_x + residual_y * residual_y;
        if (!internal::IsWithinRgbdMapReanchorReprojectionGate(
                candidate_squared_error)) {
            continue;
        }
        inliers.push_back(index);
        *squared_error += candidate_squared_error;
    }
    return inliers;
}

bool OpenCvPoseToEigen(const cv::Mat& rotation_vector,
                       const cv::Mat& translation_vector,
                       Eigen::Matrix3d* rotation,
                       Eigen::Vector3d* translation) {
    cv::Mat normalized_rotation_vector;
    cv::Mat normalized_translation_vector;
    if (!rotation || !translation ||
        !NormalizePoseVector(rotation_vector, &normalized_rotation_vector) ||
        !NormalizePoseVector(translation_vector,
                             &normalized_translation_vector)) {
        return false;
    }
    cv::Mat rotation_matrix;
    try {
        cv::Rodrigues(normalized_rotation_vector, rotation_matrix);
        rotation_matrix.convertTo(rotation_matrix, CV_64F);
    } catch (const cv::Exception&) {
        return false;
    }
    if (rotation_matrix.rows != 3 || rotation_matrix.cols != 3 ||
        !cv::checkRange(rotation_matrix, true)) {
        return false;
    }
    for (int row = 0; row < 3; ++row) {
        (*translation)(row) = normalized_translation_vector.at<double>(row);
        for (int column = 0; column < 3; ++column) {
            (*rotation)(row, column) =
                rotation_matrix.at<double>(row, column);
        }
    }
    return rotation->allFinite() && translation->allFinite() &&
           std::abs(rotation->determinant() - 1.0) < 1.0e-4;
}

PnpEstimate EstimateDeterministicMapPose(
    const RgbdMapReanchorMatcher::FrameView& frame,
    const std::vector<RgbdMapReanchorMatcher::MapPointView>& map_points,
    const std::vector<MapPoint*>& associations) {
    PnpEstimate result;
    std::map<MapPoint*, int> pointer_counts;
    for (const RgbdMapReanchorMatcher::MapPointView& map_point : map_points) {
        if (map_point.map_point && map_point.valid_for_expected_map &&
            map_point.world_point.allFinite()) {
            ++pointer_counts[map_point.map_point];
        }
    }
    std::map<MapPoint*, const RgbdMapReanchorMatcher::MapPointView*>
        view_by_pointer;
    for (const RgbdMapReanchorMatcher::MapPointView& map_point : map_points) {
        if (map_point.map_point && map_point.valid_for_expected_map &&
            map_point.world_point.allFinite() &&
            pointer_counts[map_point.map_point] == 1) {
            view_by_pointer.emplace(map_point.map_point, &map_point);
        }
    }

    std::vector<PnpCorrespondence> correspondences;
    correspondences.reserve(associations.size());
    for (int feature_index = 0;
         feature_index < static_cast<int>(associations.size());
         ++feature_index) {
        MapPoint* const map_point = associations[feature_index];
        const auto view = view_by_pointer.find(map_point);
        if (!map_point || view == view_by_pointer.end() ||
            feature_index >= frame.feature_count) {
            continue;
        }
        const Eigen::Vector3f& world_point = view->second->world_point;
        const cv::Point2f& image_point =
            (*frame.keypoints)[feature_index].pt;
        if (!world_point.allFinite() || !std::isfinite(image_point.x) ||
            !std::isfinite(image_point.y)) {
            continue;
        }
        correspondences.push_back(
            {cv::Point3f(world_point.x(), world_point.y(), world_point.z()),
             image_point, view->second->map_point_id, feature_index});
    }
    if (correspondences.size() < 4U) {
        return result;
    }

    std::uint32_t seed = 0x4d415052U;
    for (const PnpCorrespondence& correspondence : correspondences) {
        seed ^= static_cast<std::uint32_t>(correspondence.map_point_id);
        seed = seed * 1664525U + 1013904223U;
        seed ^= static_cast<std::uint32_t>(correspondence.map_point_id >> 32U);
        seed ^= static_cast<std::uint32_t>(correspondence.feature_index) *
                0x9e3779b9U;
    }
    std::mt19937 random(seed);
    std::uniform_int_distribution<int> distribution(
        0, static_cast<int>(correspondences.size()) - 1);
    const cv::Mat camera_matrix = CameraMatrix(frame.intrinsics);
    std::vector<int> best_inliers;
    double best_squared_error = std::numeric_limits<double>::infinity();
    cv::Mat best_rotation_vector;
    cv::Mat best_translation_vector;
    constexpr int kRansacIterations = 100;
    for (int iteration = 0; iteration < kRansacIterations; ++iteration) {
        std::array<int, 4> sample_indices{};
        if (correspondences.size() == sample_indices.size()) {
            std::iota(sample_indices.begin(), sample_indices.end(), 0);
        } else {
            for (int sample = 0; sample < 4; ++sample) {
                int candidate = distribution(random);
                while (std::find(sample_indices.begin(),
                                 sample_indices.begin() + sample,
                                 candidate) !=
                       sample_indices.begin() + sample) {
                    candidate = distribution(random);
                }
                sample_indices[sample] = candidate;
            }
            std::sort(sample_indices.begin(), sample_indices.end());
        }

        std::vector<cv::Point3f> sample_world_points;
        std::vector<cv::Point2f> sample_image_points;
        sample_world_points.reserve(4);
        sample_image_points.reserve(4);
        for (int sample_index : sample_indices) {
            sample_world_points.push_back(
                correspondences[sample_index].world_point);
            sample_image_points.push_back(
                correspondences[sample_index].image_point);
        }
        cv::Mat rotation_vector;
        cv::Mat translation_vector;
        bool solved = false;
        try {
            solved = cv::solvePnP(
                sample_world_points, sample_image_points, camera_matrix,
                cv::noArray(), rotation_vector, translation_vector, false,
                cv::SOLVEPNP_EPNP);
        } catch (const cv::Exception&) {
            continue;
        }
        Eigen::Matrix3d rotation;
        Eigen::Vector3d translation;
        if (!solved ||
            !OpenCvPoseToEigen(rotation_vector, translation_vector, &rotation,
                               &translation)) {
            continue;
        }
        double squared_error = 0.0;
        std::vector<int> inliers =
            ScorePnpPose(correspondences, rotation, translation,
                         frame.intrinsics, &squared_error);
        if (inliers.size() > best_inliers.size() ||
            (inliers.size() == best_inliers.size() &&
             squared_error < best_squared_error)) {
            best_inliers = std::move(inliers);
            best_squared_error = squared_error;
            best_rotation_vector = rotation_vector.clone();
            best_translation_vector = translation_vector.clone();
        }
    }
    if (best_inliers.size() < 4U) {
        return result;
    }

    std::vector<cv::Point3f> inlier_world_points;
    std::vector<cv::Point2f> inlier_image_points;
    inlier_world_points.reserve(best_inliers.size());
    inlier_image_points.reserve(best_inliers.size());
    for (int inlier : best_inliers) {
        inlier_world_points.push_back(correspondences[inlier].world_point);
        inlier_image_points.push_back(correspondences[inlier].image_point);
    }
    bool refinement_seeded = false;
    try {
        refinement_seeded = cv::solvePnP(
            inlier_world_points, inlier_image_points, camera_matrix,
            cv::noArray(), best_rotation_vector, best_translation_vector, true,
            cv::SOLVEPNP_ITERATIVE);
        if (!refinement_seeded) {
            return result;
        }
        cv::solvePnPRefineLM(inlier_world_points, inlier_image_points,
                             camera_matrix, cv::noArray(),
                             best_rotation_vector, best_translation_vector);
    } catch (const cv::Exception&) {
        return result;
    }
    Eigen::Matrix3d refined_rotation;
    Eigen::Vector3d refined_translation;
    if (!OpenCvPoseToEigen(best_rotation_vector, best_translation_vector,
                           &refined_rotation, &refined_translation)) {
        return result;
    }
    double refined_squared_error = 0.0;
    const std::vector<int> refined_inliers =
        ScorePnpPose(correspondences, refined_rotation, refined_translation,
                     frame.intrinsics, &refined_squared_error);
    if (refined_inliers.size() < 4U) {
        return result;
    }
    result.success = true;
    result.inlier_count = static_cast<int>(refined_inliers.size());
    result.camera_from_world = autonomy::common::math::SE3f(
        refined_rotation.cast<float>(), refined_translation.cast<float>());
    return result;
}

}  // namespace

namespace internal {

bool AppendUniqueRgbdMapReanchorDescriptor(const cv::Mat& descriptor,
                                           cv::Mat* descriptor_bank) {
    if (!descriptor_bank || descriptor.rows != 1 || descriptor.cols != 32 ||
        descriptor.type() != CV_8U || descriptor_bank->rows < 0 ||
        descriptor_bank->rows >=
            kRgbdMapReanchorDescriptorRowsPerMapPoint ||
        (!descriptor_bank->empty() &&
         (descriptor_bank->cols != 32 ||
          descriptor_bank->type() != CV_8U))) {
        return false;
    }
    for (int row = 0; row < descriptor_bank->rows; ++row) {
        if (OrbMatcher::DescriptorDistance(descriptor_bank->row(row),
                                           descriptor) == 0) {
            return false;
        }
    }
    descriptor_bank->push_back(descriptor);
    return true;
}

RgbdMapReanchorSelection SelectRgbdMapReanchorCandidates(
    const std::vector<RgbdMapReanchorCandidate>& candidates,
    double descriptor_ratio, int maximum_descriptor_distance) {
    RgbdMapReanchorSelection result;
    if (!std::isfinite(descriptor_ratio) || descriptor_ratio <= 0.0 ||
        descriptor_ratio >= 1.0 || maximum_descriptor_distance < 0 ||
        maximum_descriptor_distance > OrbMatcher::TH_HIGH) {
        return result;
    }

    std::map<std::size_t, std::vector<const RgbdMapReanchorCandidate*>>
        by_map_point;
    std::map<int, std::vector<const RgbdMapReanchorCandidate*>> by_feature;
    for (const RgbdMapReanchorCandidate& candidate : candidates) {
        if (!IsFiniteCandidate(candidate) ||
            candidate.descriptor_distance > 256) {
            continue;
        }
        by_map_point[candidate.map_point_index].push_back(&candidate);
        by_feature[candidate.feature_index].push_back(&candidate);
    }

    std::map<std::size_t, const RgbdMapReanchorCandidate*>
        best_by_map_point;
    for (auto& [map_point_index, ranked] : by_map_point) {
        std::sort(ranked.begin(), ranked.end(), CandidateRankLess);
        if (PassesDescriptorRatio(ranked, descriptor_ratio,
                                  maximum_descriptor_distance)) {
            best_by_map_point.emplace(map_point_index, ranked.front());
        }
    }

    std::map<int, const RgbdMapReanchorCandidate*> best_by_feature;
    for (auto& [feature_index, ranked] : by_feature) {
        std::sort(ranked.begin(), ranked.end(), CandidateRankLess);
        if (PassesDescriptorRatio(ranked, descriptor_ratio,
                                  maximum_descriptor_distance)) {
            best_by_feature.emplace(feature_index, ranked.front());
        }
    }

    std::vector<const RgbdMapReanchorCandidate*> mutual;
    mutual.reserve(best_by_map_point.size());
    for (const auto& [map_point_index, candidate] : best_by_map_point) {
        static_cast<void>(map_point_index);
        const auto feature_match =
            best_by_feature.find(candidate->feature_index);
        if (feature_match != best_by_feature.end() &&
            feature_match->second->map_point_index ==
                candidate->map_point_index) {
            mutual.push_back(candidate);
        }
    }
    result.mutual_matches = static_cast<int>(mutual.size());
    std::sort(mutual.begin(), mutual.end(), CandidateRankLess);

    std::set<std::size_t> assigned_map_points;
    std::set<int> assigned_features;
    result.assigned.reserve(mutual.size());
    for (const RgbdMapReanchorCandidate* candidate : mutual) {
        if (!assigned_map_points.insert(candidate->map_point_index).second ||
            !assigned_features.insert(candidate->feature_index).second) {
            continue;
        }
        result.assigned.push_back(*candidate);
    }
    return result;
}

RgbdMapReanchorSelection SelectRgbdMapReanchorGeometryCandidates(
    const std::vector<RgbdMapReanchorCandidate>& candidates,
    double maximum_xyz_error_meters) {
    RgbdMapReanchorSelection result;
    const bool finite_limit = std::isfinite(maximum_xyz_error_meters);
    if ((finite_limit && maximum_xyz_error_meters < 0.0) ||
        (!finite_limit &&
         !(std::isinf(maximum_xyz_error_meters) &&
           maximum_xyz_error_meters > 0.0))) {
        return result;
    }
    const double maximum_squared_xyz_error =
        finite_limit
            ? maximum_xyz_error_meters * maximum_xyz_error_meters
            : std::numeric_limits<double>::infinity();

    std::map<std::size_t, std::vector<const RgbdMapReanchorCandidate*>>
        by_map_point;
    std::map<int, std::vector<const RgbdMapReanchorCandidate*>> by_feature;
    for (const RgbdMapReanchorCandidate& candidate : candidates) {
        if (!IsFiniteCandidate(candidate) ||
            candidate.descriptor_distance > 256 ||
            candidate.squared_xyz_error_meters >
                maximum_squared_xyz_error) {
            continue;
        }
        by_map_point[candidate.map_point_index].push_back(&candidate);
        by_feature[candidate.feature_index].push_back(&candidate);
    }

    std::map<std::size_t, const RgbdMapReanchorCandidate*>
        best_by_map_point;
    for (auto& [map_point_index, ranked] : by_map_point) {
        std::sort(ranked.begin(), ranked.end(), GeometryCandidateRankLess);
        best_by_map_point.emplace(map_point_index, ranked.front());
    }
    std::map<int, const RgbdMapReanchorCandidate*> best_by_feature;
    for (auto& [feature_index, ranked] : by_feature) {
        std::sort(ranked.begin(), ranked.end(), GeometryCandidateRankLess);
        best_by_feature.emplace(feature_index, ranked.front());
    }

    std::vector<const RgbdMapReanchorCandidate*> mutual;
    mutual.reserve(best_by_map_point.size());
    for (const auto& [map_point_index, candidate] : best_by_map_point) {
        static_cast<void>(map_point_index);
        const auto feature_match =
            best_by_feature.find(candidate->feature_index);
        if (feature_match != best_by_feature.end() &&
            feature_match->second->map_point_index ==
                candidate->map_point_index) {
            mutual.push_back(candidate);
        }
    }
    result.mutual_matches = static_cast<int>(mutual.size());
    std::sort(mutual.begin(), mutual.end(), GeometryCandidateRankLess);

    std::set<std::size_t> assigned_map_points;
    std::set<int> assigned_features;
    result.assigned.reserve(mutual.size());
    for (const RgbdMapReanchorCandidate* candidate : mutual) {
        if (!assigned_map_points.insert(candidate->map_point_index).second ||
            !assigned_features.insert(candidate->feature_index).second) {
            continue;
        }
        result.assigned.push_back(*candidate);
    }
    return result;
}

bool IsWithinRgbdMapReanchorReprojectionGate(
    double squared_reprojection_error_pixels) {
    constexpr double kMaximumSquaredError =
        kRgbdTemporalMapReprojectionLimitPixels *
        kRgbdTemporalMapReprojectionLimitPixels;
    return std::isfinite(squared_reprojection_error_pixels) &&
           squared_reprojection_error_pixels >= 0.0 &&
           squared_reprojection_error_pixels <= kMaximumSquaredError;
}

}  // namespace internal

RgbdMapReanchorMatcher::RgbdMapReanchorMatcher() = default;

RgbdMapReanchorMatcher::RgbdMapReanchorMatcher(const Options& options)
    : options_(options) {}

RgbdMapReanchorMatcher::Result RgbdMapReanchorMatcher::Match(
    const FrameView& frame, const std::vector<MapPointView>& local_map_points,
    const autonomy::common::math::SE3f& provisional_camera_from_world) const {
    Result result = MatchAtPose(frame, local_map_points,
                                provisional_camera_from_world);
    if (!result.diagnostics.input_valid ||
        result.diagnostics.assigned_matches < 4) {
        return result;
    }
    result.pnp_attempted = true;
    const PnpEstimate pnp = EstimateDeterministicMapPose(
        frame, local_map_points, result.associations);
    result.pnp_succeeded = pnp.success;
    result.pnp_inliers = pnp.inlier_count;
    if (!pnp.success) {
        return result;
    }
    result.refined_camera_from_world = pnp.camera_from_world;
    Result refined =
        MatchAtPose(frame, local_map_points, pnp.camera_from_world);
    result.refined_associations = std::move(refined.associations);
    result.refined_diagnostics = refined.diagnostics;
    return result;
}

RgbdMapReanchorMatcher::AssociationValidation
RgbdMapReanchorMatcher::ValidateFiveCentimeterAssociations(
    const FrameView& frame, const std::vector<AssociationView>& associations,
    const autonomy::common::math::SE3f& camera_from_world) const {
    AssociationValidation result;
    result.input_associations = static_cast<int>(associations.size());
    const bool pose_valid =
        camera_from_world.translation().allFinite() &&
        camera_from_world.rotationMatrix().allFinite();
    if (!HasValidGeometryFrameStorage(frame) || !pose_valid) {
        return result;
    }
    result.input_valid = true;

    std::map<MapPoint*, int> pointer_counts;
    std::map<std::uint64_t, int> id_counts;
    std::map<int, int> feature_counts;
    for (const AssociationView& association : associations) {
        if (!association.map_point) {
            continue;
        }
        ++pointer_counts[association.map_point];
        ++id_counts[association.map_point_id];
        ++feature_counts[association.feature_index];
    }

    constexpr double kMaximumXyzErrorMeters = 0.05;
    constexpr double kMaximumSquaredXyzErrorMeters =
        kMaximumXyzErrorMeters * kMaximumXyzErrorMeters;
    const double fx = frame.intrinsics(0, 0);
    const double fy = frame.intrinsics(1, 1);
    const double cx = frame.intrinsics(0, 2);
    const double cy = frame.intrinsics(1, 2);
    const double image_width = frame.maximum_x - frame.minimum_x;
    const double image_height = frame.maximum_y - frame.minimum_y;
    std::array<bool, kGridColumns * kGridRows> occupied_grid_cells{};
    result.accepted_feature_indices.reserve(associations.size());
    for (const AssociationView& association : associations) {
        if (!association.map_point || !association.valid_for_expected_map ||
            association.optimizer_outlier ||
            !association.world_point.allFinite() ||
            pointer_counts[association.map_point] != 1 ||
            id_counts[association.map_point_id] != 1 ||
            feature_counts[association.feature_index] != 1 ||
            association.feature_index < 0 ||
            association.feature_index >= frame.feature_count) {
            continue;
        }
        ++result.valid_unique_associations;

        const cv::Point2f& observed_pixel =
            (*frame.keypoints)[association.feature_index].pt;
        const double measured_depth =
            (*frame.depths)[association.feature_index];
        if (!std::isfinite(observed_pixel.x) ||
            !std::isfinite(observed_pixel.y) ||
            observed_pixel.x < frame.minimum_x ||
            observed_pixel.x > frame.maximum_x ||
            observed_pixel.y < frame.minimum_y ||
            observed_pixel.y > frame.maximum_y ||
            !std::isfinite(measured_depth) || measured_depth <= 0.0) {
            continue;
        }

        const Eigen::Vector3d predicted_point =
            (camera_from_world * association.world_point).cast<double>();
        if (!predicted_point.allFinite() || predicted_point.z() <= 0.0) {
            continue;
        }
        const double projected_x =
            fx * predicted_point.x() / predicted_point.z() + cx;
        const double projected_y =
            fy * predicted_point.y() / predicted_point.z() + cy;
        if (!std::isfinite(projected_x) || !std::isfinite(projected_y) ||
            projected_x < frame.minimum_x ||
            projected_x > frame.maximum_x ||
            projected_y < frame.minimum_y ||
            projected_y > frame.maximum_y) {
            continue;
        }
        const double residual_x = projected_x - observed_pixel.x;
        const double residual_y = projected_y - observed_pixel.y;
        const double squared_reprojection_error =
            residual_x * residual_x + residual_y * residual_y;
        if (!internal::IsWithinRgbdMapReanchorReprojectionGate(
                squared_reprojection_error)) {
            continue;
        }
        const Eigen::Vector3d measured_point(
            (static_cast<double>(observed_pixel.x) - cx) * measured_depth /
                fx,
            (static_cast<double>(observed_pixel.y) - cy) * measured_depth /
                fy,
            measured_depth);
        const double squared_xyz_error =
            (predicted_point - measured_point).squaredNorm();
        if (!measured_point.allFinite() ||
            !std::isfinite(squared_xyz_error) || squared_xyz_error < 0.0 ||
            squared_xyz_error > kMaximumSquaredXyzErrorMeters) {
            continue;
        }

        result.accepted_feature_indices.push_back(
            association.feature_index);
        const int column = std::clamp(
            static_cast<int>((observed_pixel.x - frame.minimum_x) /
                             image_width * kGridColumns),
            0, kGridColumns - 1);
        const int row = std::clamp(
            static_cast<int>((observed_pixel.y - frame.minimum_y) /
                             image_height * kGridRows),
            0, kGridRows - 1);
        occupied_grid_cells[row * kGridColumns + column] = true;
    }
    std::sort(result.accepted_feature_indices.begin(),
              result.accepted_feature_indices.end());
    result.consistent_associations =
        static_cast<int>(result.accepted_feature_indices.size());
    result.occupied_grid_cells = static_cast<int>(std::count(
        occupied_grid_cells.begin(), occupied_grid_cells.end(), true));
    return result;
}

RgbdMapReanchorMatcher::Result RgbdMapReanchorMatcher::MatchAtPose(
    const FrameView& frame, const std::vector<MapPointView>& local_map_points,
    const autonomy::common::math::SE3f& camera_from_world) const {
    Result result;
    result.diagnostics.input_map_points =
        static_cast<int>(local_map_points.size());
    if (frame.feature_count > 0) {
        result.associations.assign(frame.feature_count, nullptr);
        result.geometry_five_centimeter_associations.assign(
            frame.feature_count, nullptr);
    }

    const bool options_valid =
        std::isfinite(options_.descriptor_ratio) &&
        options_.descriptor_ratio > 0.0 &&
        options_.descriptor_ratio < 1.0 &&
        kDescriptorTelemetryThresholds.front() == OrbMatcher::TH_LOW &&
        kDescriptorTelemetryThresholds.back() == OrbMatcher::TH_HIGH;
    const bool pose_valid =
        camera_from_world.translation().allFinite() &&
        camera_from_world.rotationMatrix().allFinite();
    if (!options_valid || !HasValidFrameStorage(frame) || !pose_valid) {
        return result;
    }
    result.diagnostics.input_valid = true;

    std::map<std::uint64_t, int> id_counts;
    std::map<MapPoint*, int> pointer_counts;
    for (const MapPointView& map_point : local_map_points) {
        if (!map_point.map_point || !map_point.valid_for_expected_map ||
            !map_point.world_point.allFinite() ||
            !HasValidDescriptorBank(map_point.descriptor_bank,
                                    map_point.observation_descriptor_rows)) {
            continue;
        }
        ++id_counts[map_point.map_point_id];
        ++pointer_counts[map_point.map_point];
    }
    std::map<std::uint64_t, ValidMapPoint> valid_map_points;
    for (std::size_t view_index = 0; view_index < local_map_points.size();
         ++view_index) {
        const MapPointView& map_point = local_map_points[view_index];
        if (!map_point.map_point || !map_point.valid_for_expected_map ||
            !map_point.world_point.allFinite() ||
            !HasValidDescriptorBank(map_point.descriptor_bank,
                                    map_point.observation_descriptor_rows) ||
            id_counts[map_point.map_point_id] != 1 ||
            pointer_counts[map_point.map_point] != 1) {
            continue;
        }
        valid_map_points.emplace(
            map_point.map_point_id,
            ValidMapPoint{view_index, map_point.map_point,
                          map_point.world_point, map_point.descriptor_bank,
                          map_point.observation_descriptor_rows});
    }
    result.diagnostics.valid_unique_map_points =
        static_cast<int>(valid_map_points.size());
    for (const auto& [map_point_id, valid_point] : valid_map_points) {
        static_cast<void>(map_point_id);
        result.diagnostics.descriptor_rows +=
            valid_point.descriptor_bank.rows;
        if (valid_point.descriptor_bank.rows > 1) {
            ++result.diagnostics.multi_descriptor_map_points;
        }
    }

    std::vector<int> feature_order;
    feature_order.reserve(frame.feature_count);
    for (int feature_index = 0; feature_index < frame.feature_count;
         ++feature_index) {
        const cv::Point2f& point = (*frame.keypoints)[feature_index].pt;
        if (std::isfinite(point.x) && std::isfinite(point.y)) {
            feature_order.push_back(feature_index);
        }
    }
    std::sort(feature_order.begin(), feature_order.end(),
              [&frame](int left, int right) {
                  const cv::Point2f& left_point =
                      (*frame.keypoints)[left].pt;
                  const cv::Point2f& right_point =
                      (*frame.keypoints)[right].pt;
                  return std::tie(left_point.x, left_point.y, left) <
                         std::tie(right_point.x, right_point.y, right);
              });

    const double fx = frame.intrinsics(0, 0);
    const double fy = frame.intrinsics(1, 1);
    const double cx = frame.intrinsics(0, 2);
    const double cy = frame.intrinsics(1, 2);
    constexpr double kRadius =
        internal::kRgbdTemporalMapReprojectionLimitPixels;
    std::vector<internal::RgbdMapReanchorCandidate> candidates;
    candidates.reserve(valid_map_points.size());
    for (const auto& [map_point_id, valid_point] : valid_map_points) {
        const Eigen::Vector3f predicted_point =
            camera_from_world * valid_point.world_point;
        if (!predicted_point.allFinite() || predicted_point.z() <= 0.0F) {
            continue;
        }
        const double projected_x =
            fx * predicted_point.x() / predicted_point.z() + cx;
        const double projected_y =
            fy * predicted_point.y() / predicted_point.z() + cy;
        if (!std::isfinite(projected_x) || !std::isfinite(projected_y) ||
            projected_x < frame.minimum_x ||
            projected_x > frame.maximum_x ||
            projected_y < frame.minimum_y ||
            projected_y > frame.maximum_y) {
            continue;
        }
        ++result.diagnostics.projected_map_points;

        const auto first_feature = std::lower_bound(
            feature_order.begin(), feature_order.end(), projected_x - kRadius,
            [&frame](int feature_index, double minimum_x) {
                return (*frame.keypoints)[feature_index].pt.x < minimum_x;
            });
        for (auto feature_it = first_feature;
             feature_it != feature_order.end(); ++feature_it) {
            const int feature_index = *feature_it;
            const cv::Point2f& observed_pixel =
                (*frame.keypoints)[feature_index].pt;
            if (observed_pixel.x > projected_x + kRadius) {
                break;
            }
            if (observed_pixel.x < frame.minimum_x ||
                observed_pixel.x > frame.maximum_x ||
                observed_pixel.y < frame.minimum_y ||
                observed_pixel.y > frame.maximum_y) {
                continue;
            }
            const double residual_x = observed_pixel.x - projected_x;
            const double residual_y = observed_pixel.y - projected_y;
            const double squared_reprojection_error =
                residual_x * residual_x + residual_y * residual_y;
            if (!internal::IsWithinRgbdMapReanchorReprojectionGate(
                    squared_reprojection_error)) {
                continue;
            }
            ++result.diagnostics.nearby_candidates;

            const double measured_depth = (*frame.depths)[feature_index];
            if (!std::isfinite(measured_depth) || measured_depth <= 0.0) {
                continue;
            }
            const Eigen::Vector3d measured_point(
                (static_cast<double>(observed_pixel.x) - cx) *
                    measured_depth / fx,
                (static_cast<double>(observed_pixel.y) - cy) *
                    measured_depth / fy,
                measured_depth);
            const double xyz_error =
                (predicted_point.cast<double>() - measured_point).norm();
            if (!measured_point.allFinite() || !std::isfinite(xyz_error) ||
                !internal::IsRgbdTemporalMapXyzConsistent(
                    predicted_point.z(), measured_depth, xyz_error)) {
                continue;
            }
            ++result.diagnostics.candidates_after_geometry;

            int descriptor_distance = 257;
            int best_descriptor_row = -1;
            for (int descriptor_row = 0;
                 descriptor_row < valid_point.descriptor_bank.rows;
                 ++descriptor_row) {
                const int candidate_distance = OrbMatcher::DescriptorDistance(
                    valid_point.descriptor_bank.row(descriptor_row),
                    frame.descriptors->row(feature_index));
                if (candidate_distance < descriptor_distance) {
                    descriptor_distance = candidate_distance;
                    best_descriptor_row = descriptor_row;
                }
            }
            if (descriptor_distance < 0 || descriptor_distance > 256) {
                continue;
            }
            if (descriptor_distance <= OrbMatcher::TH_LOW) {
                ++result.diagnostics.candidates_after_descriptor;
            }
            candidates.push_back(
                {valid_point.view_index, map_point_id, feature_index,
                 descriptor_distance, squared_reprojection_error,
                 xyz_error * xyz_error,
                 best_descriptor_row >= 0 &&
                     best_descriptor_row <
                         valid_point.observation_descriptor_rows});
        }
    }

    internal::RgbdMapReanchorSelection selection;
    for (std::size_t threshold_index = 0;
         threshold_index <
         kGeometryTelemetryMaximumXyzErrorsMeters.size();
         ++threshold_index) {
        const double maximum_xyz_error_meters =
            kGeometryTelemetryMaximumXyzErrorsMeters[threshold_index];
        const internal::RgbdMapReanchorSelection geometry_selection =
            internal::SelectRgbdMapReanchorGeometryCandidates(
                candidates, maximum_xyz_error_meters);
        GeometryThresholdSupport& support =
            result.diagnostics
                .geometry_threshold_support[threshold_index];
        support.maximum_xyz_error_meters = maximum_xyz_error_meters;
        support.mutual_matches = geometry_selection.mutual_matches;
        support.assigned_matches =
            static_cast<int>(geometry_selection.assigned.size());
        support.occupied_grid_cells =
            CountOccupiedGridCells(frame, geometry_selection.assigned);
        if (threshold_index == 0U) {
            for (const internal::RgbdMapReanchorCandidate& match :
                 geometry_selection.assigned) {
                if (match.map_point_index >= local_map_points.size() ||
                    match.feature_index < 0 ||
                    match.feature_index >= frame.feature_count ||
                    result.geometry_five_centimeter_associations
                        [match.feature_index]) {
                    continue;
                }
                result.geometry_five_centimeter_associations
                    [match.feature_index] =
                    local_map_points[match.map_point_index].map_point;
            }
        }
    }
    for (std::size_t threshold_index = 0;
         threshold_index < kDescriptorTelemetryThresholds.size();
         ++threshold_index) {
        const int maximum_descriptor_distance =
            kDescriptorTelemetryThresholds[threshold_index];
        internal::RgbdMapReanchorSelection threshold_selection =
            internal::SelectRgbdMapReanchorCandidates(
                candidates, options_.descriptor_ratio,
                maximum_descriptor_distance);
        DescriptorThresholdSupport& support =
            result.diagnostics
                .descriptor_threshold_support[threshold_index];
        support.maximum_descriptor_distance = maximum_descriptor_distance;
        support.mutual_matches = threshold_selection.mutual_matches;
        support.assigned_matches =
            static_cast<int>(threshold_selection.assigned.size());
        support.occupied_grid_cells =
            CountOccupiedGridCells(frame, threshold_selection.assigned);
        if (threshold_index == 0U) {
            selection = std::move(threshold_selection);
        }
    }
    result.diagnostics.mutual_matches = selection.mutual_matches;
    result.diagnostics.assigned_matches = 0;
    std::array<bool, kGridColumns * kGridRows> occupied_grid_cells{};
    double squared_reprojection_error = 0.0;
    double squared_xyz_error = 0.0;
    for (const internal::RgbdMapReanchorCandidate& match :
         selection.assigned) {
        if (match.map_point_index >= local_map_points.size() ||
            match.feature_index < 0 ||
            match.feature_index >= frame.feature_count ||
            result.associations[match.feature_index]) {
            continue;
        }
        result.associations[match.feature_index] =
            local_map_points[match.map_point_index].map_point;
        ++result.diagnostics.assigned_matches;
        if (match.used_observation_descriptor) {
            ++result.diagnostics.observation_descriptor_matches;
        }
        squared_reprojection_error +=
            match.squared_reprojection_error_pixels;
        squared_xyz_error += match.squared_xyz_error_meters;

        const cv::Point2f& point = (*frame.keypoints)[match.feature_index].pt;
        const double image_width = frame.maximum_x - frame.minimum_x;
        const double image_height = frame.maximum_y - frame.minimum_y;
        const int column = std::clamp(
            static_cast<int>((point.x - frame.minimum_x) / image_width *
                             kGridColumns),
            0, kGridColumns - 1);
        const int row = std::clamp(
            static_cast<int>((point.y - frame.minimum_y) / image_height *
                             kGridRows),
            0, kGridRows - 1);
        occupied_grid_cells[row * kGridColumns + column] = true;
    }
    result.diagnostics.occupied_grid_cells = static_cast<int>(std::count(
        occupied_grid_cells.begin(), occupied_grid_cells.end(), true));
    if (result.diagnostics.assigned_matches > 0) {
        result.diagnostics.reprojection_rmse_pixels =
            std::sqrt(squared_reprojection_error /
                      result.diagnostics.assigned_matches);
        result.diagnostics.xyz_rmse_meters =
            std::sqrt(squared_xyz_error /
                      result.diagnostics.assigned_matches);
    }
    return result;
}

}  // namespace autonomy::localization::atlas
