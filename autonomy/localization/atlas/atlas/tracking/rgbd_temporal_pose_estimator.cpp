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

#include "atlas/tracking/rgbd_temporal_pose_estimator.hpp"
#include "atlas/tracking/recovery_policy.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <stdexcept>
#include <tuple>
#include <utility>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <opencv2/calib3d.hpp>

namespace autonomy::localization::atlas {
namespace {

constexpr int kGridColumns = 4;
constexpr int kGridRows = 4;
constexpr int kGridCellCount = kGridColumns * kGridRows;
constexpr int kRigidRansacIterations = 500;
constexpr int kMinimumInitialProposalMatches = 12;
constexpr int kMinimumRigidProposalInliers = 10;
constexpr int kMinimumFinalRigidCandidates = 30;
constexpr int kMinimumFinalRigidInliers = 20;
constexpr int kMinimumFinalXyzChecks = 12;
constexpr double kMinimumRigidInlierRatio = 0.60;
constexpr double kMinimumFinalXyzCheckRatio = 0.60;
constexpr double kMinimumFinalXyzConsistencyRatio = 0.65;
constexpr double kMaximumRigidRmseMeters = 0.06;
constexpr double kMaximumRigidMedianMeters = 0.04;
constexpr double kFiveCentimeters = 0.05;
constexpr double kMaximumAcceptedXyzRmseMeters = 0.15;
constexpr double kMaximumLowCoverageXyzRmseMeters = 0.10;
constexpr double kMaximumSeedTranslationDeltaMeters = 0.10;
constexpr double kMaximumSeedRotationDeltaRadians =
    2.0 * 3.14159265358979323846 / 180.0;
constexpr float kMaximumHybridReprojectionErrorPixels = 3.0F;
constexpr float kGuidedSearchRadiusPixels = 20.0F;
constexpr int kMaximumGuidedReferenceFeatures = 800;
constexpr int kMaximumGuidedCandidatesPerReference = 8;
constexpr int kMaximumGuidedMatches = 64;

struct MatchPair {
    int reference_index = -1;
    int current_index = -1;
};

struct RigidObservation {
    Eigen::Vector3d reference_point = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_point = Eigen::Vector3d::Zero();
    bool has_current_point = false;
};

struct RigidEstimate {
    bool valid = false;
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    std::vector<int> inlier_indices;
    int valid_pair_count = 0;
    int five_centimeter_inlier_count = 0;
    double inlier_ratio = 0.0;
    double rmse_meters = std::numeric_limits<double>::infinity();
    double median_meters = std::numeric_limits<double>::infinity();
};

struct GuidedProposal {
    int reference_index = -1;
    int current_index = -1;
    int descriptor_distance = std::numeric_limits<int>::max();
    float squared_reprojection_distance =
        std::numeric_limits<float>::infinity();
};

bool IsValidOptions(const RgbdTemporalPoseEstimator::Options& options) {
    return std::isfinite(options.descriptor_ratio) &&
           options.descriptor_ratio > 0.0F && options.descriptor_ratio < 1.0F &&
           options.maximum_descriptor_distance > 0 &&
           options.maximum_descriptor_distance <= 256 &&
           options.minimum_matches >= 4 && options.minimum_inliers >= 4 &&
           options.minimum_matches >= options.minimum_inliers &&
           options.ransac_iterations > 0 &&
           std::isfinite(options.ransac_reprojection_error_pixels) &&
           options.ransac_reprojection_error_pixels > 0.0F &&
           std::isfinite(options.ransac_confidence) &&
           options.ransac_confidence > 0.0 && options.ransac_confidence < 1.0 &&
           options.minimum_grid_cells > 0 &&
           options.minimum_grid_cells <= kGridCellCount &&
           options.minimum_depth_checks > 0 &&
           std::isfinite(options.depth_absolute_tolerance_meters) &&
           options.depth_absolute_tolerance_meters >= 0.0F &&
           std::isfinite(options.depth_relative_tolerance) &&
           options.depth_relative_tolerance >= 0.0F &&
           (options.depth_absolute_tolerance_meters > 0.0F ||
            options.depth_relative_tolerance > 0.0F) &&
           std::isfinite(options.minimum_depth_consistency_ratio) &&
           options.minimum_depth_consistency_ratio > 0.0F &&
           options.minimum_depth_consistency_ratio <= 1.0F &&
           std::isfinite(options.minimum_depth_check_ratio) &&
           options.minimum_depth_check_ratio > 0.0F &&
           options.minimum_depth_check_ratio <= 1.0F &&
           std::isfinite(options.maximum_reprojection_rmse_pixels) &&
           options.maximum_reprojection_rmse_pixels > 0.0F &&
           std::isfinite(options.maximum_pose_condition_number) &&
           options.maximum_pose_condition_number >= 1.0 &&
           std::isfinite(options.maximum_reference_age_seconds) &&
           options.maximum_reference_age_seconds > 0.0;
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

bool IsFiniteRotationMatrix(const cv::Mat& rotation) {
    return rotation.rows == 3 && rotation.cols == 3 &&
           rotation.channels() == 1 && rotation.type() == CV_64F &&
           cv::checkRange(rotation, true);
}

bool IsFinitePoint(const cv::Point2f& point) {
    return std::isfinite(point.x) && std::isfinite(point.y);
}

bool IsFiniteIntrinsics(const Eigen::Matrix3f& intrinsics) {
    return intrinsics.array().isFinite().all() && intrinsics(0, 0) > 0.0F &&
           intrinsics(1, 1) > 0.0F;
}

bool HasValidBounds(float minimum_x, float maximum_x, float minimum_y,
                    float maximum_y) {
    const double width =
        static_cast<double>(maximum_x) - static_cast<double>(minimum_x);
    const double height =
        static_cast<double>(maximum_y) - static_cast<double>(minimum_y);
    return std::isfinite(minimum_x) && std::isfinite(maximum_x) &&
           std::isfinite(minimum_y) && std::isfinite(maximum_y) &&
           std::isfinite(width) && width > 0.0 && std::isfinite(height) &&
           height > 0.0;
}

bool HasValidFeatureStorage(const RgbdTemporalPoseEstimator::FrameView& frame) {
    if (!frame.descriptors || !frame.keypoints || !frame.depths ||
        frame.descriptors->empty() || frame.descriptors->type() != CV_8U ||
        frame.descriptors->rows < 2 || frame.descriptors->cols != 32 ||
        !IsFiniteIntrinsics(frame.intrinsics) ||
        !HasValidBounds(frame.minimum_x, frame.maximum_x, frame.minimum_y,
                        frame.maximum_y) ||
        !std::isfinite(frame.timestamp) || frame.map_change_index < 0) {
        return false;
    }
    const std::size_t descriptor_count =
        static_cast<std::size_t>(frame.descriptors->rows);
    return frame.keypoints->size() >= descriptor_count &&
           frame.depths->size() >= descriptor_count;
}

bool PassesRatioTest(const std::vector<cv::DMatch>& matches,
                     float descriptor_ratio, int maximum_descriptor_distance) {
    return matches.size() >= 2 && matches[0].distance >= 0.0F &&
           matches[0].distance <= maximum_descriptor_distance &&
           matches[0].distance < descriptor_ratio * matches[1].distance;
}

cv::Mat CameraMatrix(const Eigen::Matrix3f& intrinsics) {
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            camera_matrix.at<double>(row, column) =
                static_cast<double>(intrinsics(row, column));
        }
    }
    return camera_matrix;
}

Eigen::Vector3d BackProject(const cv::Point2f& pixel, double depth,
                            const Eigen::Matrix3f& intrinsics) {
    return Eigen::Vector3d(
        (static_cast<double>(pixel.x) - intrinsics(0, 2)) * depth /
            intrinsics(0, 0),
        (static_cast<double>(pixel.y) - intrinsics(1, 2)) * depth /
            intrinsics(1, 1),
        depth);
}

double RigidResidual(const RigidObservation& observation,
                     const Eigen::Matrix3d& rotation,
                     const Eigen::Vector3d& translation) {
    return (rotation * observation.reference_point + translation -
            observation.current_point)
        .norm();
}

double RigidThreshold(const RigidObservation& observation) {
    return std::clamp(
        0.03 * std::max(observation.reference_point.z(),
                        observation.current_point.z()),
        0.10, 0.25);
}

bool IsNonDegenerateTriplet(const std::vector<RigidObservation>& observations,
                            int first, int second, int third) {
    const auto is_non_degenerate = [](const Eigen::Vector3d& a,
                                      const Eigen::Vector3d& b,
                                      const Eigen::Vector3d& c) {
        const Eigen::Vector3d first_edge = b - a;
        const Eigen::Vector3d second_edge = c - a;
        const double maximum_edge_squared =
            std::max({first_edge.squaredNorm(), second_edge.squaredNorm(),
                      (c - b).squaredNorm()});
        return maximum_edge_squared > 1.0e-8 &&
               first_edge.cross(second_edge).norm() >
                   1.0e-5 * maximum_edge_squared;
    };
    return is_non_degenerate(observations[first].reference_point,
                             observations[second].reference_point,
                             observations[third].reference_point) &&
           is_non_degenerate(observations[first].current_point,
                             observations[second].current_point,
                             observations[third].current_point);
}

bool EstimateKabsch(const std::vector<RigidObservation>& observations,
                    const std::vector<int>& indices,
                    Eigen::Matrix3d* rotation,
                    Eigen::Vector3d* translation) {
    if (!rotation || !translation || indices.size() < 3) {
        return false;
    }
    Eigen::Vector3d reference_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d current_center = Eigen::Vector3d::Zero();
    for (int index : indices) {
        if (index < 0 ||
            static_cast<std::size_t>(index) >= observations.size() ||
            !observations[index].has_current_point) {
            return false;
        }
        reference_center += observations[index].reference_point;
        current_center += observations[index].current_point;
    }
    reference_center /= static_cast<double>(indices.size());
    current_center /= static_cast<double>(indices.size());

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (int index : indices) {
        covariance.noalias() +=
            (observations[index].reference_point - reference_center) *
            (observations[index].current_point - current_center).transpose();
    }
    const Eigen::JacobiSVD<Eigen::Matrix3d> decomposition(
        covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Vector3d singular_values = decomposition.singularValues();
    if (decomposition.info() != Eigen::Success ||
        !singular_values.allFinite() || singular_values(0) <= 1.0e-12 ||
        singular_values(1) <= 1.0e-6 * singular_values(0)) {
        return false;
    }
    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d candidate =
        decomposition.matrixV() * decomposition.matrixU().transpose();
    if (candidate.determinant() < 0.0) {
        correction(2, 2) = -1.0;
    }
    *rotation = decomposition.matrixV() * correction *
                decomposition.matrixU().transpose();
    *translation = current_center - *rotation * reference_center;
    return rotation->allFinite() && translation->allFinite() &&
           std::abs(rotation->determinant() - 1.0) < 1.0e-6;
}

std::vector<int> RigidInliers(
    const std::vector<RigidObservation>& observations,
    const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation,
    double* squared_error) {
    std::vector<int> inliers;
    inliers.reserve(observations.size());
    *squared_error = 0.0;
    for (int index = 0; index < static_cast<int>(observations.size()); ++index) {
        if (!observations[index].has_current_point) {
            continue;
        }
        const double residual =
            RigidResidual(observations[index], rotation, translation);
        if (std::isfinite(residual) &&
            residual <= RigidThreshold(observations[index])) {
            inliers.push_back(index);
            *squared_error += residual * residual;
        }
    }
    return inliers;
}

RigidEstimate EstimateRigidTransform(
    const std::vector<RigidObservation>& observations, std::uint32_t seed) {
    RigidEstimate result;
    std::vector<int> valid_indices;
    valid_indices.reserve(observations.size());
    for (int index = 0; index < static_cast<int>(observations.size()); ++index) {
        if (observations[index].has_current_point) {
            valid_indices.push_back(index);
        }
    }
    result.valid_pair_count = static_cast<int>(valid_indices.size());
    if (valid_indices.size() < 3) {
        return result;
    }

    std::mt19937 random(seed);
    std::uniform_int_distribution<std::size_t> distribution(
        0, valid_indices.size() - 1);
    std::vector<int> best_inliers;
    double best_squared_error = std::numeric_limits<double>::infinity();
    Eigen::Matrix3d best_rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d best_translation = Eigen::Vector3d::Zero();
    for (int iteration = 0; iteration < kRigidRansacIterations; ++iteration) {
        const int first = valid_indices[distribution(random)];
        int second = valid_indices[distribution(random)];
        int third = valid_indices[distribution(random)];
        for (int attempt = 0; second == first && attempt < 8; ++attempt) {
            second = valid_indices[distribution(random)];
        }
        for (int attempt = 0;
             (third == first || third == second) && attempt < 8; ++attempt) {
            third = valid_indices[distribution(random)];
        }
        if (first == second || first == third || second == third ||
            !IsNonDegenerateTriplet(observations, first, second, third)) {
            continue;
        }
        const std::vector<int> sample = {first, second, third};
        Eigen::Matrix3d rotation;
        Eigen::Vector3d translation;
        if (!EstimateKabsch(observations, sample, &rotation, &translation)) {
            continue;
        }
        double squared_error = 0.0;
        std::vector<int> inliers =
            RigidInliers(observations, rotation, translation, &squared_error);
        if (inliers.size() > best_inliers.size() ||
            (inliers.size() == best_inliers.size() &&
             squared_error < best_squared_error)) {
            best_inliers = std::move(inliers);
            best_squared_error = squared_error;
            best_rotation = rotation;
            best_translation = translation;
        }
    }
    if (best_inliers.size() < 3 ||
        !EstimateKabsch(observations, best_inliers, &best_rotation,
                        &best_translation)) {
        return result;
    }
    for (int refinement = 0; refinement < 2; ++refinement) {
        double squared_error = 0.0;
        std::vector<int> inliers = RigidInliers(
            observations, best_rotation, best_translation, &squared_error);
        if (inliers.size() < 3 ||
            !EstimateKabsch(observations, inliers, &best_rotation,
                            &best_translation)) {
            break;
        }
        best_inliers = std::move(inliers);
    }
    double squared_error = 0.0;
    best_inliers = RigidInliers(observations, best_rotation, best_translation,
                                &squared_error);
    if (best_inliers.size() < 3) {
        return result;
    }

    std::vector<double> residuals;
    residuals.reserve(best_inliers.size());
    for (int index : best_inliers) {
        residuals.push_back(
            RigidResidual(observations[index], best_rotation,
                          best_translation));
    }
    std::sort(residuals.begin(), residuals.end());
    const std::size_t middle = residuals.size() / 2;
    result.valid = true;
    result.rotation = best_rotation;
    result.translation = best_translation;
    result.inlier_indices = std::move(best_inliers);
    result.inlier_ratio =
        static_cast<double>(result.inlier_indices.size()) /
        result.valid_pair_count;
    result.rmse_meters =
        std::sqrt(squared_error / result.inlier_indices.size());
    result.median_meters =
        residuals.size() % 2 == 0
            ? 0.5 * (residuals[middle - 1] + residuals[middle])
            : residuals[middle];
    result.five_centimeter_inlier_count = static_cast<int>(std::count_if(
        residuals.begin(), residuals.end(), [](double residual) {
            return residual <= kFiveCentimeters;
        }));
    return result;
}

bool HasStrictRigidSupport(const RigidEstimate& estimate,
                           int final_candidate_count,
                           int minimum_candidate_count,
                           int minimum_inlier_count) {
    const int required_candidate_count =
        std::max(minimum_candidate_count, kMinimumFinalRigidCandidates);
    const int required_inlier_count =
        std::max(minimum_inlier_count, kMinimumFinalRigidInliers);
    return estimate.valid &&
           final_candidate_count >= required_candidate_count &&
           estimate.valid_pair_count >= required_candidate_count &&
           static_cast<int>(estimate.inlier_indices.size()) >=
               required_inlier_count &&
           estimate.inlier_ratio >= kMinimumRigidInlierRatio &&
           estimate.rmse_meters <= kMaximumRigidRmseMeters &&
           estimate.median_meters <= kMaximumRigidMedianMeters &&
           estimate.five_centimeter_inlier_count >= required_inlier_count;
}

double RotationDeltaRadians(const Eigen::Matrix3d& first,
                            const Eigen::Matrix3d& second) {
    const double cosine = std::clamp(
        ((second * first.transpose()).trace() - 1.0) * 0.5, -1.0, 1.0);
    return std::acos(cosine);
}

bool PosesAgree(const RigidEstimate& first, const RigidEstimate& second) {
    return first.valid && second.valid &&
           (first.translation - second.translation).norm() <=
               kMaximumSeedTranslationDeltaMeters &&
           RotationDeltaRadians(first.rotation, second.rotation) <=
               kMaximumSeedRotationDeltaRadians;
}

void CopyRigidPoseToOpenCv(const RigidEstimate& estimate,
                           cv::Mat* rotation_matrix, cv::Mat* translation) {
    *rotation_matrix = cv::Mat::zeros(3, 3, CV_64F);
    *translation = cv::Mat::zeros(3, 1, CV_64F);
    for (int row = 0; row < 3; ++row) {
        translation->at<double>(row) = estimate.translation(row);
        for (int column = 0; column < 3; ++column) {
            rotation_matrix->at<double>(row, column) =
                estimate.rotation(row, column);
        }
    }
}

RigidEstimate RigidPoseFromOpenCv(const cv::Mat& rotation_matrix,
                                  const cv::Mat& translation) {
    RigidEstimate result;
    if (!IsFiniteRotationMatrix(rotation_matrix) || translation.rows != 3 ||
        translation.cols != 1 || translation.type() != CV_64F ||
        !cv::checkRange(translation, true)) {
        return result;
    }
    for (int row = 0; row < 3; ++row) {
        result.translation(row) = translation.at<double>(row);
        for (int column = 0; column < 3; ++column) {
            result.rotation(row, column) =
                rotation_matrix.at<double>(row, column);
        }
    }
    result.valid = result.rotation.allFinite() &&
                   result.translation.allFinite();
    return result;
}

void SetRigidDiagnostics(
    const RigidEstimate& estimate,
    RgbdTemporalPoseEstimator::Diagnostics* diagnostics) {
    diagnostics->rigid_depth_pair_count = estimate.valid_pair_count;
    diagnostics->rigid_inlier_count =
        static_cast<int>(estimate.inlier_indices.size());
    diagnostics->rigid_five_centimeter_inlier_count =
        estimate.five_centimeter_inlier_count;
    diagnostics->rigid_inlier_ratio = estimate.inlier_ratio;
    diagnostics->rigid_rmse_meters = estimate.rmse_meters;
    diagnostics->rigid_xyz_median_meters = estimate.median_meters;
}

void SetRetryDiagnostics(
    const RgbdTemporalPoseEstimator::Diagnostics& source,
    RgbdTemporalPoseEstimator::RetryDiagnostics* retry) {
    retry->attempted = true;
    retry->stage = source.stage;
    retry->rejection = source.rejection;
    retry->guided_match_count = source.guided_match_count;
    retry->final_match_count = source.final_match_count;
    retry->rigid_depth_pair_count = source.rigid_depth_pair_count;
    retry->rigid_inlier_count = source.rigid_inlier_count;
    retry->rigid_five_centimeter_inlier_count =
        source.rigid_five_centimeter_inlier_count;
    retry->covered_grid_cells = source.covered_grid_cells;
    retry->depth_check_count = source.depth_check_count;
    retry->depth_consistent_count = source.depth_consistent_count;
    retry->xyz_check_count = source.xyz_check_count;
    retry->xyz_consistent_count = source.xyz_consistent_count;
    retry->rigid_inlier_ratio = source.rigid_inlier_ratio;
    retry->rigid_rmse_meters = source.rigid_rmse_meters;
    retry->rigid_xyz_median_meters = source.rigid_xyz_median_meters;
    retry->xyz_rmse_meters = source.xyz_rmse_meters;
    retry->reprojection_rmse_pixels = source.reprojection_rmse_pixels;
    retry->pose_condition_number = source.pose_condition_number;
}

int GridCell(const cv::Point2f& point, float minimum_x, float maximum_x,
             float minimum_y, float maximum_y) {
    if (!IsFinitePoint(point) || point.x < minimum_x || point.x > maximum_x ||
        point.y < minimum_y || point.y > maximum_y) {
        return -1;
    }
    const double normalized_x =
        (static_cast<double>(point.x) - minimum_x) /
        (static_cast<double>(maximum_x) - minimum_x);
    const double normalized_y =
        (static_cast<double>(point.y) - minimum_y) /
        (static_cast<double>(maximum_y) - minimum_y);
    if (!std::isfinite(normalized_x) || !std::isfinite(normalized_y)) {
        return -1;
    }
    const int column = std::clamp(
        static_cast<int>(normalized_x * kGridColumns), 0, kGridColumns - 1);
    const int row = std::clamp(static_cast<int>(normalized_y * kGridRows), 0,
                               kGridRows - 1);
    return row * kGridColumns + column;
}

int CoveredGridCells(const std::vector<int>& inlier_indices,
                     const std::vector<MatchPair>& match_pairs,
                     const std::vector<cv::KeyPoint>& keypoints,
                     bool use_reference_index, float minimum_x, float maximum_x,
                     float minimum_y, float maximum_y) {
    std::array<bool, kGridCellCount> occupied{};
    for (int inlier_index : inlier_indices) {
        if (inlier_index < 0 ||
            static_cast<std::size_t>(inlier_index) >= match_pairs.size()) {
            continue;
        }
        const MatchPair& pair = match_pairs[inlier_index];
        const int feature_index =
            use_reference_index ? pair.reference_index : pair.current_index;
        if (feature_index < 0 ||
            static_cast<std::size_t>(feature_index) >= keypoints.size()) {
            continue;
        }
        const int cell = GridCell(keypoints[feature_index].pt, minimum_x,
                                  maximum_x, minimum_y, maximum_y);
        if (cell >= 0) {
            occupied[cell] = true;
        }
    }
    return static_cast<int>(std::count(occupied.begin(), occupied.end(), true));
}

int HammingDistance(const cv::Mat& reference_descriptors,
                    int reference_index, const cv::Mat& current_descriptors,
                    int current_index) {
    return static_cast<int>(std::lround(cv::norm(
        reference_descriptors.row(reference_index),
        current_descriptors.row(current_index), cv::NORM_HAMMING)));
}

int AddPoseGuidedMatches(
    const cv::Mat& reference_descriptors,
    const std::vector<cv::KeyPoint>& reference_keypoints,
    const std::vector<float>& reference_depths,
    const Eigen::Matrix3f& reference_intrinsics, float reference_minimum_x,
    float reference_maximum_x, float reference_minimum_y,
    float reference_maximum_y,
    const RgbdTemporalPoseEstimator::FrameView& frame,
    const RigidEstimate& seed, float descriptor_ratio,
    int maximum_descriptor_distance, std::vector<MatchPair>* match_pairs,
    std::vector<cv::Point3f>* object_points,
    std::vector<cv::Point2f>* image_points,
    std::vector<RigidObservation>* rigid_observations) {
    if (!seed.valid || !match_pairs || !object_points || !image_points ||
        !rigid_observations) {
        return 0;
    }
    std::vector<bool> reference_matched(reference_keypoints.size(), false);
    std::vector<bool> current_matched(frame.descriptors->rows, false);
    for (const MatchPair& pair : *match_pairs) {
        if (pair.reference_index >= 0 &&
            pair.reference_index < static_cast<int>(reference_matched.size())) {
            reference_matched[pair.reference_index] = true;
        }
        if (pair.current_index >= 0 &&
            pair.current_index < static_cast<int>(current_matched.size())) {
            current_matched[pair.current_index] = true;
        }
    }

    std::array<std::vector<int>, kGridCellCount> reference_buckets;
    for (int reference_index = 0;
         reference_index < static_cast<int>(reference_keypoints.size());
         ++reference_index) {
        if (reference_matched[reference_index] ||
            !std::isfinite(reference_depths[reference_index]) ||
            reference_depths[reference_index] <= 0.0F) {
            continue;
        }
        const int cell = GridCell(reference_keypoints[reference_index].pt,
                                  reference_minimum_x, reference_maximum_x,
                                  reference_minimum_y, reference_maximum_y);
        if (cell >= 0) {
            reference_buckets[cell].push_back(reference_index);
        }
    }
    std::array<std::size_t, kGridCellCount> bucket_offsets{};
    std::vector<int> selected_reference_indices;
    selected_reference_indices.reserve(kMaximumGuidedReferenceFeatures);
    bool made_progress = true;
    while (selected_reference_indices.size() <
               kMaximumGuidedReferenceFeatures &&
           made_progress) {
        made_progress = false;
        for (std::size_t cell = 0;
             cell < reference_buckets.size() &&
             selected_reference_indices.size() <
                 kMaximumGuidedReferenceFeatures;
             ++cell) {
            if (bucket_offsets[cell] >= reference_buckets[cell].size()) {
                continue;
            }
            selected_reference_indices.push_back(
                reference_buckets[cell][bucket_offsets[cell]++]);
            made_progress = true;
        }
    }

    const float squared_radius =
        kGuidedSearchRadiusPixels * kGuidedSearchRadiusPixels;
    std::vector<GuidedProposal> proposals;
    proposals.reserve(kMaximumGuidedMatches * 2);
    for (int reference_index : selected_reference_indices) {
        const cv::Point2f& reference_pixel =
            reference_keypoints[reference_index].pt;
        const Eigen::Vector3d reference_point = BackProject(
            reference_pixel, reference_depths[reference_index],
            reference_intrinsics);
        const Eigen::Vector3d predicted =
            seed.rotation * reference_point + seed.translation;
        if (!predicted.allFinite() || predicted.z() <= 1.0e-6) {
            continue;
        }
        const cv::Point2f predicted_pixel(
            static_cast<float>(frame.intrinsics(0, 0) * predicted.x() /
                                   predicted.z() +
                               frame.intrinsics(0, 2)),
            static_cast<float>(frame.intrinsics(1, 1) * predicted.y() /
                                   predicted.z() +
                               frame.intrinsics(1, 2)));
        if (!IsFinitePoint(predicted_pixel)) {
            continue;
        }

        std::vector<std::pair<float, int>> nearby;
        nearby.reserve(kMaximumGuidedCandidatesPerReference + 1);
        for (int current_index = 0;
             current_index < frame.descriptors->rows; ++current_index) {
            if (current_matched[current_index]) {
                continue;
            }
            const float current_depth = (*frame.depths)[current_index];
            if (!std::isfinite(current_depth) || current_depth <= 0.0F) {
                continue;
            }
            const cv::Point2f& current_pixel =
                (*frame.keypoints)[current_index].pt;
            if (!IsFinitePoint(current_pixel)) {
                continue;
            }
            const cv::Point2f difference = current_pixel - predicted_pixel;
            const float squared_distance = difference.dot(difference);
            if (squared_distance <= squared_radius) {
                nearby.emplace_back(squared_distance, current_index);
            }
        }
        std::sort(nearby.begin(), nearby.end());
        if (nearby.size() > kMaximumGuidedCandidatesPerReference) {
            nearby.resize(kMaximumGuidedCandidatesPerReference);
        }
        int best_index = -1;
        int best_distance = std::numeric_limits<int>::max();
        int second_distance = std::numeric_limits<int>::max();
        for (const auto& candidate : nearby) {
            const int distance = HammingDistance(
                reference_descriptors, reference_index, *frame.descriptors,
                candidate.second);
            if (distance < best_distance) {
                second_distance = best_distance;
                best_distance = distance;
                best_index = candidate.second;
            } else if (distance < second_distance) {
                second_distance = distance;
            }
        }
        if (best_index < 0 ||
            second_distance == std::numeric_limits<int>::max() ||
            best_distance > maximum_descriptor_distance ||
            static_cast<double>(best_distance) >=
                descriptor_ratio * second_distance) {
            continue;
        }
        const cv::Point2f reprojection_difference =
            (*frame.keypoints)[best_index].pt - predicted_pixel;
        proposals.push_back({reference_index, best_index, best_distance,
                             reprojection_difference.dot(
                                 reprojection_difference)});
    }

    std::sort(proposals.begin(), proposals.end(),
              [](const GuidedProposal& left, const GuidedProposal& right) {
                  return std::tie(left.descriptor_distance,
                                  left.squared_reprojection_distance,
                                  left.reference_index, left.current_index) <
                         std::tie(right.descriptor_distance,
                                  right.squared_reprojection_distance,
                                  right.reference_index, right.current_index);
              });
    int added = 0;
    for (const GuidedProposal& proposal : proposals) {
        if (added >= kMaximumGuidedMatches ||
            current_matched[proposal.current_index]) {
            continue;
        }
        const cv::Point2f& reference_pixel =
            reference_keypoints[proposal.reference_index].pt;
        const cv::Point2f& current_pixel =
            (*frame.keypoints)[proposal.current_index].pt;
        const double reference_depth =
            reference_depths[proposal.reference_index];
        const double current_depth = (*frame.depths)[proposal.current_index];
        const Eigen::Vector3d reference_point = BackProject(
            reference_pixel, reference_depth, reference_intrinsics);
        const Eigen::Vector3d current_point =
            BackProject(current_pixel, current_depth, frame.intrinsics);
        if (!reference_point.allFinite() || !current_point.allFinite()) {
            continue;
        }
        current_matched[proposal.current_index] = true;
        match_pairs->push_back(
            {proposal.reference_index, proposal.current_index});
        object_points->emplace_back(static_cast<float>(reference_point.x()),
                                    static_cast<float>(reference_point.y()),
                                    static_cast<float>(reference_point.z()));
        image_points->push_back(current_pixel);
        rigid_observations->push_back(
            {reference_point, current_point, true});
        ++added;
    }
    return added;
}

Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d& point) {
    Eigen::Matrix3d skew;
    skew << 0.0, -point.z(), point.y(), point.z(), 0.0, -point.x(), -point.y(),
        point.x(), 0.0;
    return skew;
}

double PoseConditionNumber(const std::vector<cv::Point3f>& object_points,
                           const std::vector<int>& inlier_indices,
                           const cv::Mat& rotation_matrix,
                           const cv::Mat& translation,
                           const Eigen::Matrix3f& intrinsics) {
    Eigen::Matrix3d rotation;
    Eigen::Vector3d offset;
    for (int row = 0; row < 3; ++row) {
        offset(row) = translation.at<double>(row);
        for (int column = 0; column < 3; ++column) {
            rotation(row, column) = rotation_matrix.at<double>(row, column);
        }
    }

    Eigen::Matrix<double, 6, 6> normal = Eigen::Matrix<double, 6, 6>::Zero();
    for (int inlier_index : inlier_indices) {
        if (inlier_index < 0 ||
            static_cast<std::size_t>(inlier_index) >= object_points.size()) {
            continue;
        }
        const cv::Point3f& object_point = object_points[inlier_index];
        const Eigen::Vector3d point =
            rotation * Eigen::Vector3d(object_point.x, object_point.y,
                                       object_point.z) +
            offset;
        if (!point.allFinite() || point.z() <= 1.0e-6) {
            continue;
        }

        const double inverse_z = 1.0 / point.z();
        const double inverse_z_squared = inverse_z * inverse_z;
        Eigen::Matrix<double, 2, 3> projection_jacobian;
        projection_jacobian
            << static_cast<double>(intrinsics(0, 0)) * inverse_z,
            0.0,
            -static_cast<double>(intrinsics(0, 0)) * point.x() *
                inverse_z_squared,
            0.0, static_cast<double>(intrinsics(1, 1)) * inverse_z,
            -static_cast<double>(intrinsics(1, 1)) * point.y() *
                inverse_z_squared;
        Eigen::Matrix<double, 3, 6> motion_jacobian;
        motion_jacobian.template leftCols<3>() = -SkewSymmetric(point);
        motion_jacobian.template rightCols<3>() = Eigen::Matrix3d::Identity();
        const Eigen::Matrix<double, 2, 6> jacobian =
            projection_jacobian * motion_jacobian;
        normal.noalias() += jacobian.transpose() * jacobian;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(normal);
    if (solver.info() != Eigen::Success) {
        return std::numeric_limits<double>::infinity();
    }
    const double minimum_eigenvalue = solver.eigenvalues().minCoeff();
    const double maximum_eigenvalue = solver.eigenvalues().maxCoeff();
    if (!std::isfinite(minimum_eigenvalue) ||
        !std::isfinite(maximum_eigenvalue) || minimum_eigenvalue <= 1.0e-9) {
        return std::numeric_limits<double>::infinity();
    }
    return maximum_eigenvalue / minimum_eigenvalue;
}

autonomy::common::math::SE3f ToSe3(const cv::Mat& rotation_matrix,
                                   const cv::Mat& translation) {
    Eigen::Matrix3f rotation;
    Eigen::Vector3f offset;
    for (int row = 0; row < 3; ++row) {
        offset(row) = static_cast<float>(translation.at<double>(row));
        for (int column = 0; column < 3; ++column) {
            rotation(row, column) =
                static_cast<float>(rotation_matrix.at<double>(row, column));
        }
    }
    return autonomy::common::math::SE3f(offset, rotation);
}

}  // namespace

const char* RgbdTemporalPoseEstimator::StageName(Stage stage) {
    switch (stage) {
        case Stage::kInputValidation:
            return "input_validation";
        case Stage::kDescriptorMatching:
            return "descriptor_matching";
        case Stage::kRigid3dRansac:
            return "rigid_3d_ransac";
        case Stage::kPoseGuidedMatching:
            return "pose_guided_matching";
        case Stage::kPnpRansac:
            return "pnp_ransac";
        case Stage::kPnpRefinement:
            return "pnp_refinement";
        case Stage::kSpatialCoverage:
            return "spatial_coverage";
        case Stage::kDepthConsistency:
            return "depth_consistency";
        case Stage::kReprojection:
            return "reprojection";
        case Stage::kConditioning:
            return "conditioning";
        case Stage::kPoseComposition:
            return "pose_composition";
        case Stage::kComplete:
            return "complete";
    }
    return "unknown";
}

const char* RgbdTemporalPoseEstimator::RejectionReasonName(
    RejectionReason rejection) {
    switch (rejection) {
        case RejectionReason::kNone:
            return "none";
        case RejectionReason::kReferenceUnavailable:
            return "reference_unavailable";
        case RejectionReason::kInvalidCurrentFrame:
            return "invalid_current_frame";
        case RejectionReason::kMapIdMismatch:
            return "map_id_mismatch";
        case RejectionReason::kCoordinateEpochMismatch:
            return "coordinate_epoch_mismatch";
        case RejectionReason::kMapRevisionMismatch:
            return "map_revision_mismatch";
        case RejectionReason::kSameFrame:
            return "same_frame";
        case RejectionReason::kTimestampRegression:
            return "timestamp_regression";
        case RejectionReason::kReferenceExpired:
            return "reference_expired";
        case RejectionReason::kDescriptorLayoutMismatch:
            return "descriptor_layout_mismatch";
        case RejectionReason::kDescriptorMatchingFailed:
            return "descriptor_matching_failed";
        case RejectionReason::kInsufficientMutualMatches:
            return "insufficient_mutual_matches";
        case RejectionReason::kRigid3dValidationFailed:
            return "rigid_3d_validation_failed";
        case RejectionReason::kRigidPoseDisagreement:
            return "rigid_pose_disagreement";
        case RejectionReason::kPnpRansacFailed:
            return "pnp_ransac_failed";
        case RejectionReason::kInsufficientPnpInliers:
            return "insufficient_pnp_inliers";
        case RejectionReason::kInvalidPnpPose:
            return "invalid_pnp_pose";
        case RejectionReason::kInvalidPnpInlierIndex:
            return "invalid_pnp_inlier_index";
        case RejectionReason::kPnpRefinementFailed:
            return "pnp_refinement_failed";
        case RejectionReason::kInvalidRefinedPose:
            return "invalid_refined_pose";
        case RejectionReason::kInsufficientGridCoverage:
            return "insufficient_grid_coverage";
        case RejectionReason::kInvalidRotationMatrix:
            return "invalid_rotation_matrix";
        case RejectionReason::kInsufficientDepthChecks:
            return "insufficient_depth_checks";
        case RejectionReason::kInsufficientDepthCheckRatio:
            return "insufficient_depth_check_ratio";
        case RejectionReason::kInsufficientDepthConsistency:
            return "insufficient_depth_consistency";
        case RejectionReason::kInsufficientXyzConsistency:
            return "insufficient_xyz_consistency";
        case RejectionReason::kExcessiveXyzError:
            return "excessive_xyz_error";
        case RejectionReason::kReprojectionFailed:
            return "reprojection_failed";
        case RejectionReason::kInvalidReprojectedPoints:
            return "invalid_reprojected_points";
        case RejectionReason::kExcessiveReprojectionError:
            return "excessive_reprojection_error";
        case RejectionReason::kIllConditionedPose:
            return "ill_conditioned_pose";
        case RejectionReason::kInvalidWorldPose:
            return "invalid_world_pose";
        case RejectionReason::kLowCoverageConfidenceFailed:
            return "low_coverage_confidence_failed";
    }
    return "unknown";
}

RgbdTemporalPoseEstimator::RgbdTemporalPoseEstimator()
    : RgbdTemporalPoseEstimator(Options()) {}

RgbdTemporalPoseEstimator::RgbdTemporalPoseEstimator(const Options& options)
    : options_(options) {
    if (!IsValidOptions(options_)) {
        throw std::invalid_argument(
            "invalid RGB-D temporal pose-estimator options");
    }
}

bool RgbdTemporalPoseEstimator::UpdateReference(
    const FrameView& frame,
    const autonomy::common::math::SE3f& camera_from_world) {
    if (!HasValidFeatureStorage(frame) ||
        !camera_from_world.translation().allFinite() ||
        !camera_from_world.rotationMatrix().allFinite()) {
        return false;
    }

    int valid_depth_count = 0;
    for (int index = 0; index < frame.descriptors->rows; ++index) {
        const float depth = (*frame.depths)[index];
        if (std::isfinite(depth) && depth > 0.0F &&
            IsFinitePoint((*frame.keypoints)[index].pt)) {
            ++valid_depth_count;
        }
    }
    const int minimum_reference_features =
        std::max(options_.minimum_matches, options_.minimum_inliers);
    if (valid_depth_count < minimum_reference_features) {
        return false;
    }

    // Store only features that can produce a 3D PnP correspondence. OpenCV
    // create/copyTo and vector resize retain capacity on the success path.
    reference_.descriptors.create(valid_depth_count, frame.descriptors->cols,
                                  frame.descriptors->type());
    reference_.keypoints.resize(valid_depth_count);
    reference_.depths.resize(valid_depth_count);
    reference_.original_feature_indices.resize(valid_depth_count);
    int output_index = 0;
    for (int input_index = 0; input_index < frame.descriptors->rows;
         ++input_index) {
        const float depth = (*frame.depths)[input_index];
        const cv::KeyPoint& keypoint = (*frame.keypoints)[input_index];
        if (!std::isfinite(depth) || depth <= 0.0F ||
            !IsFinitePoint(keypoint.pt)) {
            continue;
        }
        frame.descriptors->row(input_index)
            .copyTo(reference_.descriptors.row(output_index));
        reference_.keypoints[output_index] = keypoint;
        reference_.depths[output_index] = depth;
        reference_.original_feature_indices[output_index] = input_index;
        ++output_index;
    }
    reference_.intrinsics = frame.intrinsics;
    reference_.camera_from_world = camera_from_world;
    reference_.minimum_x = frame.minimum_x;
    reference_.maximum_x = frame.maximum_x;
    reference_.minimum_y = frame.minimum_y;
    reference_.maximum_y = frame.maximum_y;
    reference_.timestamp = frame.timestamp;
    reference_.frame_id = frame.frame_id;
    reference_.map_id = frame.map_id;
    reference_.coordinate_epoch = frame.coordinate_epoch;
    reference_.map_change_index = frame.map_change_index;
    has_reference_ = true;
    return true;
}

bool RgbdTemporalPoseEstimator::RebaseReferencePose(
    std::uint64_t expected_frame_id, std::uint64_t expected_map_id,
    std::uint64_t expected_coordinate_epoch, int expected_map_change_index,
    int new_map_change_index,
    const autonomy::common::math::SE3f& camera_from_world) {
    if (!has_reference_ || reference_.frame_id != expected_frame_id ||
        reference_.map_id != expected_map_id ||
        reference_.coordinate_epoch != expected_coordinate_epoch ||
        reference_.map_change_index != expected_map_change_index ||
        expected_map_change_index < 0 ||
        new_map_change_index <= expected_map_change_index ||
        !camera_from_world.translation().allFinite() ||
        !camera_from_world.rotationMatrix().allFinite()) {
        return false;
    }

    reference_.camera_from_world = camera_from_world;
    reference_.map_change_index = new_map_change_index;
    return true;
}

RgbdTemporalPoseEstimator::Result RgbdTemporalPoseEstimator::Estimate(
    const FrameView& frame) const {
    return EstimateInternal(frame, false, false);
}

RgbdTemporalPoseEstimator::Result RgbdTemporalPoseEstimator::EstimateInternal(
    const FrameView& frame, bool prefer_rigid_recovery,
    bool require_pnp_agreement) const {
    Result result;
    Diagnostics& diagnostics = result.diagnostics;
    diagnostics.current_frame_id = frame.frame_id;
    diagnostics.current_feature_count =
        frame.descriptors ? frame.descriptors->rows : 0;
    diagnostics.current_map_id = frame.map_id;
    diagnostics.current_coordinate_epoch = frame.coordinate_epoch;
    diagnostics.current_map_revision = frame.map_change_index;
    if (has_reference_) {
        diagnostics.reference_available = true;
        diagnostics.reference_frame_id = reference_.frame_id;
        diagnostics.reference_feature_count = reference_.descriptors.rows;
        diagnostics.reference_map_id = reference_.map_id;
        diagnostics.reference_coordinate_epoch = reference_.coordinate_epoch;
        diagnostics.reference_map_revision = reference_.map_change_index;
    }
    const auto reject = [&result](Stage stage, RejectionReason rejection) {
        result.diagnostics.stage = stage;
        result.diagnostics.rejection = rejection;
        return result;
    };

    if (!has_reference_) {
        return reject(Stage::kInputValidation,
                      RejectionReason::kReferenceUnavailable);
    }
    if (!HasValidFeatureStorage(frame)) {
        return reject(Stage::kInputValidation,
                      RejectionReason::kInvalidCurrentFrame);
    }
    if (frame.map_id != reference_.map_id) {
        return reject(Stage::kInputValidation, RejectionReason::kMapIdMismatch);
    }
    if (frame.coordinate_epoch != reference_.coordinate_epoch) {
        return reject(Stage::kInputValidation,
                      RejectionReason::kCoordinateEpochMismatch);
    }
    if (frame.map_change_index != reference_.map_change_index) {
        return reject(Stage::kInputValidation,
                      RejectionReason::kMapRevisionMismatch);
    }
    if (frame.frame_id == reference_.frame_id) {
        return reject(Stage::kInputValidation, RejectionReason::kSameFrame);
    }
    if (frame.timestamp < reference_.timestamp) {
        return reject(Stage::kInputValidation,
                      RejectionReason::kTimestampRegression);
    }
    if (frame.timestamp - reference_.timestamp >
        options_.maximum_reference_age_seconds) {
        return reject(Stage::kInputValidation,
                      RejectionReason::kReferenceExpired);
    }
    if (frame.descriptors->cols != reference_.descriptors.cols ||
        frame.descriptors->type() != reference_.descriptors.type()) {
        return reject(Stage::kInputValidation,
                      RejectionReason::kDescriptorLayoutMismatch);
    }

    diagnostics.stage = Stage::kDescriptorMatching;

    cv::BFMatcher matcher(cv::NORM_HAMMING, false);
    std::vector<std::vector<cv::DMatch>> forward_matches;
    std::vector<std::vector<cv::DMatch>> reverse_matches;
    try {
        matcher.knnMatch(reference_.descriptors, *frame.descriptors,
                         forward_matches, 2);
        matcher.knnMatch(*frame.descriptors, reference_.descriptors,
                         reverse_matches, 2);
    } catch (const cv::Exception&) {
        return reject(Stage::kDescriptorMatching,
                      RejectionReason::kDescriptorMatchingFailed);
    }

    std::vector<int> reverse_reference_index(frame.descriptors->rows, -1);
    for (const std::vector<cv::DMatch>& matches : reverse_matches) {
        if (!PassesRatioTest(matches, options_.descriptor_ratio,
                             options_.maximum_descriptor_distance)) {
            continue;
        }
        const cv::DMatch& best = matches.front();
        if (best.queryIdx >= 0 &&
            best.queryIdx < static_cast<int>(reverse_reference_index.size())) {
            reverse_reference_index[best.queryIdx] = best.trainIdx;
        }
    }

    std::vector<MatchPair> match_pairs;
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    std::vector<RigidObservation> rigid_observations;
    match_pairs.reserve(forward_matches.size());
    object_points.reserve(forward_matches.size());
    image_points.reserve(forward_matches.size());
    rigid_observations.reserve(forward_matches.size() +
                               kMaximumGuidedMatches);
    const float reference_fx = reference_.intrinsics(0, 0);
    const float reference_fy = reference_.intrinsics(1, 1);
    const float reference_cx = reference_.intrinsics(0, 2);
    const float reference_cy = reference_.intrinsics(1, 2);
    for (const std::vector<cv::DMatch>& matches : forward_matches) {
        if (!PassesRatioTest(matches, options_.descriptor_ratio,
                             options_.maximum_descriptor_distance)) {
            continue;
        }
        const cv::DMatch& best = matches.front();
        if (best.queryIdx < 0 || best.trainIdx < 0 ||
            best.queryIdx >= static_cast<int>(reference_.keypoints.size()) ||
            best.queryIdx >= static_cast<int>(reference_.depths.size()) ||
            best.trainIdx >= static_cast<int>(frame.keypoints->size()) ||
            best.trainIdx >= static_cast<int>(reverse_reference_index.size()) ||
            reverse_reference_index[best.trainIdx] != best.queryIdx) {
            continue;
        }

        const float depth = reference_.depths[best.queryIdx];
        const cv::Point2f& reference_point =
            reference_.keypoints[best.queryIdx].pt;
        const cv::Point2f& current_point = (*frame.keypoints)[best.trainIdx].pt;
        if (!std::isfinite(depth) || depth <= 0.0F ||
            !IsFinitePoint(reference_point) || !IsFinitePoint(current_point)) {
            continue;
        }
        const float reference_x =
            (reference_point.x - reference_cx) * depth / reference_fx;
        const float reference_y =
            (reference_point.y - reference_cy) * depth / reference_fy;
        if (!std::isfinite(reference_x) || !std::isfinite(reference_y)) {
            continue;
        }
        object_points.emplace_back(reference_x, reference_y, depth);
        image_points.push_back(current_point);
        match_pairs.push_back({best.queryIdx, best.trainIdx});
        RigidObservation observation;
        observation.reference_point =
            Eigen::Vector3d(reference_x, reference_y, depth);
        const float current_depth = (*frame.depths)[best.trainIdx];
        if (std::isfinite(current_depth) && current_depth > 0.0F) {
            observation.current_point = BackProject(
                current_point, current_depth, frame.intrinsics);
            observation.has_current_point =
                observation.current_point.allFinite();
        }
        rigid_observations.push_back(observation);
    }
    diagnostics.mutual_match_count = static_cast<int>(match_pairs.size());
    diagnostics.final_match_count = diagnostics.mutual_match_count;
    const std::uint32_t rigid_seed =
        0x52474244U ^ static_cast<std::uint32_t>(reference_.frame_id) ^
        static_cast<std::uint32_t>(reference_.frame_id >> 32U) ^
        static_cast<std::uint32_t>(frame.frame_id * 0x9e3779b9U) ^
        static_cast<std::uint32_t>(frame.frame_id >> 32U);
    diagnostics.stage = Stage::kRigid3dRansac;
    const RigidEstimate initial_rigid =
        EstimateRigidTransform(rigid_observations, rigid_seed);
    SetRigidDiagnostics(initial_rigid, &diagnostics);

    diagnostics.stage = Stage::kPnpRansac;
    const cv::Mat camera_matrix = CameraMatrix(frame.intrinsics);
    cv::Mat rotation_vector;
    cv::Mat rotation_matrix;
    cv::Mat translation;
    std::vector<int> inlier_indices;
    bool pnp_succeeded = false;
    bool pnp_threw = false;
    if (diagnostics.mutual_match_count >= options_.minimum_matches) {
        try {
            pnp_succeeded = cv::solvePnPRansac(
                object_points, image_points, camera_matrix, cv::noArray(),
                rotation_vector, translation, false,
                options_.ransac_iterations,
                options_.ransac_reprojection_error_pixels,
                options_.ransac_confidence, inlier_indices,
                cv::SOLVEPNP_EPNP);
        } catch (const cv::Exception&) {
            pnp_threw = true;
        }
    }
    diagnostics.pnp_inlier_count = static_cast<int>(inlier_indices.size());
    const bool pnp_has_final_support =
        pnp_succeeded &&
        static_cast<int>(inlier_indices.size()) >= options_.minimum_inliers;
    bool used_rigid_recovery = false;
    bool pnp_already_refined = false;
    RigidEstimate accepted_rigid;
    if (!pnp_has_final_support || prefer_rigid_recovery) {
        if (HasStrictRigidSupport(initial_rigid,
                                  diagnostics.mutual_match_count,
                                  options_.minimum_matches,
                                  options_.minimum_inliers)) {
            accepted_rigid = initial_rigid;
            used_rigid_recovery = true;
        } else if (diagnostics.mutual_match_count >=
                       kMinimumInitialProposalMatches &&
                   static_cast<int>(initial_rigid.inlier_indices.size()) >=
                       kMinimumRigidProposalInliers) {
            diagnostics.stage = Stage::kPoseGuidedMatching;
            diagnostics.guided_match_count = AddPoseGuidedMatches(
                reference_.descriptors, reference_.keypoints,
                reference_.depths, reference_.intrinsics,
                reference_.minimum_x, reference_.maximum_x,
                reference_.minimum_y, reference_.maximum_y, frame,
                initial_rigid, options_.descriptor_ratio,
                options_.maximum_descriptor_distance, &match_pairs,
                &object_points, &image_points, &rigid_observations);
            diagnostics.used_pose_guided_matching =
                diagnostics.guided_match_count > 0;
            diagnostics.final_match_count =
                static_cast<int>(match_pairs.size());
            diagnostics.stage = Stage::kRigid3dRansac;
            const RigidEstimate guided_rigid = EstimateRigidTransform(
                rigid_observations, rigid_seed ^ 0x9e3779b9U);
            SetRigidDiagnostics(guided_rigid, &diagnostics);
            if (HasStrictRigidSupport(guided_rigid,
                                      diagnostics.final_match_count,
                                      options_.minimum_matches,
                                      options_.minimum_inliers)) {
                if (!PosesAgree(initial_rigid, guided_rigid)) {
                    return reject(Stage::kRigid3dRansac,
                                  RejectionReason::kRigidPoseDisagreement);
                }
                accepted_rigid = guided_rigid;
                used_rigid_recovery = true;
            }
        }

        if (!used_rigid_recovery) {
            if (diagnostics.used_pose_guided_matching) {
                return reject(Stage::kRigid3dRansac,
                              RejectionReason::kRigid3dValidationFailed);
            }
            if (diagnostics.mutual_match_count < options_.minimum_matches) {
                return reject(Stage::kDescriptorMatching,
                              RejectionReason::kInsufficientMutualMatches);
            }
            if (pnp_threw || !pnp_succeeded) {
                return reject(Stage::kPnpRansac,
                              RejectionReason::kPnpRansacFailed);
            }
            if (prefer_rigid_recovery) {
                return reject(Stage::kRigid3dRansac,
                              RejectionReason::kRigid3dValidationFailed);
            }
            return reject(Stage::kPnpRansac,
                          RejectionReason::kInsufficientPnpInliers);
        }
        const bool use_guided_hybrid_pnp =
            prefer_rigid_recovery && require_pnp_agreement &&
            diagnostics.used_pose_guided_matching;
        if (!use_guided_hybrid_pnp && prefer_rigid_recovery &&
            require_pnp_agreement && !pnp_has_final_support) {
            return reject(Stage::kPnpRansac,
                          pnp_threw || !pnp_succeeded
                              ? RejectionReason::kPnpRansacFailed
                              : RejectionReason::kInsufficientPnpInliers);
        }
        if (use_guided_hybrid_pnp) {
            diagnostics.stage = Stage::kPnpRansac;
            rotation_vector.release();
            rotation_matrix.release();
            translation.release();
            inlier_indices.clear();
            try {
                pnp_succeeded = cv::solvePnPRansac(
                    object_points, image_points, camera_matrix, cv::noArray(),
                    rotation_vector, translation, false,
                    options_.ransac_iterations,
                    options_.ransac_reprojection_error_pixels,
                    options_.ransac_confidence, inlier_indices,
                    cv::SOLVEPNP_EPNP);
            } catch (const cv::Exception&) {
                return reject(Stage::kPnpRansac,
                              RejectionReason::kPnpRansacFailed);
            }
            diagnostics.pnp_inlier_count =
                static_cast<int>(inlier_indices.size());
            const int minimum_hybrid_inliers =
                std::max(options_.minimum_inliers,
                         kMinimumFinalRigidInliers);
            if (!pnp_succeeded ||
                static_cast<int>(inlier_indices.size()) <
                    minimum_hybrid_inliers) {
                return reject(Stage::kPnpRansac,
                              pnp_succeeded
                                  ? RejectionReason::kInsufficientPnpInliers
                                  : RejectionReason::kPnpRansacFailed);
            }

            cv::Mat normalized_rotation_vector;
            cv::Mat normalized_translation;
            if (!NormalizePoseVector(rotation_vector,
                                     &normalized_rotation_vector) ||
                !NormalizePoseVector(translation, &normalized_translation)) {
                return reject(Stage::kPnpRansac,
                              RejectionReason::kInvalidPnpPose);
            }
            rotation_vector = normalized_rotation_vector;
            translation = normalized_translation;

            std::vector<cv::Point3f> hybrid_object_points;
            std::vector<cv::Point2f> hybrid_image_points;
            hybrid_object_points.reserve(inlier_indices.size());
            hybrid_image_points.reserve(inlier_indices.size());
            for (int inlier_index : inlier_indices) {
                if (inlier_index < 0 ||
                    static_cast<std::size_t>(inlier_index) >=
                        object_points.size() ||
                    static_cast<std::size_t>(inlier_index) >=
                        image_points.size()) {
                    return reject(Stage::kPnpRansac,
                                  RejectionReason::kInvalidPnpInlierIndex);
                }
                hybrid_object_points.push_back(object_points[inlier_index]);
                hybrid_image_points.push_back(image_points[inlier_index]);
            }
            diagnostics.stage = Stage::kPnpRefinement;
            try {
                cv::solvePnPRefineLM(
                    hybrid_object_points, hybrid_image_points, camera_matrix,
                    cv::noArray(), rotation_vector, translation);
            } catch (const cv::Exception&) {
                return reject(Stage::kPnpRefinement,
                              RejectionReason::kPnpRefinementFailed);
            }
            cv::Mat refined_rotation_vector;
            cv::Mat refined_translation;
            if (!NormalizePoseVector(rotation_vector,
                                     &refined_rotation_vector) ||
                !NormalizePoseVector(translation, &refined_translation)) {
                return reject(Stage::kPnpRefinement,
                              RejectionReason::kInvalidRefinedPose);
            }
            rotation_vector = refined_rotation_vector;
            translation = refined_translation;
            try {
                cv::Rodrigues(rotation_vector, rotation_matrix);
                if (rotation_matrix.type() != CV_64F) {
                    rotation_matrix.convertTo(rotation_matrix, CV_64F);
                }
            } catch (const cv::Exception&) {
                return reject(Stage::kPnpRefinement,
                              RejectionReason::kInvalidRefinedPose);
            }
            if (!IsFiniteRotationMatrix(rotation_matrix)) {
                return reject(Stage::kPnpRefinement,
                              RejectionReason::kInvalidRefinedPose);
            }
            const RigidEstimate refined_pnp =
                RigidPoseFromOpenCv(rotation_matrix, translation);
            if (!PosesAgree(initial_rigid, refined_pnp) ||
                !PosesAgree(accepted_rigid, refined_pnp)) {
                return reject(Stage::kRigid3dRansac,
                              RejectionReason::kRigidPoseDisagreement);
            }

            std::vector<cv::Point2f> reprojected_points;
            try {
                cv::projectPoints(object_points, rotation_vector, translation,
                                  camera_matrix, cv::noArray(),
                                  reprojected_points);
            } catch (const cv::Exception&) {
                return reject(Stage::kReprojection,
                              RejectionReason::kReprojectionFailed);
            }
            if (reprojected_points.size() != image_points.size()) {
                return reject(Stage::kReprojection,
                              RejectionReason::kInvalidReprojectedPoints);
            }
            const double maximum_reprojection_error = std::min(
                options_.ransac_reprojection_error_pixels,
                kMaximumHybridReprojectionErrorPixels);
            const double maximum_squared_reprojection_error =
                maximum_reprojection_error * maximum_reprojection_error;
            std::vector<int> joint_inlier_indices;
            joint_inlier_indices.reserve(reprojected_points.size());
            for (int index = 0;
                 index < static_cast<int>(reprojected_points.size());
                 ++index) {
                if (static_cast<std::size_t>(index) >=
                        rigid_observations.size() ||
                    !rigid_observations[index].has_current_point ||
                    !IsFinitePoint(reprojected_points[index])) {
                    continue;
                }
                const cv::Point2f reprojection_residual =
                    reprojected_points[index] - image_points[index];
                const Eigen::Vector3d predicted_point =
                    refined_pnp.rotation *
                        rigid_observations[index].reference_point +
                    refined_pnp.translation;
                const double xyz_residual =
                    (predicted_point -
                     rigid_observations[index].current_point)
                        .norm();
                if (static_cast<double>(reprojection_residual.dot(
                        reprojection_residual)) >
                        maximum_squared_reprojection_error ||
                    !predicted_point.allFinite() ||
                    predicted_point.z() <= 1.0e-6 ||
                    !std::isfinite(xyz_residual) || xyz_residual >
                        RigidThreshold(rigid_observations[index])) {
                    continue;
                }
                joint_inlier_indices.push_back(index);
            }
            if (static_cast<int>(joint_inlier_indices.size()) <
                minimum_hybrid_inliers) {
                return reject(Stage::kPnpRansac,
                              RejectionReason::kInsufficientPnpInliers);
            }
            inlier_indices = std::move(joint_inlier_indices);
            used_rigid_recovery = false;
            pnp_already_refined = true;
        } else if (prefer_rigid_recovery && require_pnp_agreement) {
            cv::Mat normalized_rotation_vector;
            cv::Mat normalized_translation;
            if (!NormalizePoseVector(rotation_vector,
                                     &normalized_rotation_vector) ||
                !NormalizePoseVector(translation, &normalized_translation)) {
                return reject(Stage::kPnpRansac,
                              RejectionReason::kInvalidPnpPose);
            }
            cv::Mat pnp_rotation_matrix;
            try {
                cv::Rodrigues(normalized_rotation_vector,
                              pnp_rotation_matrix);
                if (pnp_rotation_matrix.type() != CV_64F) {
                    pnp_rotation_matrix.convertTo(pnp_rotation_matrix,
                                                  CV_64F);
                }
            } catch (const cv::Exception&) {
                return reject(Stage::kPnpRansac,
                              RejectionReason::kInvalidPnpPose);
            }
            if (!IsFiniteRotationMatrix(pnp_rotation_matrix) ||
                !PosesAgree(
                    accepted_rigid,
                    RigidPoseFromOpenCv(pnp_rotation_matrix,
                                        normalized_translation))) {
                return reject(Stage::kRigid3dRansac,
                              RejectionReason::kRigidPoseDisagreement);
            }
        }
        if (!use_guided_hybrid_pnp) {
            inlier_indices = accepted_rigid.inlier_indices;
            diagnostics.used_rigid_pose = true;
            CopyRigidPoseToOpenCv(accepted_rigid, &rotation_matrix,
                                 &translation);
            try {
                cv::Rodrigues(rotation_matrix, rotation_vector);
            } catch (const cv::Exception&) {
                return reject(Stage::kRigid3dRansac,
                              RejectionReason::kInvalidRotationMatrix);
            }
        }
    } else {
        cv::Mat normalized_rotation_vector;
        cv::Mat normalized_translation;
        if (!NormalizePoseVector(rotation_vector,
                                 &normalized_rotation_vector) ||
            !NormalizePoseVector(translation, &normalized_translation)) {
            return reject(Stage::kPnpRansac,
                          RejectionReason::kInvalidPnpPose);
        }
        rotation_vector = normalized_rotation_vector;
        translation = normalized_translation;
    }
    diagnostics.inlier_count = static_cast<int>(inlier_indices.size());
    const bool rigid_retry_available =
        !prefer_rigid_recovery && pnp_has_final_support &&
        (HasStrictRigidSupport(initial_rigid,
                               diagnostics.mutual_match_count,
                               options_.minimum_matches,
                               options_.minimum_inliers) ||
         (diagnostics.mutual_match_count >=
              kMinimumInitialProposalMatches &&
          static_cast<int>(initial_rigid.inlier_indices.size()) >=
              kMinimumRigidProposalInliers));
    const auto reject_pose_quality =
        [this, &frame, prefer_rigid_recovery, rigid_retry_available,
         &diagnostics, &reject](Stage stage, RejectionReason rejection) {
            if (!prefer_rigid_recovery && rigid_retry_available) {
                Result retry = EstimateInternal(frame, true, true);
                SetRetryDiagnostics(retry.diagnostics,
                                    &retry.diagnostics.rigid_retry);
                if (retry.success) {
                    return retry;
                }
                diagnostics.rigid_retry = retry.diagnostics.rigid_retry;
            }
            return reject(stage, rejection);
        };

    std::vector<PnpInlierPair> pnp_inlier_pairs;
    std::vector<cv::Point3f> inlier_object_points;
    std::vector<cv::Point2f> inlier_image_points;
    pnp_inlier_pairs.reserve(inlier_indices.size());
    inlier_object_points.reserve(inlier_indices.size());
    inlier_image_points.reserve(inlier_indices.size());
    for (int inlier_index : inlier_indices) {
        if (inlier_index < 0 ||
            static_cast<std::size_t>(inlier_index) >= object_points.size() ||
            static_cast<std::size_t>(inlier_index) >= image_points.size() ||
            static_cast<std::size_t>(inlier_index) >= match_pairs.size()) {
            return reject(Stage::kPnpRansac,
                          RejectionReason::kInvalidPnpInlierIndex);
        }
        const MatchPair& pair = match_pairs[inlier_index];
        if (pair.reference_index < 0 || pair.current_index < 0 ||
            static_cast<std::size_t>(pair.reference_index) >=
                reference_.original_feature_indices.size() ||
            pair.current_index >= frame.descriptors->rows ||
            static_cast<std::size_t>(pair.current_index) >=
                frame.keypoints->size()) {
            return reject(Stage::kPnpRansac,
                          RejectionReason::kInvalidPnpInlierIndex);
        }
        pnp_inlier_pairs.push_back(
            {reference_.original_feature_indices[pair.reference_index],
             pair.current_index});
        inlier_object_points.push_back(object_points[inlier_index]);
        inlier_image_points.push_back(image_points[inlier_index]);
    }
    result.pnp_inlier_pairs = std::move(pnp_inlier_pairs);
    if (!used_rigid_recovery && !pnp_already_refined) {
        diagnostics.stage = Stage::kPnpRefinement;
        try {
            cv::solvePnPRefineLM(inlier_object_points, inlier_image_points,
                                 camera_matrix, cv::noArray(), rotation_vector,
                                 translation);
        } catch (const cv::Exception&) {
            return reject(Stage::kPnpRefinement,
                          RejectionReason::kPnpRefinementFailed);
        }
        cv::Mat refined_rotation_vector;
        cv::Mat refined_translation;
        if (!NormalizePoseVector(rotation_vector, &refined_rotation_vector) ||
            !NormalizePoseVector(translation, &refined_translation)) {
            return reject(Stage::kPnpRefinement,
                          RejectionReason::kInvalidRefinedPose);
        }
        rotation_vector = refined_rotation_vector;
        translation = refined_translation;
    }

    diagnostics.stage = Stage::kSpatialCoverage;
    const int reference_grid_cells =
        CoveredGridCells(inlier_indices, match_pairs, reference_.keypoints,
                         true, reference_.minimum_x, reference_.maximum_x,
                         reference_.minimum_y, reference_.maximum_y);
    const int current_grid_cells = CoveredGridCells(
        inlier_indices, match_pairs, *frame.keypoints, false, frame.minimum_x,
        frame.maximum_x, frame.minimum_y, frame.maximum_y);
    diagnostics.covered_grid_cells =
        std::min(reference_grid_cells, current_grid_cells);
    const int minimum_accepted_grid_cells =
        diagnostics.used_pose_guided_matching
            ? std::max(options_.minimum_grid_cells,
                       internal::kRgbdTemporalStandardMinimumGridCells)
            : (options_.minimum_grid_cells >
                       internal::kRgbdTemporalStandardMinimumGridCells
                   ? options_.minimum_grid_cells
                   : internal::kRgbdTemporalFallbackMinimumGridCells);
    if (diagnostics.covered_grid_cells < minimum_accepted_grid_cells) {
        return reject_pose_quality(
            Stage::kSpatialCoverage,
            RejectionReason::kInsufficientGridCoverage);
    }

    if (rotation_matrix.empty()) {
        try {
            cv::Rodrigues(rotation_vector, rotation_matrix);
            if (rotation_matrix.type() != CV_64F) {
                rotation_matrix.convertTo(rotation_matrix, CV_64F);
            }
        } catch (const cv::Exception&) {
            return reject_pose_quality(
                Stage::kSpatialCoverage,
                RejectionReason::kInvalidRotationMatrix);
        }
    }
    if (!IsFiniteRotationMatrix(rotation_matrix)) {
        return reject_pose_quality(
            Stage::kSpatialCoverage,
            RejectionReason::kInvalidRotationMatrix);
    }

    diagnostics.stage = Stage::kDepthConsistency;
    if (!used_rigid_recovery &&
        HasStrictRigidSupport(initial_rigid,
                              diagnostics.mutual_match_count,
                              options_.minimum_matches,
                              options_.minimum_inliers) &&
        !PosesAgree(initial_rigid,
                    RigidPoseFromOpenCv(rotation_matrix, translation))) {
        return reject(Stage::kRigid3dRansac,
                      RejectionReason::kRigidPoseDisagreement);
    }
    double squared_xyz_error = 0.0;
    for (int inlier_index : inlier_indices) {
        const MatchPair& pair = match_pairs[inlier_index];
        const float measured_depth = (*frame.depths)[pair.current_index];
        if (!std::isfinite(measured_depth) || measured_depth <= 0.0F) {
            continue;
        }
        ++diagnostics.depth_check_count;
        const cv::Point3f& object_point = object_points[inlier_index];
        Eigen::Vector3d predicted_point;
        for (int row = 0; row < 3; ++row) {
            predicted_point(row) =
                rotation_matrix.at<double>(row, 0) * object_point.x +
                rotation_matrix.at<double>(row, 1) * object_point.y +
                rotation_matrix.at<double>(row, 2) * object_point.z +
                translation.at<double>(row);
        }
        const double predicted_depth = predicted_point.z();
        if (!std::isfinite(predicted_depth) || predicted_depth <= 0.0) {
            continue;
        }
        const double tolerance = std::max(
            static_cast<double>(options_.depth_absolute_tolerance_meters),
            static_cast<double>(options_.depth_relative_tolerance) *
                std::max(predicted_depth, static_cast<double>(measured_depth)));
        if (std::abs(predicted_depth - measured_depth) <= tolerance) {
            ++diagnostics.depth_consistent_count;
        }
        if (inlier_index < 0 ||
            static_cast<std::size_t>(inlier_index) >=
                rigid_observations.size() ||
            !rigid_observations[inlier_index].has_current_point ||
            !predicted_point.allFinite()) {
            continue;
        }
        const double xyz_residual =
            (predicted_point -
             rigid_observations[inlier_index].current_point)
                .norm();
        if (!std::isfinite(xyz_residual)) {
            continue;
        }
        ++diagnostics.xyz_check_count;
        squared_xyz_error += xyz_residual * xyz_residual;
        if (xyz_residual <= RigidThreshold(rigid_observations[inlier_index])) {
            ++diagnostics.xyz_consistent_count;
        }
    }
    if (diagnostics.depth_check_count < options_.minimum_depth_checks) {
        return reject_pose_quality(
            Stage::kDepthConsistency,
            RejectionReason::kInsufficientDepthChecks);
    }
    if (static_cast<double>(diagnostics.depth_check_count) /
            diagnostics.inlier_count <
        options_.minimum_depth_check_ratio) {
        return reject_pose_quality(
            Stage::kDepthConsistency,
            RejectionReason::kInsufficientDepthCheckRatio);
    }
    if (static_cast<double>(diagnostics.depth_consistent_count) /
            diagnostics.depth_check_count <
        options_.minimum_depth_consistency_ratio) {
        return reject_pose_quality(
            Stage::kDepthConsistency,
            RejectionReason::kInsufficientDepthConsistency);
    }
    const int minimum_xyz_checks =
        std::max(options_.minimum_depth_checks, kMinimumFinalXyzChecks);
    const double minimum_xyz_check_ratio =
        std::max(static_cast<double>(options_.minimum_depth_check_ratio),
                 kMinimumFinalXyzCheckRatio);
    const double minimum_xyz_consistency_ratio = std::max(
        static_cast<double>(options_.minimum_depth_consistency_ratio),
        kMinimumFinalXyzConsistencyRatio);
    if (diagnostics.xyz_check_count < minimum_xyz_checks ||
        static_cast<double>(diagnostics.xyz_check_count) /
                diagnostics.inlier_count <
            minimum_xyz_check_ratio ||
        static_cast<double>(diagnostics.xyz_consistent_count) /
                diagnostics.xyz_check_count <
            minimum_xyz_consistency_ratio) {
        return reject_pose_quality(
            Stage::kDepthConsistency,
            RejectionReason::kInsufficientXyzConsistency);
    }
    diagnostics.xyz_rmse_meters =
        std::sqrt(squared_xyz_error / diagnostics.xyz_check_count);
    if (!std::isfinite(diagnostics.xyz_rmse_meters) ||
        diagnostics.xyz_rmse_meters > kMaximumAcceptedXyzRmseMeters) {
        return reject_pose_quality(Stage::kDepthConsistency,
                                   RejectionReason::kExcessiveXyzError);
    }

    diagnostics.stage = Stage::kReprojection;
    std::vector<cv::Point2f> reprojected_points;
    try {
        cv::projectPoints(inlier_object_points, rotation_vector, translation,
                          camera_matrix, cv::noArray(), reprojected_points);
    } catch (const cv::Exception&) {
        return reject_pose_quality(Stage::kReprojection,
                                   RejectionReason::kReprojectionFailed);
    }
    if (reprojected_points.size() != inlier_image_points.size() ||
        reprojected_points.empty()) {
        return reject_pose_quality(
            Stage::kReprojection,
            RejectionReason::kInvalidReprojectedPoints);
    }
    for (const cv::Point2f& point : reprojected_points) {
        if (!IsFinitePoint(point)) {
            return reject_pose_quality(
                Stage::kReprojection,
                RejectionReason::kInvalidReprojectedPoints);
        }
    }
    double squared_reprojection_error = 0.0;
    for (std::size_t index = 0; index < reprojected_points.size(); ++index) {
        const cv::Point2f residual =
            reprojected_points[index] - inlier_image_points[index];
        squared_reprojection_error += residual.dot(residual);
    }
    diagnostics.reprojection_rmse_pixels =
        std::sqrt(squared_reprojection_error / reprojected_points.size());
    if (!std::isfinite(diagnostics.reprojection_rmse_pixels) ||
        diagnostics.reprojection_rmse_pixels >
            options_.maximum_reprojection_rmse_pixels) {
        return reject_pose_quality(
            Stage::kReprojection,
            RejectionReason::kExcessiveReprojectionError);
    }

    diagnostics.stage = Stage::kConditioning;
    diagnostics.pose_condition_number =
        PoseConditionNumber(object_points, inlier_indices, rotation_matrix,
                            translation, frame.intrinsics);
    if (!std::isfinite(diagnostics.pose_condition_number) ||
        diagnostics.pose_condition_number >
            options_.maximum_pose_condition_number) {
        return reject_pose_quality(Stage::kConditioning,
                                   RejectionReason::kIllConditionedPose);
    }
    const bool low_coverage = diagnostics.covered_grid_cells <
                              internal::kRgbdTemporalStandardMinimumGridCells;
    if (low_coverage && !(diagnostics.mutual_match_count >= 40 &&
                          diagnostics.inlier_count >= 20 &&
                          diagnostics.inlier_count * 100 >=
                              diagnostics.mutual_match_count * 45 &&
                          diagnostics.depth_check_count * 100 >=
                              diagnostics.inlier_count * 90 &&
                          diagnostics.depth_consistent_count * 100 >=
                              diagnostics.depth_check_count * 85 &&
                          diagnostics.xyz_consistent_count * 100 >=
                              diagnostics.xyz_check_count * 85 &&
                          diagnostics.xyz_rmse_meters <=
                              kMaximumLowCoverageXyzRmseMeters &&
                          diagnostics.reprojection_rmse_pixels <= 1.75 &&
                          diagnostics.pose_condition_number <= 1.0e3)) {
        return reject_pose_quality(
            Stage::kSpatialCoverage,
            RejectionReason::kLowCoverageConfidenceFailed);
    }

    diagnostics.stage = Stage::kPoseComposition;
    const autonomy::common::math::SE3f current_from_reference =
        ToSe3(rotation_matrix, translation);
    result.camera_from_world =
        current_from_reference * reference_.camera_from_world;
    result.success = result.camera_from_world.translation().allFinite() &&
                     result.camera_from_world.rotationMatrix().allFinite();
    if (!result.success) {
        return reject_pose_quality(Stage::kPoseComposition,
                                   RejectionReason::kInvalidWorldPose);
    }
    result.quality =
        (low_coverage || diagnostics.used_pose_guided_matching)
            ? Quality::kLowCoverageFallback
            : Quality::kStandard;
    diagnostics.stage = Stage::kComplete;
    diagnostics.rejection = RejectionReason::kNone;
    return result;
}

void RgbdTemporalPoseEstimator::Reset() {
    reference_ = Reference();
    has_reference_ = false;
}

bool RgbdTemporalPoseEstimator::HasReference() const {
    return has_reference_;
}

}  // namespace autonomy::localization::atlas
