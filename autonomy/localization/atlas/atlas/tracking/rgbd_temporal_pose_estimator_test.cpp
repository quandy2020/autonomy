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

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <Eigen/Geometry>

namespace autonomy::localization::atlas {
namespace {

struct SyntheticFrame {
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> keypoints;
    std::vector<float> depths;
    Eigen::Matrix3f intrinsics = Eigen::Matrix3f::Identity();
    double timestamp = 0.0;
    std::uint64_t frame_id = 0;
    std::uint64_t map_id = 7;
    std::uint64_t coordinate_epoch = 3;
    int map_change_index = 5;

    RgbdTemporalPoseEstimator::FrameView View() const {
        RgbdTemporalPoseEstimator::FrameView view;
        view.descriptors = &descriptors;
        view.keypoints = &keypoints;
        view.depths = &depths;
        view.intrinsics = intrinsics;
        view.minimum_x = 0.0F;
        view.maximum_x = 640.0F;
        view.minimum_y = 0.0F;
        view.maximum_y = 480.0F;
        view.timestamp = timestamp;
        view.frame_id = frame_id;
        view.map_id = map_id;
        view.coordinate_epoch = coordinate_epoch;
        view.map_change_index = map_change_index;
        return view;
    }
};

cv::Mat MakeDescriptors(int count) {
    cv::Mat descriptors = cv::Mat::zeros(count, 32, CV_8U);
    for (int row = 0; row < count; ++row) {
        descriptors.at<std::uint8_t>(row, 0) = static_cast<std::uint8_t>(row);
        descriptors.at<std::uint8_t>(row, 1) =
            static_cast<std::uint8_t>(37 * row + 11);
        descriptors.at<std::uint8_t>(row, 2) =
            static_cast<std::uint8_t>(101 * row + 3);
    }
    return descriptors;
}

std::pair<SyntheticFrame, SyntheticFrame> MakeSyntheticPair(
    const autonomy::common::math::SE3f& current_from_reference,
    bool clustered = false, bool five_cell_coverage = false) {
    SyntheticFrame reference;
    SyntheticFrame current;
    reference.timestamp = 1.0;
    reference.frame_id = 10;
    current.timestamp = 1.1;
    current.frame_id = 11;
    reference.intrinsics << 500.0F, 0.0F, 320.0F, 0.0F, 500.0F, 240.0F, 0.0F,
        0.0F, 1.0F;
    current.intrinsics = reference.intrinsics;

    constexpr int kColumns = 7;
    constexpr int kRows = 7;
    constexpr int kPointCount = kColumns * kRows;
    constexpr float kFiveCellU[] = {80.0F, 560.0F, 80.0F, 560.0F, 240.0F};
    constexpr float kFiveCellV[] = {60.0F, 60.0F, 420.0F, 420.0F, 180.0F};
    reference.descriptors = MakeDescriptors(kPointCount);
    current.descriptors = reference.descriptors.clone();
    reference.keypoints.reserve(kPointCount);
    current.keypoints.reserve(kPointCount);
    reference.depths.reserve(kPointCount);
    current.depths.reserve(kPointCount);

    for (int row = 0; row < kRows; ++row) {
        for (int column = 0; column < kColumns; ++column) {
            float u =
                clustered ? 260.0F + 8.0F * column : 80.0F + 80.0F * column;
            float v = clustered ? 200.0F + 6.0F * row : 60.0F + 60.0F * row;
            if (five_cell_coverage) {
                const int cell = (row * kColumns + column) % 5;
                u = kFiveCellU[cell] + 2.0F * (column - 3);
                v = kFiveCellV[cell] + 2.0F * (row - 3);
            }
            const float depth = 2.0F + 0.07F * row + 0.05F * column;
            const Eigen::Vector3f reference_point(
                (u - reference.intrinsics(0, 2)) * depth /
                    reference.intrinsics(0, 0),
                (v - reference.intrinsics(1, 2)) * depth /
                    reference.intrinsics(1, 1),
                depth);
            const Eigen::Vector3f current_point =
                current_from_reference * reference_point;
            const float current_u = current.intrinsics(0, 0) *
                                        current_point.x() / current_point.z() +
                                    current.intrinsics(0, 2);
            const float current_v = current.intrinsics(1, 1) *
                                        current_point.y() / current_point.z() +
                                    current.intrinsics(1, 2);

            reference.keypoints.emplace_back(cv::Point2f(u, v), 1.0F);
            current.keypoints.emplace_back(cv::Point2f(current_u, current_v),
                                           1.0F);
            reference.depths.push_back(depth);
            current.depths.push_back(current_point.z());
        }
    }
    return {std::move(reference), std::move(current)};
}

std::pair<SyntheticFrame, SyntheticFrame> MakeGuidedRecoveryPair(
    const autonomy::common::math::SE3f& current_from_reference,
    int initial_mutual_matches = 13) {
    auto pair = MakeSyntheticPair(current_from_reference);
    SyntheticFrame& reference = pair.first;
    SyntheticFrame& current = pair.second;
    const int original_feature_count = current.descriptors.rows;
    for (int index = initial_mutual_matches; index < original_feature_count;
         ++index) {
        // A far exact duplicate deliberately defeats global ratio matching.
        current.descriptors.push_back(reference.descriptors.row(index));
        current.keypoints.emplace_back(
            cv::Point2f(8.0F + index % 4, 8.0F + index % 5), 1.0F);
        current.depths.push_back(2.0F);

        // A nearby appearance-negative feature provides the second local
        // candidate required by the unchanged 0.8 ratio test.
        cv::Mat local_decoy = reference.descriptors.row(index).clone();
        cv::bitwise_not(local_decoy, local_decoy);
        current.descriptors.push_back(local_decoy);
        current.keypoints.emplace_back(
            current.keypoints[index].pt + cv::Point2f(4.0F, 0.0F), 1.0F);
        current.depths.push_back(current.depths[index]);
    }
    return pair;
}

TEST(RgbdTemporalPoseEstimatorTest, RecoversComposedCameraPose) {
    const Eigen::Matrix3f relative_rotation =
        Eigen::AngleAxisf(0.03F, Eigen::Vector3f::UnitY()).toRotationMatrix();
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.18F, -0.02F, 0.04F), relative_rotation);
    auto [reference, current] = MakeSyntheticPair(current_from_reference);

    const Eigen::Matrix3f reference_rotation =
        Eigen::AngleAxisf(-0.08F, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    const autonomy::common::math::SE3f reference_from_world(
        Eigen::Vector3f(1.0F, -0.2F, 0.3F), reference_rotation);
    const autonomy::common::math::SE3f expected =
        current_from_reference * reference_from_world;

    RgbdTemporalPoseEstimator estimator;
    ASSERT_TRUE(
        estimator.UpdateReference(reference.View(), reference_from_world));
    const RgbdTemporalPoseEstimator::Result result =
        estimator.Estimate(current.View());

    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.quality, RgbdTemporalPoseEstimator::Quality::kStandard);
    EXPECT_EQ(result.diagnostics.stage,
              RgbdTemporalPoseEstimator::Stage::kComplete);
    EXPECT_EQ(result.diagnostics.rejection,
              RgbdTemporalPoseEstimator::RejectionReason::kNone);
    EXPECT_GE(result.diagnostics.inlier_count, 40);
    EXPECT_GE(result.diagnostics.covered_grid_cells, 6);
    EXPECT_EQ(result.diagnostics.reference_frame_id, reference.frame_id);
    EXPECT_EQ(result.diagnostics.current_frame_id, current.frame_id);
    ASSERT_EQ(result.pnp_inlier_pairs.size(),
              static_cast<std::size_t>(result.diagnostics.inlier_count));
    EXPECT_EQ(result.pnp_inlier_pairs.front().reference_original_index,
              result.pnp_inlier_pairs.front().current_index);
    EXPECT_LT(result.diagnostics.reprojection_rmse_pixels, 0.1);
    EXPECT_EQ(result.diagnostics.xyz_check_count, 49);
    EXPECT_EQ(result.diagnostics.xyz_consistent_count, 49);
    EXPECT_LT(result.diagnostics.xyz_rmse_meters, 1.0e-3);
    EXPECT_TRUE(result.camera_from_world.translation().isApprox(
        expected.translation(), 1.0e-3F));
    EXPECT_TRUE(result.camera_from_world.rotationMatrix().isApprox(
        expected.rotationMatrix(), 1.0e-3F));
}

TEST(RgbdTemporalPoseEstimatorTest,
     PrefersSupportedPnpWhenRigidPoseAlsoPassesStrictGates) {
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.1F, 0.0F, 0.0F), Eigen::Matrix3f::Identity());
    auto [reference, current] = MakeSyntheticPair(current_from_reference);
    for (float& depth : current.depths) {
        depth += 0.05F;
    }
    RgbdTemporalPoseEstimator estimator;
    ASSERT_TRUE(estimator.UpdateReference(reference.View(),
                                          autonomy::common::math::SE3f()));

    const auto result = estimator.Estimate(current.View());
    ASSERT_TRUE(result.success);
    EXPECT_GE(result.diagnostics.rigid_depth_pair_count, 30);
    EXPECT_GE(result.diagnostics.rigid_inlier_count, 20);
    EXPECT_GE(result.diagnostics.rigid_inlier_ratio, 0.60);
    EXPECT_LE(result.diagnostics.rigid_rmse_meters, 0.06);
    EXPECT_LE(result.diagnostics.rigid_xyz_median_meters, 0.04);
    EXPECT_FALSE(result.diagnostics.used_rigid_pose);
    EXPECT_FALSE(result.diagnostics.used_pose_guided_matching);
    EXPECT_FALSE(result.diagnostics.rigid_retry.attempted);
    EXPECT_GT(result.diagnostics.xyz_rmse_meters, 0.04);
    EXPECT_LT(result.diagnostics.reprojection_rmse_pixels, 0.1);
    EXPECT_TRUE(result.camera_from_world.translation().isApprox(
        current_from_reference.translation(), 1.0e-3F));
    EXPECT_TRUE(result.camera_from_world.rotationMatrix().isApprox(
        current_from_reference.rotationMatrix(), 1.0e-3F));
}

TEST(RgbdTemporalPoseEstimatorTest,
     RejectsConflictingStrictRigidAndPnpPoses) {
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.1F, 0.0F, 0.0F), Eigen::Matrix3f::Identity());
    auto [reference, current] = MakeSyntheticPair(current_from_reference);
    for (float& depth : current.depths) {
        depth += 0.104F;
    }
    RgbdTemporalPoseEstimator estimator;
    ASSERT_TRUE(estimator.UpdateReference(reference.View(),
                                          autonomy::common::math::SE3f()));

    const auto result = estimator.Estimate(current.View());
    EXPECT_FALSE(result.success);
    EXPECT_GE(result.diagnostics.rigid_depth_pair_count, 30);
    EXPECT_GE(result.diagnostics.rigid_inlier_count, 20);
    EXPECT_GE(result.diagnostics.rigid_inlier_ratio, 0.60);
    EXPECT_LE(result.diagnostics.rigid_rmse_meters, 0.06);
    EXPECT_LE(result.diagnostics.rigid_xyz_median_meters, 0.04);
    EXPECT_EQ(result.diagnostics.stage,
              RgbdTemporalPoseEstimator::Stage::kRigid3dRansac);
    EXPECT_EQ(result.diagnostics.rejection,
              RgbdTemporalPoseEstimator::RejectionReason::
                  kRigidPoseDisagreement);
}

TEST(RgbdTemporalPoseEstimatorTest,
     PoseGuidedMatchingRequiresStrictFinalRigidSupport) {
    const Eigen::Matrix3f rotation =
        Eigen::AngleAxisf(0.02F, Eigen::Vector3f::UnitY()).toRotationMatrix();
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.12F, -0.01F, 0.03F), rotation);
    auto [reference, current] =
        MakeGuidedRecoveryPair(current_from_reference);
    RgbdTemporalPoseEstimator estimator;
    ASSERT_TRUE(estimator.UpdateReference(reference.View(),
                                          autonomy::common::math::SE3f()));

    const auto first = estimator.Estimate(current.View());
    ASSERT_TRUE(first.success);
    EXPECT_EQ(first.quality,
              RgbdTemporalPoseEstimator::Quality::kLowCoverageFallback);
    EXPECT_TRUE(first.diagnostics.used_pose_guided_matching);
    EXPECT_EQ(first.diagnostics.mutual_match_count, 13);
    EXPECT_GT(first.diagnostics.guided_match_count, 0);
    EXPECT_LE(first.diagnostics.guided_match_count, 64);
    EXPECT_GE(first.diagnostics.final_match_count, 30);
    EXPECT_GE(first.diagnostics.rigid_depth_pair_count, 30);
    EXPECT_GE(first.diagnostics.rigid_inlier_count, 20);
    EXPECT_GE(first.diagnostics.rigid_inlier_ratio, 0.60);
    EXPECT_LE(first.diagnostics.rigid_rmse_meters, 0.06);
    EXPECT_LE(first.diagnostics.rigid_xyz_median_meters, 0.04);
    EXPECT_TRUE(first.diagnostics.used_rigid_pose);
    EXPECT_GE(first.diagnostics.rigid_five_centimeter_inlier_count, 20);
    EXPECT_GE(first.diagnostics.covered_grid_cells, 6);
    EXPECT_GE(first.diagnostics.xyz_check_count, 20);
    EXPECT_LE(first.diagnostics.xyz_rmse_meters, 0.15);
    EXPECT_TRUE(first.camera_from_world.translation().isApprox(
        current_from_reference.translation(), 1.0e-3F));
    EXPECT_TRUE(first.camera_from_world.rotationMatrix().isApprox(
        current_from_reference.rotationMatrix(), 1.0e-3F));
    for (const auto& inlier : first.pnp_inlier_pairs) {
        EXPECT_EQ(inlier.reference_original_index, inlier.current_index);
    }

    const auto second = estimator.Estimate(current.View());
    ASSERT_TRUE(second.success);
    EXPECT_EQ(second.diagnostics.guided_match_count,
              first.diagnostics.guided_match_count);
    EXPECT_EQ(second.diagnostics.rigid_inlier_count,
              first.diagnostics.rigid_inlier_count);
    EXPECT_EQ(second.pnp_inlier_pairs.size(), first.pnp_inlier_pairs.size());
    EXPECT_TRUE(second.camera_from_world.translation().isApprox(
        first.camera_from_world.translation(), 1.0e-6F));
    EXPECT_TRUE(second.camera_from_world.rotationMatrix().isApprox(
        first.camera_from_world.rotationMatrix(), 1.0e-6F));
}

TEST(RgbdTemporalPoseEstimatorTest,
     RetriesRigidRecoveryAfterSupportedPnpFailsDepthValidation) {
    const Eigen::Matrix3f rotation =
        Eigen::AngleAxisf(0.02F, Eigen::Vector3f::UnitY()).toRotationMatrix();
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.12F, -0.01F, 0.03F), rotation);
    auto [reference, current] =
        MakeGuidedRecoveryPair(current_from_reference, 30);

    // PnP sees 30 geometrically correct 2D correspondences, but eleven of
    // their measured depths are incompatible with that pose. The remaining
    // nineteen rigid inliers form a valid proposal, but cannot pass the
    // unchanged final requirement of twenty <=5 cm inliers without guided
    // recovery.
    for (int index = 0; index < 11; ++index) {
        current.depths[index] += 0.5F;
    }

    RgbdTemporalPoseEstimator estimator;
    ASSERT_TRUE(estimator.UpdateReference(reference.View(),
                                          autonomy::common::math::SE3f()));

    const auto result = estimator.Estimate(current.View());
    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.quality,
              RgbdTemporalPoseEstimator::Quality::kLowCoverageFallback);
    EXPECT_GE(result.diagnostics.pnp_inlier_count, 30);
    EXPECT_EQ(result.diagnostics.mutual_match_count, 30);
    EXPECT_TRUE(result.diagnostics.used_pose_guided_matching);
    EXPECT_FALSE(result.diagnostics.used_rigid_pose);
    EXPECT_TRUE(result.diagnostics.rigid_retry.attempted);
    EXPECT_EQ(result.diagnostics.rigid_retry.stage,
              RgbdTemporalPoseEstimator::Stage::kComplete);
    EXPECT_EQ(result.diagnostics.rigid_retry.rejection,
              RgbdTemporalPoseEstimator::RejectionReason::kNone);
    EXPECT_EQ(result.diagnostics.rigid_retry.guided_match_count,
              result.diagnostics.guided_match_count);
    EXPECT_EQ(result.diagnostics.rigid_retry.final_match_count,
              result.diagnostics.final_match_count);
    EXPECT_EQ(result.diagnostics.rigid_retry.rigid_depth_pair_count,
              result.diagnostics.rigid_depth_pair_count);
    EXPECT_EQ(result.diagnostics.rigid_retry.rigid_inlier_count,
              result.diagnostics.rigid_inlier_count);
    EXPECT_EQ(
        result.diagnostics.rigid_retry.rigid_five_centimeter_inlier_count,
        result.diagnostics.rigid_five_centimeter_inlier_count);
    EXPECT_DOUBLE_EQ(result.diagnostics.rigid_retry.rigid_inlier_ratio,
                     result.diagnostics.rigid_inlier_ratio);
    EXPECT_DOUBLE_EQ(result.diagnostics.rigid_retry.rigid_rmse_meters,
                     result.diagnostics.rigid_rmse_meters);
    EXPECT_DOUBLE_EQ(result.diagnostics.rigid_retry.rigid_xyz_median_meters,
                     result.diagnostics.rigid_xyz_median_meters);
    EXPECT_EQ(result.diagnostics.rigid_retry.covered_grid_cells,
              result.diagnostics.covered_grid_cells);
    EXPECT_EQ(result.diagnostics.rigid_retry.depth_check_count,
              result.diagnostics.depth_check_count);
    EXPECT_EQ(result.diagnostics.rigid_retry.depth_consistent_count,
              result.diagnostics.depth_consistent_count);
    EXPECT_EQ(result.diagnostics.rigid_retry.xyz_check_count,
              result.diagnostics.xyz_check_count);
    EXPECT_EQ(result.diagnostics.rigid_retry.xyz_consistent_count,
              result.diagnostics.xyz_consistent_count);
    EXPECT_DOUBLE_EQ(result.diagnostics.rigid_retry.xyz_rmse_meters,
                     result.diagnostics.xyz_rmse_meters);
    EXPECT_DOUBLE_EQ(
        result.diagnostics.rigid_retry.reprojection_rmse_pixels,
        result.diagnostics.reprojection_rmse_pixels);
    EXPECT_DOUBLE_EQ(result.diagnostics.rigid_retry.pose_condition_number,
                     result.diagnostics.pose_condition_number);
    EXPECT_GT(result.diagnostics.guided_match_count, 0);
    EXPECT_GE(result.diagnostics.final_match_count, 30);
    EXPECT_GE(result.diagnostics.rigid_depth_pair_count, 30);
    EXPECT_GE(result.diagnostics.rigid_inlier_count, 20);
    EXPECT_GE(result.diagnostics.rigid_five_centimeter_inlier_count, 20);
    EXPECT_GE(result.diagnostics.rigid_inlier_ratio, 0.60);
    EXPECT_LE(result.diagnostics.rigid_rmse_meters, 0.06);
    EXPECT_LE(result.diagnostics.rigid_xyz_median_meters, 0.04);
    EXPECT_GE(result.diagnostics.covered_grid_cells, 6);
    EXPECT_GE(result.diagnostics.inlier_count, 20);
    EXPECT_EQ(result.pnp_inlier_pairs.size(),
              static_cast<std::size_t>(result.diagnostics.inlier_count));
    EXPECT_GE(result.diagnostics.xyz_check_count, 20);
    EXPECT_LE(result.diagnostics.xyz_rmse_meters, 0.15);
    EXPECT_TRUE(result.camera_from_world.translation().isApprox(
        current_from_reference.translation(), 1.0e-3F));
    EXPECT_TRUE(result.camera_from_world.rotationMatrix().isApprox(
        current_from_reference.rotationMatrix(), 1.0e-3F));
}

TEST(RgbdTemporalPoseEstimatorTest,
     SupportedPnpRetryRejectsOnlyTwentyNineFinalDepthPairs) {
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.12F, -0.01F, 0.03F),
        Eigen::Matrix3f::Identity());
    auto [reference, current] =
        MakeGuidedRecoveryPair(current_from_reference, 30);
    for (int index = 0; index < 11; ++index) {
        current.depths[index] += 0.5F;
    }
    for (int index = 29; index < 49; ++index) {
        current.depths[index] = -1.0F;
    }

    RgbdTemporalPoseEstimator estimator;
    ASSERT_TRUE(estimator.UpdateReference(reference.View(),
                                          autonomy::common::math::SE3f()));

    const auto result = estimator.Estimate(current.View());
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.diagnostics.pnp_inlier_count, 30);
    EXPECT_EQ(result.diagnostics.rigid_depth_pair_count, 29);
    EXPECT_EQ(result.diagnostics.rejection,
              RgbdTemporalPoseEstimator::RejectionReason::
                  kInsufficientDepthConsistency);
    ASSERT_TRUE(result.diagnostics.rigid_retry.attempted);
    EXPECT_EQ(result.diagnostics.rigid_retry.stage,
              RgbdTemporalPoseEstimator::Stage::kRigid3dRansac);
    EXPECT_EQ(result.diagnostics.rigid_retry.rejection,
              RgbdTemporalPoseEstimator::RejectionReason::
                  kRigid3dValidationFailed);
    EXPECT_EQ(result.diagnostics.rigid_retry.final_match_count, 30);
    EXPECT_EQ(result.diagnostics.rigid_retry.rigid_depth_pair_count, 29);
    EXPECT_LT(result.diagnostics.rigid_retry.rigid_inlier_count, 20);
    EXPECT_EQ(result.diagnostics.rigid_retry.covered_grid_cells, 0);
    EXPECT_EQ(result.diagnostics.rigid_retry.depth_check_count, 0);
    EXPECT_EQ(result.diagnostics.rigid_retry.xyz_check_count, 0);
}

TEST(RgbdTemporalPoseEstimatorTest,
     GuidedHybridJointSupportCapsReprojectionAtThreePixels) {
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.12F, -0.01F, 0.03F),
        Eigen::Matrix3f::Identity());
    auto [reference, current] =
        MakeGuidedRecoveryPair(current_from_reference, 30);
    for (int index = 0; index < 11; ++index) {
        current.depths[index] += 0.5F;
    }
    constexpr int kFourPixelResidualIndex = 48;
    current.keypoints[kFourPixelResidualIndex].pt.x += 4.0F;

    RgbdTemporalPoseEstimator::Options options;
    options.ransac_reprojection_error_pixels = 6.0F;
    RgbdTemporalPoseEstimator estimator(options);
    ASSERT_TRUE(estimator.UpdateReference(reference.View(),
                                          autonomy::common::math::SE3f()));

    const auto result = estimator.Estimate(current.View());
    ASSERT_TRUE(result.success);
    EXPECT_TRUE(result.diagnostics.rigid_retry.attempted);
    EXPECT_TRUE(result.diagnostics.used_pose_guided_matching);
    EXPECT_FALSE(result.diagnostics.used_rigid_pose);
    EXPECT_GE(result.diagnostics.covered_grid_cells, 6);
    EXPECT_LT(result.diagnostics.reprojection_rmse_pixels, 2.5);
    bool retained_four_pixel_residual = false;
    for (const auto& pair : result.pnp_inlier_pairs) {
        retained_four_pixel_residual |=
            pair.current_index == kFourPixelResidualIndex;
    }
    EXPECT_FALSE(retained_four_pixel_residual);
}

TEST(RgbdTemporalPoseEstimatorTest,
     RejectsGuidedRecoveryWithOnlyTwentyNineDepthPairs) {
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.12F, -0.01F, 0.03F),
        Eigen::Matrix3f::Identity());
    auto [reference, current] =
        MakeGuidedRecoveryPair(current_from_reference);
    for (int index = 29; index < 49; ++index) {
        current.depths[index] = -1.0F;
    }
    RgbdTemporalPoseEstimator::Options options;
    options.minimum_matches = 14;
    options.minimum_inliers = 10;
    options.minimum_depth_checks = 1;
    options.minimum_depth_check_ratio = 0.01F;
    options.minimum_depth_consistency_ratio = 0.01F;
    RgbdTemporalPoseEstimator estimator(options);
    ASSERT_TRUE(estimator.UpdateReference(reference.View(),
                                          autonomy::common::math::SE3f()));

    const auto result = estimator.Estimate(current.View());
    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.diagnostics.used_pose_guided_matching);
    EXPECT_EQ(result.diagnostics.rigid_depth_pair_count, 29);
    EXPECT_GE(result.diagnostics.rigid_inlier_count, 20);
    EXPECT_EQ(result.diagnostics.rejection,
              RgbdTemporalPoseEstimator::RejectionReason::
                  kRigid3dValidationFailed);
}

TEST(RgbdTemporalPoseEstimatorTest,
     ProposalThresholdDoesNotPreemptConfiguredPnp) {
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.03F, 0.0F, 0.0F), Eigen::Matrix3f::Identity());
    auto [reference, current] = MakeSyntheticPair(current_from_reference);
    constexpr int kSelectedIndices[] = {0, 2, 4, 6, 42, 44, 46, 48};
    cv::Mat reference_descriptors;
    cv::Mat current_descriptors;
    std::vector<cv::KeyPoint> reference_keypoints;
    std::vector<cv::KeyPoint> current_keypoints;
    std::vector<float> reference_depths;
    std::vector<float> current_depths;
    for (int index : kSelectedIndices) {
        reference_descriptors.push_back(reference.descriptors.row(index));
        current_descriptors.push_back(current.descriptors.row(index));
        reference_keypoints.push_back(reference.keypoints[index]);
        current_keypoints.push_back(current.keypoints[index]);
        reference_depths.push_back(reference.depths[index]);
        current_depths.push_back(current.depths[index]);
    }
    reference.descriptors = std::move(reference_descriptors);
    current.descriptors = std::move(current_descriptors);
    reference.keypoints = std::move(reference_keypoints);
    current.keypoints = std::move(current_keypoints);
    reference.depths = std::move(reference_depths);
    current.depths = std::move(current_depths);

    RgbdTemporalPoseEstimator::Options options;
    options.minimum_matches = 4;
    options.minimum_inliers = 4;
    options.minimum_depth_checks = 1;
    options.minimum_depth_check_ratio = 0.01F;
    options.minimum_depth_consistency_ratio = 0.01F;
    RgbdTemporalPoseEstimator estimator(options);
    ASSERT_TRUE(estimator.UpdateReference(reference.View(),
                                          autonomy::common::math::SE3f()));

    const auto result = estimator.Estimate(current.View());
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.diagnostics.mutual_match_count, 8);
    EXPECT_GE(result.diagnostics.inlier_count, 4);
    EXPECT_EQ(result.diagnostics.rejection,
              RgbdTemporalPoseEstimator::RejectionReason::
                  kInsufficientXyzConsistency);
}

TEST(RgbdTemporalPoseEstimatorTest, GatesFiveCellFallbackByConfidence) {
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.03F, 0.0F, 0.0F), Eigen::Matrix3f::Identity());
    auto [reference, current] =
        MakeSyntheticPair(current_from_reference, false, true);
    RgbdTemporalPoseEstimator estimator;
    ASSERT_TRUE(estimator.UpdateReference(reference.View(),
                                          autonomy::common::math::SE3f()));

    const auto accepted = estimator.Estimate(current.View());
    ASSERT_TRUE(accepted.success);
    EXPECT_EQ(accepted.diagnostics.covered_grid_cells, 5);
    EXPECT_EQ(accepted.quality,
              RgbdTemporalPoseEstimator::Quality::kLowCoverageFallback);

    current.descriptors = current.descriptors.rowRange(0, 39).clone();
    current.keypoints.resize(39);
    current.depths.resize(39);
    const auto rejected = estimator.Estimate(current.View());
    EXPECT_FALSE(rejected.success);
    EXPECT_EQ(rejected.diagnostics.rejection,
              RgbdTemporalPoseEstimator::RejectionReason::
                  kLowCoverageConfidenceFailed);
}

TEST(RgbdTemporalPoseEstimatorTest, RejectsCoordinateEpochMismatch) {
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.1F, 0.0F, 0.0F), Eigen::Matrix3f::Identity());
    auto [reference, current] = MakeSyntheticPair(current_from_reference);
    RgbdTemporalPoseEstimator estimator;
    ASSERT_TRUE(estimator.UpdateReference(reference.View(),
                                          autonomy::common::math::SE3f()));

    ++current.coordinate_epoch;
    const auto result = estimator.Estimate(current.View());
    EXPECT_FALSE(result.success);
    EXPECT_EQ(
        result.diagnostics.rejection,
        RgbdTemporalPoseEstimator::RejectionReason::kCoordinateEpochMismatch);
    EXPECT_EQ(result.diagnostics.reference_coordinate_epoch,
              reference.coordinate_epoch);
    EXPECT_EQ(result.diagnostics.current_coordinate_epoch,
              current.coordinate_epoch);
}

TEST(RgbdTemporalPoseEstimatorTest, RejectsMissingReferenceWithDiagnostics) {
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.1F, 0.0F, 0.0F), Eigen::Matrix3f::Identity());
    auto [reference, current] = MakeSyntheticPair(current_from_reference);
    (void)reference;
    RgbdTemporalPoseEstimator estimator;

    const auto result = estimator.Estimate(current.View());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.diagnostics.stage,
              RgbdTemporalPoseEstimator::Stage::kInputValidation);
    EXPECT_EQ(
        result.diagnostics.rejection,
        RgbdTemporalPoseEstimator::RejectionReason::kReferenceUnavailable);
    EXPECT_FALSE(result.diagnostics.reference_available);
    EXPECT_EQ(result.diagnostics.current_feature_count,
              current.descriptors.rows);
}

TEST(RgbdTemporalPoseEstimatorTest, ReportsInsufficientMutualMatches) {
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.1F, 0.0F, 0.0F), Eigen::Matrix3f::Identity());
    auto [reference, current] = MakeSyntheticPair(current_from_reference);
    RgbdTemporalPoseEstimator estimator;
    ASSERT_TRUE(estimator.UpdateReference(reference.View(),
                                          autonomy::common::math::SE3f()));
    current.descriptors = cv::Mat::zeros(current.descriptors.size(), CV_8U);

    const auto result = estimator.Estimate(current.View());

    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.diagnostics.stage,
              RgbdTemporalPoseEstimator::Stage::kDescriptorMatching);
    EXPECT_EQ(
        result.diagnostics.rejection,
        RgbdTemporalPoseEstimator::RejectionReason::kInsufficientMutualMatches);
    EXPECT_TRUE(result.pnp_inlier_pairs.empty());
}

TEST(RgbdTemporalPoseEstimatorTest, RejectsMapChangeMismatch) {
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.1F, 0.0F, 0.0F), Eigen::Matrix3f::Identity());
    auto [reference, current] = MakeSyntheticPair(current_from_reference);
    RgbdTemporalPoseEstimator estimator;
    ASSERT_TRUE(estimator.UpdateReference(reference.View(),
                                          autonomy::common::math::SE3f()));

    ++current.map_change_index;
    const auto result = estimator.Estimate(current.View());
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.diagnostics.rejection,
              RgbdTemporalPoseEstimator::RejectionReason::kMapRevisionMismatch);
    EXPECT_EQ(result.diagnostics.reference_map_revision,
              reference.map_change_index);
    EXPECT_EQ(result.diagnostics.current_map_revision,
              current.map_change_index);
}

TEST(RgbdTemporalPoseEstimatorTest,
     RebasesReferencePoseAcrossMonotonicMapRevision) {
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.1F, 0.0F, 0.0F), Eigen::Matrix3f::Identity());
    auto [reference, current] = MakeSyntheticPair(current_from_reference);
    const autonomy::common::math::SE3f reference_from_world(
        Eigen::Vector3f(1.0F, -0.2F, 0.3F),
        Eigen::AngleAxisf(-0.08F, Eigen::Vector3f::UnitZ()).toRotationMatrix());
    const autonomy::common::math::SE3f corrected_reference_from_world(
        Eigen::Vector3f(1.2F, -0.1F, 0.35F),
        Eigen::AngleAxisf(-0.05F, Eigen::Vector3f::UnitZ()).toRotationMatrix());
    RgbdTemporalPoseEstimator estimator;
    ASSERT_TRUE(
        estimator.UpdateReference(reference.View(), reference_from_world));

    const int previous_revision = reference.map_change_index;
    ++current.map_change_index;
    ASSERT_TRUE(estimator.RebaseReferencePose(
        reference.frame_id, reference.map_id, reference.coordinate_epoch,
        previous_revision, current.map_change_index,
        corrected_reference_from_world));

    const auto result = estimator.Estimate(current.View());
    const autonomy::common::math::SE3f expected =
        current_from_reference * corrected_reference_from_world;
    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.diagnostics.reference_map_revision,
              current.map_change_index);
    EXPECT_TRUE(result.camera_from_world.translation().isApprox(
        expected.translation(), 1.0e-3F));
    EXPECT_TRUE(result.camera_from_world.rotationMatrix().isApprox(
        expected.rotationMatrix(), 1.0e-3F));
}

TEST(RgbdTemporalPoseEstimatorTest,
     RejectsInvalidReferencePoseRebaseWithoutMutatingReference) {
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.1F, 0.0F, 0.0F), Eigen::Matrix3f::Identity());
    auto [reference, current] = MakeSyntheticPair(current_from_reference);
    RgbdTemporalPoseEstimator estimator;
    ASSERT_TRUE(estimator.UpdateReference(reference.View(),
                                          autonomy::common::math::SE3f()));
    const int previous_revision = reference.map_change_index;
    const int next_revision = previous_revision + 1;
    const autonomy::common::math::SE3f corrected_reference_from_world(
        Eigen::Vector3f(1.0F, 0.0F, 0.0F), Eigen::Matrix3f::Identity());

    EXPECT_FALSE(estimator.RebaseReferencePose(
        reference.frame_id + 1, reference.map_id, reference.coordinate_epoch,
        previous_revision, next_revision, corrected_reference_from_world));
    EXPECT_FALSE(estimator.RebaseReferencePose(
        reference.frame_id, reference.map_id + 1, reference.coordinate_epoch,
        previous_revision, next_revision, corrected_reference_from_world));
    EXPECT_FALSE(estimator.RebaseReferencePose(
        reference.frame_id, reference.map_id, reference.coordinate_epoch + 1,
        previous_revision, next_revision, corrected_reference_from_world));
    EXPECT_FALSE(estimator.RebaseReferencePose(
        reference.frame_id, reference.map_id, reference.coordinate_epoch,
        previous_revision + 1, next_revision, corrected_reference_from_world));
    EXPECT_FALSE(estimator.RebaseReferencePose(
        reference.frame_id, reference.map_id, reference.coordinate_epoch,
        previous_revision, previous_revision, corrected_reference_from_world));
    autonomy::common::math::SE3f invalid_pose = corrected_reference_from_world;
    invalid_pose.translation().x() = std::numeric_limits<float>::infinity();
    EXPECT_FALSE(estimator.RebaseReferencePose(
        reference.frame_id, reference.map_id, reference.coordinate_epoch,
        previous_revision, next_revision, invalid_pose));

    const auto unchanged_result = estimator.Estimate(current.View());
    ASSERT_TRUE(unchanged_result.success);
    EXPECT_TRUE(unchanged_result.camera_from_world.translation().isApprox(
        current_from_reference.translation(), 1.0e-3F));
    EXPECT_TRUE(unchanged_result.camera_from_world.rotationMatrix().isApprox(
        current_from_reference.rotationMatrix(), 1.0e-3F));

    ++current.map_change_index;
    const auto result = estimator.Estimate(current.View());
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.diagnostics.rejection,
              RgbdTemporalPoseEstimator::RejectionReason::kMapRevisionMismatch);
    EXPECT_EQ(result.diagnostics.reference_map_revision, previous_revision);
}

TEST(RgbdTemporalPoseEstimatorTest, RejectsSpatiallyDegenerateInliers) {
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.03F, 0.0F, 0.0F), Eigen::Matrix3f::Identity());
    auto [reference, current] = MakeSyntheticPair(current_from_reference, true);
    RgbdTemporalPoseEstimator estimator;
    ASSERT_TRUE(estimator.UpdateReference(reference.View(),
                                          autonomy::common::math::SE3f()));

    const RgbdTemporalPoseEstimator::Result result =
        estimator.Estimate(current.View());
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.diagnostics.stage,
              RgbdTemporalPoseEstimator::Stage::kSpatialCoverage);
    EXPECT_EQ(
        result.diagnostics.rejection,
        RgbdTemporalPoseEstimator::RejectionReason::kInsufficientGridCoverage);
    EXPECT_GE(result.diagnostics.inlier_count, 20);
    EXPECT_LT(result.diagnostics.covered_grid_cells, 6);
    EXPECT_FALSE(result.pnp_inlier_pairs.empty());
    EXPECT_EQ(result.pnp_inlier_pairs.size(),
              static_cast<std::size_t>(result.diagnostics.inlier_count));
}

TEST(RgbdTemporalPoseEstimatorTest, RejectsDepthInconsistentPose) {
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.1F, 0.0F, 0.0F), Eigen::Matrix3f::Identity());
    auto [reference, current] = MakeSyntheticPair(current_from_reference);
    RgbdTemporalPoseEstimator estimator;
    ASSERT_TRUE(estimator.UpdateReference(reference.View(),
                                          autonomy::common::math::SE3f()));

    for (float& depth : current.depths) {
        depth += 2.0F;
    }
    const RgbdTemporalPoseEstimator::Result result =
        estimator.Estimate(current.View());
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.diagnostics.rejection,
              RgbdTemporalPoseEstimator::RejectionReason::
                  kInsufficientDepthConsistency);
    EXPECT_GE(result.diagnostics.depth_check_count, 12);
    EXPECT_FALSE(result.pnp_inlier_pairs.empty());
    EXPECT_EQ(result.pnp_inlier_pairs.size(),
              static_cast<std::size_t>(result.diagnostics.inlier_count));
}

TEST(RgbdTemporalPoseEstimatorTest,
     RejectsFullXyzInconsistencyEvenWhenZTolerancePasses) {
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.1F, 0.0F, 0.0F), Eigen::Matrix3f::Identity());
    auto [reference, current] = MakeSyntheticPair(current_from_reference);
    for (std::size_t index = 0; index < current.depths.size(); ++index) {
        if (index < 30) {
            current.depths[index] += 0.25F;
        } else {
            current.depths[index] = -1.0F;
        }
    }
    RgbdTemporalPoseEstimator estimator;
    ASSERT_TRUE(estimator.UpdateReference(reference.View(),
                                          autonomy::common::math::SE3f()));

    const auto result = estimator.Estimate(current.View());
    EXPECT_FALSE(result.success);
    EXPECT_GE(result.diagnostics.depth_check_count, 30);
    EXPECT_GE(result.diagnostics.depth_consistent_count, 30);
    EXPECT_GE(result.diagnostics.xyz_check_count, 30);
    EXPECT_LT(result.diagnostics.xyz_consistent_count,
              result.diagnostics.xyz_check_count);
    EXPECT_EQ(result.diagnostics.rejection,
              RgbdTemporalPoseEstimator::RejectionReason::
                  kInsufficientXyzConsistency);
}

TEST(RgbdTemporalPoseEstimatorTest,
     FullXyzCheckCountCannotBeRelaxedBelowTwelve) {
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.1F, 0.0F, 0.0F), Eigen::Matrix3f::Identity());
    auto [reference, current] = MakeSyntheticPair(current_from_reference);
    for (std::size_t index = 11; index < current.depths.size(); ++index) {
        current.depths[index] = -1.0F;
    }
    RgbdTemporalPoseEstimator::Options options;
    options.minimum_depth_checks = 1;
    options.minimum_depth_check_ratio = 0.01F;
    options.minimum_depth_consistency_ratio = 0.01F;
    RgbdTemporalPoseEstimator estimator(options);
    ASSERT_TRUE(estimator.UpdateReference(reference.View(),
                                          autonomy::common::math::SE3f()));

    const auto result = estimator.Estimate(current.View());
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.diagnostics.xyz_check_count, 11);
    EXPECT_EQ(result.diagnostics.xyz_consistent_count, 11);
    EXPECT_EQ(result.diagnostics.rejection,
              RgbdTemporalPoseEstimator::RejectionReason::
                  kInsufficientXyzConsistency);
}

TEST(RgbdTemporalPoseEstimatorTest, RejectsInsufficientCurrentDepthCoverage) {
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.1F, 0.0F, 0.0F), Eigen::Matrix3f::Identity());
    auto [reference, current] = MakeSyntheticPair(current_from_reference);
    RgbdTemporalPoseEstimator estimator;
    ASSERT_TRUE(estimator.UpdateReference(reference.View(),
                                          autonomy::common::math::SE3f()));

    for (std::size_t index = 20; index < current.depths.size(); ++index) {
        current.depths[index] =
            index % 2 == 0 ? -1.0F : std::numeric_limits<float>::quiet_NaN();
    }
    const RgbdTemporalPoseEstimator::Result result =
        estimator.Estimate(current.View());
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.diagnostics.rejection,
              RgbdTemporalPoseEstimator::RejectionReason::
                  kInsufficientDepthCheckRatio);
    EXPECT_GE(result.diagnostics.inlier_count, 40);
    EXPECT_EQ(result.diagnostics.depth_check_count, 20);
}

TEST(RgbdTemporalPoseEstimatorTest, CompactsDepthBackedReference) {
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.1F, 0.0F, 0.0F), Eigen::Matrix3f::Identity());
    auto [reference, current] = MakeSyntheticPair(current_from_reference);
    int valid_reference_depths = 0;
    for (std::size_t index = 0; index < reference.depths.size(); ++index) {
        if (index % 3 == 1) {
            reference.depths[index] = -1.0F;
        } else {
            ++valid_reference_depths;
        }
    }

    RgbdTemporalPoseEstimator estimator;
    ASSERT_TRUE(estimator.UpdateReference(reference.View(),
                                          autonomy::common::math::SE3f()));
    const RgbdTemporalPoseEstimator::Result result =
        estimator.Estimate(current.View());
    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.diagnostics.mutual_match_count, valid_reference_depths);
    ASSERT_EQ(result.pnp_inlier_pairs.size(),
              static_cast<std::size_t>(result.diagnostics.inlier_count));
    bool observed_compacted_offset = false;
    for (const auto& pair : result.pnp_inlier_pairs) {
        EXPECT_EQ(pair.reference_original_index, pair.current_index);
        EXPECT_NE(pair.reference_original_index % 3, 1);
        observed_compacted_offset |=
            pair.reference_original_index >= valid_reference_depths;
    }
    EXPECT_TRUE(observed_compacted_offset);
}

TEST(RgbdTemporalPoseEstimatorTest, InvalidUpdatePreservesReference) {
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.1F, 0.0F, 0.0F), Eigen::Matrix3f::Identity());
    auto [reference, current] = MakeSyntheticPair(current_from_reference);
    RgbdTemporalPoseEstimator estimator;
    ASSERT_TRUE(estimator.UpdateReference(reference.View(),
                                          autonomy::common::math::SE3f()));

    SyntheticFrame invalid_reference = reference;
    for (std::size_t index = 10; index < invalid_reference.depths.size();
         ++index) {
        invalid_reference.depths[index] = -1.0F;
    }
    EXPECT_FALSE(estimator.UpdateReference(invalid_reference.View(),
                                           autonomy::common::math::SE3f()));
    EXPECT_TRUE(estimator.HasReference());
    EXPECT_TRUE(estimator.Estimate(current.View()).success);
}

TEST(RgbdTemporalPoseEstimatorTest, RejectsInvalidOptions) {
    RgbdTemporalPoseEstimator::Options options;
    options.minimum_depth_check_ratio = std::numeric_limits<float>::quiet_NaN();
    EXPECT_THROW((void)RgbdTemporalPoseEstimator{options},
                 std::invalid_argument);

    options = RgbdTemporalPoseEstimator::Options();
    options.minimum_matches = 3;
    EXPECT_THROW((void)RgbdTemporalPoseEstimator{options},
                 std::invalid_argument);
}

TEST(RgbdTemporalPoseEstimatorTest, ResetDiscardsReference) {
    const autonomy::common::math::SE3f current_from_reference(
        Eigen::Vector3f(0.1F, 0.0F, 0.0F), Eigen::Matrix3f::Identity());
    auto [reference, current] = MakeSyntheticPair(current_from_reference);
    RgbdTemporalPoseEstimator estimator;
    ASSERT_TRUE(estimator.UpdateReference(reference.View(),
                                          autonomy::common::math::SE3f()));
    ASSERT_TRUE(estimator.HasReference());

    estimator.Reset();
    EXPECT_FALSE(estimator.HasReference());
    EXPECT_FALSE(estimator.Estimate(current.View()).success);
}

TEST(RgbdTemporalPoseEstimatorTest, DiagnosticNamesAreStable) {
    EXPECT_STREQ(RgbdTemporalPoseEstimator::StageName(
                     RgbdTemporalPoseEstimator::Stage::kDepthConsistency),
                 "depth_consistency");
    EXPECT_STREQ(RgbdTemporalPoseEstimator::RejectionReasonName(
                     RgbdTemporalPoseEstimator::RejectionReason::
                         kInsufficientDepthConsistency),
                 "insufficient_depth_consistency");
}

}  // namespace
}  // namespace autonomy::localization::atlas
