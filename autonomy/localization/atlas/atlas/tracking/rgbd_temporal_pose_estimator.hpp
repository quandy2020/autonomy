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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_TRACKING_RGBD_TEMPORAL_POSE_ESTIMATOR_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_TRACKING_RGBD_TEMPORAL_POSE_ESTIMATOR_HPP_

#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "autonomy/common/math/se3.hpp"

namespace autonomy::localization::atlas {

/**
 * Estimates an RGB-D camera pose from the last visually verified frame.
 *
 * The stored reference is deliberately self-contained: it owns descriptors,
 * keypoints, depths, intrinsics, and the verified pose. It never retains Frame,
 * Map, KeyFrame, or MapPoint pointers, so reset and map replacement cannot
 * leave dangling recovery state.
 */
class RgbdTemporalPoseEstimator
{
public:
    enum class Quality : std::uint8_t {
        kRejected,
        kStandard,
        kLowCoverageFallback,
    };

    enum class Stage : std::uint8_t {
        kInputValidation,
        kDescriptorMatching,
        kRigid3dRansac,
        kPoseGuidedMatching,
        kPnpRansac,
        kPnpRefinement,
        kSpatialCoverage,
        kDepthConsistency,
        kReprojection,
        kConditioning,
        kPoseComposition,
        kComplete,
    };

    enum class RejectionReason : std::uint8_t {
        kNone,
        kReferenceUnavailable,
        kInvalidCurrentFrame,
        kMapIdMismatch,
        kCoordinateEpochMismatch,
        kMapRevisionMismatch,
        kSameFrame,
        kTimestampRegression,
        kReferenceExpired,
        kDescriptorLayoutMismatch,
        kDescriptorMatchingFailed,
        kInsufficientMutualMatches,
        kRigid3dValidationFailed,
        kRigidPoseDisagreement,
        kPnpRansacFailed,
        kInsufficientPnpInliers,
        kInvalidPnpPose,
        kInvalidPnpInlierIndex,
        kPnpRefinementFailed,
        kInvalidRefinedPose,
        kInsufficientGridCoverage,
        kInvalidRotationMatrix,
        kInsufficientDepthChecks,
        kInsufficientDepthCheckRatio,
        kInsufficientDepthConsistency,
        kInsufficientXyzConsistency,
        kExcessiveXyzError,
        kReprojectionFailed,
        kInvalidReprojectedPoints,
        kExcessiveReprojectionError,
        kIllConditionedPose,
        kInvalidWorldPose,
        kLowCoverageConfidenceFailed,
    };

    // Kept for tracker API compatibility; contains accepted geometric inliers.
    struct PnpInlierPair {
        int reference_original_index = -1;
        int current_index = -1;
    };

    struct Options {
        float descriptor_ratio = 0.8F;
        int maximum_descriptor_distance = 80;
        int minimum_matches = 30;
        int minimum_inliers = 20;
        int ransac_iterations = 100;
        float ransac_reprojection_error_pixels = 3.0F;
        double ransac_confidence = 0.999;
        int minimum_grid_cells = 6;
        int minimum_depth_checks = 12;
        float depth_absolute_tolerance_meters = 0.15F;
        float depth_relative_tolerance = 0.15F;
        float minimum_depth_consistency_ratio = 0.65F;
        float minimum_depth_check_ratio = 0.6F;
        float maximum_reprojection_rmse_pixels = 2.5F;
        double maximum_pose_condition_number = 1.0e10;
        double maximum_reference_age_seconds = 1.5;
    };

    /** Non-owning view valid only for the duration of a method call. */
    struct FrameView {
        const cv::Mat* descriptors = nullptr;
        const std::vector<cv::KeyPoint>* keypoints = nullptr;
        const std::vector<float>* depths = nullptr;
        Eigen::Matrix3f intrinsics = Eigen::Matrix3f::Identity();
        float minimum_x = 0.0F;
        float maximum_x = 0.0F;
        float minimum_y = 0.0F;
        float maximum_y = 0.0F;
        double timestamp = 0.0;
        std::uint64_t frame_id = 0;
        std::uint64_t map_id = 0;
        std::uint64_t coordinate_epoch = 0;
        int map_change_index = 0;
    };

    /**
     * Bounded retry outcome retained without replacing the primary rejection.
     */
    struct RetryDiagnostics {
        bool attempted = false;
        Stage stage = Stage::kInputValidation;
        RejectionReason rejection = RejectionReason::kNone;
        int guided_match_count = 0;
        int final_match_count = 0;
        int rigid_depth_pair_count = 0;
        int rigid_inlier_count = 0;
        int rigid_five_centimeter_inlier_count = 0;
        int covered_grid_cells = 0;
        int depth_check_count = 0;
        int depth_consistent_count = 0;
        int xyz_check_count = 0;
        int xyz_consistent_count = 0;
        double rigid_inlier_ratio = 0.0;
        double rigid_rmse_meters = std::numeric_limits<double>::infinity();
        double rigid_xyz_median_meters =
            std::numeric_limits<double>::infinity();
        double xyz_rmse_meters = std::numeric_limits<double>::infinity();
        double reprojection_rmse_pixels =
            std::numeric_limits<double>::infinity();
        double pose_condition_number =
            std::numeric_limits<double>::infinity();
    };

    /** Owning snapshot populated on every Estimate() return path. */
    struct Diagnostics {
        Stage stage = Stage::kInputValidation;
        RejectionReason rejection = RejectionReason::kNone;
        bool reference_available = false;
        int reference_feature_count = 0;
        int current_feature_count = 0;
        int mutual_match_count = 0;
        int guided_match_count = 0;
        int final_match_count = 0;
        int inlier_count = 0;
        int pnp_inlier_count = 0;
        int rigid_depth_pair_count = 0;
        int rigid_inlier_count = 0;
        int rigid_five_centimeter_inlier_count = 0;
        int covered_grid_cells = 0;
        int depth_check_count = 0;
        int depth_consistent_count = 0;
        int xyz_check_count = 0;
        int xyz_consistent_count = 0;
        double reprojection_rmse_pixels =
            std::numeric_limits<double>::infinity();
        double pose_condition_number = std::numeric_limits<double>::infinity();
        double rigid_inlier_ratio = 0.0;
        double rigid_rmse_meters = std::numeric_limits<double>::infinity();
        double rigid_xyz_median_meters =
            std::numeric_limits<double>::infinity();
        double xyz_rmse_meters = std::numeric_limits<double>::infinity();
        bool used_rigid_pose = false;
        bool used_pose_guided_matching = false;
        RetryDiagnostics rigid_retry;
        std::uint64_t reference_frame_id = 0;
        std::uint64_t current_frame_id = 0;
        std::uint64_t reference_map_id = 0;
        std::uint64_t current_map_id = 0;
        std::uint64_t reference_coordinate_epoch = 0;
        std::uint64_t current_coordinate_epoch = 0;
        int reference_map_revision = -1;
        int current_map_revision = -1;
    };

    struct Result {
        bool success = false;
        Quality quality = Quality::kRejected;
        autonomy::common::math::SE3f camera_from_world;
        Diagnostics diagnostics;
        std::vector<PnpInlierPair> pnp_inlier_pairs;
    };

    RgbdTemporalPoseEstimator();
    explicit RgbdTemporalPoseEstimator(const Options& options);

    /**
     * Replaces the reference with a visually verified RGB-D frame.
     *
     * Returns false without modifying the previous reference when the input
     * lacks enough valid depth-supported features.
     */
    bool UpdateReference(const FrameView& frame,
                         const autonomy::common::math::SE3f& camera_from_world);

    /**
     * Rebases the stored reference after an in-place backend map correction.
     *
     * The reference identity and previous revision must match exactly, and the
     * new revision must advance monotonically. Feature storage, timestamps,
     * map identity, and coordinate epoch are left unchanged.
     */
    bool RebaseReferencePose(
        std::uint64_t expected_frame_id, std::uint64_t expected_map_id,
        std::uint64_t expected_coordinate_epoch, int expected_map_change_index,
        int new_map_change_index,
        const autonomy::common::math::SE3f& camera_from_world);

    /**
     * Estimates current camera-from-world without modifying the reference.
     *
     * A successful estimate has passed descriptor matching, robust rigid 3D or
     * PnP estimation, spatial coverage, full-XYZ consistency, reprojection, and
     * pose-conditioning gates.
     */
    Result Estimate(const FrameView& frame) const;

    void Reset();
    bool HasReference() const;

    static const char* StageName(Stage stage);
    static const char* RejectionReasonName(RejectionReason rejection);

private:
    Result EstimateInternal(const FrameView& frame,
                            bool prefer_rigid_recovery,
                            bool require_pnp_agreement) const;

    struct Reference {
        cv::Mat descriptors;
        std::vector<cv::KeyPoint> keypoints;
        std::vector<float> depths;
        std::vector<int> original_feature_indices;
        Eigen::Matrix3f intrinsics = Eigen::Matrix3f::Identity();
        autonomy::common::math::SE3f camera_from_world;
        float minimum_x = 0.0F;
        float maximum_x = 0.0F;
        float minimum_y = 0.0F;
        float maximum_y = 0.0F;
        double timestamp = 0.0;
        std::uint64_t frame_id = 0;
        std::uint64_t map_id = 0;
        std::uint64_t coordinate_epoch = 0;
        int map_change_index = 0;
    };

    Options options_;
    Reference reference_;
    bool has_reference_ = false;
};

}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_TRACKING_RGBD_TEMPORAL_POSE_ESTIMATOR_HPP_
