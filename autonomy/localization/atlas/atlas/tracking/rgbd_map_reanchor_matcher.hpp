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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_TRACKING_RGBD_MAP_REANCHOR_MATCHER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_TRACKING_RGBD_MAP_REANCHOR_MATCHER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "autonomy/common/math/se3.hpp"

namespace autonomy::localization::atlas {

class Frame;
class MapPoint;

namespace internal {

inline constexpr std::size_t kRgbdMapReanchorDescriptorKeyFrameLimit = 8;
inline constexpr int kRgbdMapReanchorDescriptorRowsPerMapPoint = 4;
inline constexpr int kRgbdGeometryReanchorMinimumAssociations = 50;
inline constexpr int kRgbdGeometryReanchorMinimumGridCells = 6;

constexpr bool ShouldActivateRgbdGeometryReanchor(bool input_valid,
                                                  int assigned_matches,
                                                  int occupied_grid_cells) {
    return input_valid &&
           assigned_matches >= kRgbdGeometryReanchorMinimumAssociations &&
           occupied_grid_cells >= kRgbdGeometryReanchorMinimumGridCells;
}

constexpr bool PassesRgbdGeometryReanchorPostOptimization(
    bool identity_stable, bool input_valid, int consistent_associations,
    int occupied_grid_cells) {
    return identity_stable && input_valid &&
           consistent_associations >=
               kRgbdGeometryReanchorMinimumAssociations &&
           occupied_grid_cells >= kRgbdGeometryReanchorMinimumGridCells;
}

/** A geometry-validated local-map/feature descriptor candidate. */
struct RgbdMapReanchorCandidate {
    std::size_t map_point_index = 0;
    std::uint64_t map_point_id = 0;
    int feature_index = -1;
    int descriptor_distance = std::numeric_limits<int>::max();
    double squared_reprojection_error_pixels =
        std::numeric_limits<double>::infinity();
    double squared_xyz_error_meters =
        std::numeric_limits<double>::infinity();
    bool used_observation_descriptor = false;
};

struct RgbdMapReanchorSelection {
    std::vector<RgbdMapReanchorCandidate> assigned;
    int mutual_matches = 0;
};

/**
 * Selects descriptor matches using bidirectional best/ratio checks followed by
 * deterministic global one-to-one assignment.
 */
RgbdMapReanchorSelection SelectRgbdMapReanchorCandidates(
    const std::vector<RgbdMapReanchorCandidate>& candidates,
    double descriptor_ratio, int maximum_descriptor_distance);

/**
 * Selects bidirectional geometry-nearest matches below an XYZ-error limit,
 * followed by deterministic global one-to-one assignment. A positive
 * infinity limit audits the candidate graph's existing dynamic XYZ gate.
 */
RgbdMapReanchorSelection SelectRgbdMapReanchorGeometryCandidates(
    const std::vector<RgbdMapReanchorCandidate>& candidates,
    double maximum_xyz_error_meters);

/** Adds a distinct descriptor while enforcing the bounded bank capacity. */
bool AppendUniqueRgbdMapReanchorDescriptor(const cv::Mat& descriptor,
                                           cv::Mat* descriptor_bank);

/** Exact circular reprojection gate shared by matching and focused tests. */
bool IsWithinRgbdMapReanchorReprojectionGate(
    double squared_reprojection_error_pixels);

}  // namespace internal

/**
 * Pure local-map matcher used to audit an RGB-D temporal pose before map
 * reanchoring. Match() performs no Frame, Map, KeyFrame, or MapPoint writes.
 */
class RgbdMapReanchorMatcher
{
public:
    inline static constexpr std::array<int, 5>
        kDescriptorTelemetryThresholds = {50, 60, 70, 80, 100};
    inline static constexpr std::array<double, 3>
        kGeometryTelemetryMaximumXyzErrorsMeters = {
            0.05, 0.10, std::numeric_limits<double>::infinity()};

    /** Read-only, non-owning feature snapshot valid for one Match() call. */
    struct FrameView {
        const cv::Mat* descriptors = nullptr;
        const std::vector<cv::KeyPoint>* keypoints = nullptr;
        const std::vector<float>* depths = nullptr;
        Eigen::Matrix3f intrinsics = Eigen::Matrix3f::Identity();
        int feature_count = 0;
        float minimum_x = 0.0F;
        float maximum_x = 0.0F;
        float minimum_y = 0.0F;
        float maximum_y = 0.0F;
    };

    /** Owning MapPoint snapshot. The opaque pointer is never dereferenced. */
    struct MapPointView {
        MapPoint* map_point = nullptr;
        std::size_t source_index = 0;
        std::uint64_t map_point_id = 0;
        bool valid_for_expected_map = false;
        Eigen::Vector3f world_point = Eigen::Vector3f::Zero();
        cv::Mat descriptor_bank;
        int observation_descriptor_rows = 0;
    };

    struct Options {
        double descriptor_ratio = 0.75;
    };

    struct DescriptorThresholdSupport {
        int maximum_descriptor_distance = 0;
        int mutual_matches = 0;
        int assigned_matches = 0;
        int occupied_grid_cells = 0;
    };

    struct GeometryThresholdSupport {
        double maximum_xyz_error_meters =
            std::numeric_limits<double>::infinity();
        int mutual_matches = 0;
        int assigned_matches = 0;
        int occupied_grid_cells = 0;
    };

    /** One existing feature-slot/MapPoint identity checked after pose BA. */
    struct AssociationView {
        int feature_index = -1;
        MapPoint* map_point = nullptr;
        std::uint64_t map_point_id = 0;
        bool valid_for_expected_map = false;
        bool optimizer_outlier = false;
        Eigen::Vector3f world_point = Eigen::Vector3f::Zero();
    };

    struct AssociationValidation {
        bool input_valid = false;
        int input_associations = 0;
        int valid_unique_associations = 0;
        int consistent_associations = 0;
        int occupied_grid_cells = 0;
        std::vector<int> accepted_feature_indices;
    };

    struct Diagnostics {
        bool input_valid = false;
        int input_map_points = 0;
        int valid_unique_map_points = 0;
        int descriptor_rows = 0;
        int multi_descriptor_map_points = 0;
        int projected_map_points = 0;
        int nearby_candidates = 0;
        int candidates_after_geometry = 0;
        int candidates_after_descriptor = 0;
        int mutual_matches = 0;
        int assigned_matches = 0;
        int observation_descriptor_matches = 0;
        int occupied_grid_cells = 0;
        std::array<DescriptorThresholdSupport,
                   kDescriptorTelemetryThresholds.size()>
            descriptor_threshold_support{};
        std::array<GeometryThresholdSupport,
                   kGeometryTelemetryMaximumXyzErrorsMeters.size()>
            geometry_threshold_support{};
        double reprojection_rmse_pixels =
            std::numeric_limits<double>::infinity();
        double xyz_rmse_meters =
            std::numeric_limits<double>::infinity();
    };

    struct Result {
        std::vector<MapPoint*> associations;
        std::vector<MapPoint*> geometry_five_centimeter_associations;
        Diagnostics diagnostics;
        bool pnp_attempted = false;
        bool pnp_succeeded = false;
        int pnp_inliers = 0;
        autonomy::common::math::SE3f refined_camera_from_world;
        std::vector<MapPoint*> refined_associations;
        Diagnostics refined_diagnostics;
    };

    RgbdMapReanchorMatcher();
    explicit RgbdMapReanchorMatcher(const Options& options);

    Result Match(
        const FrameView& frame,
        const std::vector<MapPointView>& local_map_points,
        const autonomy::common::math::SE3f& provisional_camera_from_world)
        const;

    /**
     * Revalidates existing identities at an optimized pose without rematching.
     * The gate is the exact 3 px circle and fixed full-XYZ 5 cm limit.
     */
    AssociationValidation ValidateFiveCentimeterAssociations(
        const FrameView& frame,
        const std::vector<AssociationView>& associations,
        const autonomy::common::math::SE3f& camera_from_world) const;

private:
    Result MatchAtPose(
        const FrameView& frame,
        const std::vector<MapPointView>& local_map_points,
        const autonomy::common::math::SE3f& camera_from_world) const;

    Options options_;
};

}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_TRACKING_RGBD_MAP_REANCHOR_MATCHER_HPP_
