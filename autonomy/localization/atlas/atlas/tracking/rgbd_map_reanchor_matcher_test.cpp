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
#include <cstddef>
#include <cstdint>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace autonomy::localization::atlas {
namespace {

using Candidate = internal::RgbdMapReanchorCandidate;

constexpr float kImageExtent = 200.0F;

MapPoint* OpaqueMapPoint(std::uintptr_t identity) {
    return reinterpret_cast<MapPoint*>(identity);
}

cv::Mat DescriptorWithSetBits(int set_bits) {
    cv::Mat descriptor = cv::Mat::zeros(1, 32, CV_8U);
    for (int bit = 0; bit < set_bits; ++bit) {
        descriptor.at<std::uint8_t>(0, bit / 8) |=
            static_cast<std::uint8_t>(1U << (bit % 8));
    }
    return descriptor;
}

struct SyntheticFrame {
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> keypoints;
    std::vector<float> depths;
    Eigen::Matrix3f intrinsics = Eigen::Matrix3f::Identity();

    RgbdMapReanchorMatcher::FrameView View() const {
        RgbdMapReanchorMatcher::FrameView view;
        view.descriptors = &descriptors;
        view.keypoints = &keypoints;
        view.depths = &depths;
        view.intrinsics = intrinsics;
        view.feature_count = descriptors.rows;
        view.minimum_x = 0.0F;
        view.maximum_x = kImageExtent;
        view.minimum_y = 0.0F;
        view.maximum_y = kImageExtent;
        return view;
    }
};

SyntheticFrame MakeFrame(const std::vector<cv::Point2f>& pixels,
                         const std::vector<float>& depths) {
    EXPECT_EQ(pixels.size(), depths.size());
    SyntheticFrame frame;
    frame.intrinsics << 100.0F, 0.0F, 50.0F, 0.0F, 100.0F, 50.0F, 0.0F,
        0.0F, 1.0F;
    frame.descriptors =
        cv::Mat::zeros(static_cast<int>(pixels.size()), 32, CV_8U);
    frame.depths = depths;
    frame.keypoints.reserve(pixels.size());
    for (const cv::Point2f& pixel : pixels) {
        frame.keypoints.emplace_back(pixel, 1.0F);
    }
    return frame;
}

Eigen::Vector3f BackProject(const cv::Point2f& pixel, float depth,
                            const Eigen::Matrix3f& intrinsics) {
    return Eigen::Vector3f(
        (pixel.x - intrinsics(0, 2)) * depth / intrinsics(0, 0),
        (pixel.y - intrinsics(1, 2)) * depth / intrinsics(1, 1), depth);
}

RgbdMapReanchorMatcher::MapPointView MakeMapPoint(
    std::size_t source_index, std::uint64_t map_point_id,
    MapPoint* map_point, const Eigen::Vector3f& world_point,
    const cv::Mat& descriptor = DescriptorWithSetBits(0)) {
    RgbdMapReanchorMatcher::MapPointView view;
    view.map_point = map_point;
    view.source_index = source_index;
    view.map_point_id = map_point_id;
    view.valid_for_expected_map = true;
    view.world_point = world_point;
    view.descriptor_bank = descriptor.clone();
    return view;
}

RgbdMapReanchorMatcher::MapPointView WithObservationDescriptorBank(
    RgbdMapReanchorMatcher::MapPointView view,
    const std::vector<cv::Mat>& descriptors) {
    cv::vconcat(descriptors, view.descriptor_bank);
    view.observation_descriptor_rows = view.descriptor_bank.rows;
    return view;
}

RgbdMapReanchorMatcher::AssociationView MakeAssociation(
    int feature_index, std::uint64_t map_point_id, MapPoint* map_point,
    const Eigen::Vector3f& world_point, bool optimizer_outlier = false) {
    RgbdMapReanchorMatcher::AssociationView association;
    association.feature_index = feature_index;
    association.map_point = map_point;
    association.map_point_id = map_point_id;
    association.valid_for_expected_map = true;
    association.optimizer_outlier = optimizer_outlier;
    association.world_point = world_point;
    return association;
}

std::vector<std::tuple<std::size_t, int, int>> SelectionSignature(
    const internal::RgbdMapReanchorSelection& selection) {
    std::vector<std::tuple<std::size_t, int, int>> signature;
    signature.reserve(selection.assigned.size());
    for (const Candidate& candidate : selection.assigned) {
        signature.emplace_back(candidate.map_point_index,
                               candidate.feature_index,
                               candidate.descriptor_distance);
    }
    return signature;
}

TEST(RgbdMapReanchorMatcherTest, UsesExactThreePixelCircularGate) {
    EXPECT_TRUE(
        internal::IsWithinRgbdMapReanchorReprojectionGate(0.0));
    EXPECT_TRUE(
        internal::IsWithinRgbdMapReanchorReprojectionGate(9.0));
    EXPECT_FALSE(
        internal::IsWithinRgbdMapReanchorReprojectionGate(9.000001));
    EXPECT_FALSE(internal::IsWithinRgbdMapReanchorReprojectionGate(-0.001));
    EXPECT_FALSE(internal::IsWithinRgbdMapReanchorReprojectionGate(
        std::numeric_limits<double>::infinity()));

    SyntheticFrame frame =
        MakeFrame({cv::Point2f(53.0F, 50.0F)}, {2.0F});
    MapPoint* const expected = OpaqueMapPoint(0x1000U);
    const auto map_point = MakeMapPoint(
        0, 7, expected, Eigen::Vector3f(0.0F, 0.0F, 2.0F));
    const auto boundary_result = RgbdMapReanchorMatcher().Match(
        frame.View(), {map_point}, autonomy::common::math::SE3f());
    ASSERT_EQ(boundary_result.associations.size(), 1U);
    EXPECT_EQ(boundary_result.associations[0], expected);

    frame.keypoints[0].pt = cv::Point2f(53.0F, 50.25F);
    const auto outside_circle_result = RgbdMapReanchorMatcher().Match(
        frame.View(), {map_point}, autonomy::common::math::SE3f());
    ASSERT_EQ(outside_circle_result.associations.size(), 1U);
    EXPECT_EQ(outside_circle_result.associations[0], nullptr);
    EXPECT_EQ(outside_circle_result.diagnostics.nearby_candidates, 0);
}

TEST(RgbdMapReanchorMatcherTest, EnforcesLowDescriptorDistanceBoundary) {
    SyntheticFrame frame =
        MakeFrame({cv::Point2f(50.0F, 50.0F)}, {2.0F});
    const Eigen::Vector3f world_point(0.0F, 0.0F, 2.0F);

    MapPoint* const at_limit = OpaqueMapPoint(0x1010U);
    const auto accepted = RgbdMapReanchorMatcher().Match(
        frame.View(),
        {MakeMapPoint(0, 8, at_limit, world_point,
                      DescriptorWithSetBits(50))},
        autonomy::common::math::SE3f());
    ASSERT_EQ(accepted.associations.size(), 1U);
    EXPECT_EQ(accepted.associations[0], at_limit);
    EXPECT_EQ(accepted.diagnostics.candidates_after_descriptor, 1);

    const auto rejected = RgbdMapReanchorMatcher().Match(
        frame.View(),
        {MakeMapPoint(0, 9, OpaqueMapPoint(0x1020U), world_point,
                      DescriptorWithSetBits(51))},
        autonomy::common::math::SE3f());
    ASSERT_EQ(rejected.associations.size(), 1U);
    EXPECT_EQ(rejected.associations[0], nullptr);
    EXPECT_EQ(rejected.diagnostics.candidates_after_descriptor, 0);
    EXPECT_EQ(rejected.diagnostics.assigned_matches, 0);
}

TEST(RgbdMapReanchorMatcherTest,
     UsesBoundedObservationDescriptorBankDeterministically) {
    cv::Mat bounded_bank;
    EXPECT_TRUE(internal::AppendUniqueRgbdMapReanchorDescriptor(
        DescriptorWithSetBits(0), &bounded_bank));
    EXPECT_FALSE(internal::AppendUniqueRgbdMapReanchorDescriptor(
        DescriptorWithSetBits(0), &bounded_bank));
    EXPECT_TRUE(internal::AppendUniqueRgbdMapReanchorDescriptor(
        DescriptorWithSetBits(1), &bounded_bank));
    EXPECT_TRUE(internal::AppendUniqueRgbdMapReanchorDescriptor(
        DescriptorWithSetBits(2), &bounded_bank));
    EXPECT_TRUE(internal::AppendUniqueRgbdMapReanchorDescriptor(
        DescriptorWithSetBits(3), &bounded_bank));
    EXPECT_FALSE(internal::AppendUniqueRgbdMapReanchorDescriptor(
        DescriptorWithSetBits(4), &bounded_bank));
    EXPECT_EQ(bounded_bank.rows,
              internal::kRgbdMapReanchorDescriptorRowsPerMapPoint);

    SyntheticFrame frame =
        MakeFrame({cv::Point2f(50.0F, 50.0F)}, {2.0F});
    const Eigen::Vector3f world_point(0.0F, 0.0F, 2.0F);
    MapPoint* const expected = OpaqueMapPoint(0x1028U);
    const auto base = MakeMapPoint(0, 10, expected, world_point);
    const std::vector<cv::Mat> descriptors = {
        DescriptorWithSetBits(51), DescriptorWithSetBits(40)};
    const auto forward = RgbdMapReanchorMatcher().Match(
        frame.View(), {WithObservationDescriptorBank(base, descriptors)},
        autonomy::common::math::SE3f());

    std::vector<cv::Mat> reversed_descriptors = descriptors;
    std::reverse(reversed_descriptors.begin(), reversed_descriptors.end());
    const auto reversed = RgbdMapReanchorMatcher().Match(
        frame.View(),
        {WithObservationDescriptorBank(base, reversed_descriptors)},
        autonomy::common::math::SE3f());

    ASSERT_EQ(forward.associations.size(), 1U);
    EXPECT_EQ(forward.associations[0], expected);
    EXPECT_EQ(forward.associations, reversed.associations);
    EXPECT_EQ(forward.diagnostics.descriptor_rows, 2);
    EXPECT_EQ(forward.diagnostics.multi_descriptor_map_points, 1);
    EXPECT_EQ(forward.diagnostics.observation_descriptor_matches, 1);
    EXPECT_EQ(
        forward.diagnostics.descriptor_threshold_support[0].assigned_matches,
        1);

    const std::vector<cv::Mat> too_many_descriptors(
        internal::kRgbdMapReanchorDescriptorRowsPerMapPoint + 1,
        DescriptorWithSetBits(0));
    const auto rejected = RgbdMapReanchorMatcher().Match(
        frame.View(),
        {WithObservationDescriptorBank(base, too_many_descriptors)},
        autonomy::common::math::SE3f());
    EXPECT_EQ(rejected.diagnostics.valid_unique_map_points, 0);
    EXPECT_EQ(rejected.diagnostics.assigned_matches, 0);
}

TEST(RgbdMapReanchorMatcherTest,
     ReportsMonotonicDescriptorThresholdSupportWithoutChangingDefault) {
    const std::vector<cv::Point2f> pixels = {
        {20.0F, 20.0F}, {70.0F, 20.0F}, {120.0F, 20.0F},
        {170.0F, 20.0F}, {20.0F, 70.0F}};
    SyntheticFrame frame = MakeFrame(pixels, std::vector<float>(5, 2.0F));
    std::vector<RgbdMapReanchorMatcher::MapPointView> map_points;
    map_points.reserve(pixels.size());
    for (std::size_t index = 0; index < pixels.size(); ++index) {
        const int descriptor_distance =
            RgbdMapReanchorMatcher::kDescriptorTelemetryThresholds[index];
        map_points.push_back(MakeMapPoint(
            index, 30 + index, OpaqueMapPoint(0x1100U + 0x10U * index),
            BackProject(pixels[index], 2.0F, frame.intrinsics),
            DescriptorWithSetBits(descriptor_distance)));
    }

    const auto result = RgbdMapReanchorMatcher().Match(
        frame.View(), map_points, autonomy::common::math::SE3f());

    EXPECT_EQ(result.diagnostics.assigned_matches, 1);
    EXPECT_EQ(result.diagnostics.candidates_after_descriptor, 1);
    EXPECT_FALSE(result.pnp_attempted);
    for (std::size_t index = 0;
         index < RgbdMapReanchorMatcher::kDescriptorTelemetryThresholds.size();
         ++index) {
        const auto& support =
            result.diagnostics.descriptor_threshold_support[index];
        EXPECT_EQ(
            support.maximum_descriptor_distance,
            RgbdMapReanchorMatcher::kDescriptorTelemetryThresholds[index]);
        EXPECT_EQ(support.mutual_matches, static_cast<int>(index + 1));
        EXPECT_EQ(support.assigned_matches, static_cast<int>(index + 1));
        EXPECT_EQ(support.occupied_grid_cells, static_cast<int>(index + 1));
    }
    for (const auto& support :
         result.diagnostics.geometry_threshold_support) {
        EXPECT_EQ(support.mutual_matches, 5);
        EXPECT_EQ(support.assigned_matches, 5);
        EXPECT_EQ(support.occupied_grid_cells, 5);
    }
    EXPECT_EQ(static_cast<int>(std::count_if(
                  result.geometry_five_centimeter_associations.begin(),
                  result.geometry_five_centimeter_associations.end(),
                  [](const MapPoint* map_point) {
                      return map_point != nullptr;
                  })),
              5);
}

TEST(RgbdMapReanchorMatcherTest,
     AcceptsTelemetryAtHighButRejectsThresholdBeyondHigh) {
    const std::vector<Candidate> candidates = {
        {0, 40, 0, 100, 0.1, 0.01},
    };
    const auto at_high = internal::SelectRgbdMapReanchorCandidates(
        candidates, 0.75, 100);
    EXPECT_EQ(at_high.mutual_matches, 1);
    EXPECT_EQ(at_high.assigned.size(), 1U);

    const auto beyond_high = internal::SelectRgbdMapReanchorCandidates(
        candidates, 0.75, 101);
    EXPECT_EQ(beyond_high.mutual_matches, 0);
    EXPECT_TRUE(beyond_high.assigned.empty());
}

TEST(RgbdMapReanchorMatcherTest,
     GeometryShadowEnforcesFiveAndTenCentimeterBoundaries) {
    const std::vector<Candidate> candidates = {
        {0, 40, 0, 200, 0.3, 0.05 * 0.05},
        {1, 41, 1, 210, 0.2, 0.10 * 0.10},
        {2, 42, 2, 220, 0.1, 0.100001 * 0.100001},
    };

    const auto five_centimeters =
        internal::SelectRgbdMapReanchorGeometryCandidates(candidates, 0.05);
    EXPECT_EQ(five_centimeters.mutual_matches, 1);
    ASSERT_EQ(five_centimeters.assigned.size(), 1U);
    EXPECT_EQ(five_centimeters.assigned[0].map_point_index, 0U);

    const auto ten_centimeters =
        internal::SelectRgbdMapReanchorGeometryCandidates(candidates, 0.10);
    EXPECT_EQ(ten_centimeters.mutual_matches, 2);
    ASSERT_EQ(ten_centimeters.assigned.size(), 2U);

    const auto dynamic_gate =
        internal::SelectRgbdMapReanchorGeometryCandidates(
            candidates, std::numeric_limits<double>::infinity());
    EXPECT_EQ(dynamic_gate.mutual_matches, 3);
    EXPECT_EQ(dynamic_gate.assigned.size(), 3U);

    EXPECT_TRUE(internal::SelectRgbdMapReanchorGeometryCandidates(
                    candidates, -0.01)
                    .assigned.empty());
    EXPECT_TRUE(internal::SelectRgbdMapReanchorGeometryCandidates(
                    candidates,
                    std::numeric_limits<double>::quiet_NaN())
                    .assigned.empty());
}

TEST(RgbdMapReanchorMatcherTest,
     GeometryShadowIsMutualUniqueAndInputOrderDeterministic) {
    std::vector<Candidate> candidates = {
        {0, 100, 0, 250, 0.8, 0.010},
        {0, 100, 1, 240, 0.7, 0.001},
        {1, 101, 0, 230, 0.6, 0.002},
        {1, 101, 1, 220, 0.5, 0.020},
        {2, 102, 2, 210, 0.4, 0.003},
    };
    const auto forward =
        internal::SelectRgbdMapReanchorGeometryCandidates(candidates, 0.20);
    std::reverse(candidates.begin(), candidates.end());
    const auto reversed =
        internal::SelectRgbdMapReanchorGeometryCandidates(candidates, 0.20);

    EXPECT_EQ(forward.mutual_matches, 3);
    ASSERT_EQ(forward.assigned.size(), 3U);
    EXPECT_EQ(SelectionSignature(forward), SelectionSignature(reversed));
    EXPECT_EQ(SelectionSignature(forward),
              (std::vector<std::tuple<std::size_t, int, int>>{
                  {0, 1, 240}, {1, 0, 230}, {2, 2, 210}}));
}

TEST(RgbdMapReanchorMatcherTest,
     GeometryReanchorPolicyRequiresFiftyMatchesAndSixGridCells) {
    EXPECT_FALSE(internal::ShouldActivateRgbdGeometryReanchor(false, 50, 6));
    EXPECT_FALSE(internal::ShouldActivateRgbdGeometryReanchor(true, 49, 6));
    EXPECT_FALSE(internal::ShouldActivateRgbdGeometryReanchor(true, 50, 5));
    EXPECT_TRUE(internal::ShouldActivateRgbdGeometryReanchor(true, 50, 6));

    EXPECT_FALSE(internal::PassesRgbdGeometryReanchorPostOptimization(
        false, true, 50, 6));
    EXPECT_FALSE(internal::PassesRgbdGeometryReanchorPostOptimization(
        true, false, 50, 6));
    EXPECT_FALSE(internal::PassesRgbdGeometryReanchorPostOptimization(
        true, true, 49, 6));
    EXPECT_FALSE(internal::PassesRgbdGeometryReanchorPostOptimization(
        true, true, 50, 5));
    EXPECT_TRUE(internal::PassesRgbdGeometryReanchorPostOptimization(
        true, true, 50, 6));
}

TEST(RgbdMapReanchorMatcherTest,
     PostOptimizationValidationUsesExactThreePixelAndFiveCentimeterGates) {
    SyntheticFrame frame =
        MakeFrame({cv::Point2f(50.0F, 50.0F)}, {1.0F});
    MapPoint* const map_point = OpaqueMapPoint(0x4500U);
    const RgbdMapReanchorMatcher matcher;

    auto association =
        MakeAssociation(0, 450, map_point, Eigen::Vector3f(0.03F, 0.0F, 1.0F));
    auto validation = matcher.ValidateFiveCentimeterAssociations(
        frame.View(), {association}, autonomy::common::math::SE3f());
    EXPECT_TRUE(validation.input_valid);
    EXPECT_EQ(validation.consistent_associations, 1);

    association.world_point.x() = 0.030001F;
    validation = matcher.ValidateFiveCentimeterAssociations(
        frame.View(), {association}, autonomy::common::math::SE3f());
    EXPECT_EQ(validation.consistent_associations, 0);

    association.world_point = Eigen::Vector3f(0.0F, 0.0F, 1.05F);
    validation = matcher.ValidateFiveCentimeterAssociations(
        frame.View(), {association}, autonomy::common::math::SE3f());
    EXPECT_EQ(validation.consistent_associations, 1);

    association.world_point.z() = 1.0501F;
    validation = matcher.ValidateFiveCentimeterAssociations(
        frame.View(), {association}, autonomy::common::math::SE3f());
    EXPECT_EQ(validation.consistent_associations, 0);

    association.world_point = Eigen::Vector3f(0.0F, 0.0F, 1.0F);
    const autonomy::common::math::SE3f moved_camera_from_world(
        Eigen::Matrix3f::Identity(), Eigen::Vector3f(0.1F, 0.0F, 0.0F));
    validation = matcher.ValidateFiveCentimeterAssociations(
        frame.View(), {association}, moved_camera_from_world);
    EXPECT_EQ(validation.consistent_associations, 0);
}

TEST(RgbdMapReanchorMatcherTest,
     PostOptimizationValidationIsIdentityPreservingAndDeterministic) {
    const std::vector<cv::Point2f> pixels = {
        {20.0F, 20.0F}, {70.0F, 20.0F}, {120.0F, 20.0F},
        {170.0F, 20.0F}, {20.0F, 70.0F}, {70.0F, 70.0F}};
    SyntheticFrame frame = MakeFrame(pixels, std::vector<float>(6, 2.0F));
    std::vector<RgbdMapReanchorMatcher::AssociationView> associations;
    associations.reserve(pixels.size());
    for (std::size_t index = 0; index < pixels.size(); ++index) {
        associations.push_back(MakeAssociation(
            static_cast<int>(index), 500 + index,
            OpaqueMapPoint(0x5000U + 0x10U * index),
            BackProject(pixels[index], 2.0F, frame.intrinsics)));
    }
    const RgbdMapReanchorMatcher matcher;
    const auto forward = matcher.ValidateFiveCentimeterAssociations(
        frame.View(), associations, autonomy::common::math::SE3f());
    std::reverse(associations.begin(), associations.end());
    const auto reversed = matcher.ValidateFiveCentimeterAssociations(
        frame.View(), associations, autonomy::common::math::SE3f());

    EXPECT_EQ(forward.consistent_associations, 6);
    EXPECT_EQ(forward.occupied_grid_cells, 6);
    EXPECT_EQ(forward.accepted_feature_indices,
              (std::vector<int>{0, 1, 2, 3, 4, 5}));
    EXPECT_EQ(forward.accepted_feature_indices,
              reversed.accepted_feature_indices);

    associations = {
        MakeAssociation(0, 600, OpaqueMapPoint(0x6000U),
                        BackProject(pixels[0], 2.0F, frame.intrinsics)),
        MakeAssociation(1, 600, OpaqueMapPoint(0x6010U),
                        BackProject(pixels[1], 2.0F, frame.intrinsics)),
        MakeAssociation(2, 602, OpaqueMapPoint(0x6020U),
                        BackProject(pixels[2], 2.0F, frame.intrinsics), true),
        MakeAssociation(3, 603, OpaqueMapPoint(0x6030U),
                        BackProject(pixels[3], 2.0F, frame.intrinsics)),
        MakeAssociation(4, 604, OpaqueMapPoint(0x6030U),
                        BackProject(pixels[4], 2.0F, frame.intrinsics)),
    };
    const auto rejected = matcher.ValidateFiveCentimeterAssociations(
        frame.View(), associations, autonomy::common::math::SE3f());
    EXPECT_EQ(rejected.valid_unique_associations, 0);
    EXPECT_EQ(rejected.consistent_associations, 0);
}

TEST(RgbdMapReanchorMatcherTest, EnforcesFullXyzConsistencyAtFixedPixel) {
    SyntheticFrame frame =
        MakeFrame({cv::Point2f(50.0F, 50.0F)}, {2.1F});
    const auto map_point = MakeMapPoint(
        0, 12, OpaqueMapPoint(0x1030U), Eigen::Vector3f(0.0F, 0.0F, 2.0F));
    const auto at_limit = RgbdMapReanchorMatcher().Match(
        frame.View(), {map_point}, autonomy::common::math::SE3f());
    EXPECT_EQ(at_limit.diagnostics.candidates_after_geometry, 1);
    EXPECT_EQ(at_limit.diagnostics.assigned_matches, 1);

    frame.depths[0] = 2.101F;
    const auto outside_limit = RgbdMapReanchorMatcher().Match(
        frame.View(), {map_point}, autonomy::common::math::SE3f());
    EXPECT_EQ(outside_limit.diagnostics.candidates_after_geometry, 0);
    EXPECT_EQ(outside_limit.diagnostics.assigned_matches, 0);
}

TEST(RgbdMapReanchorMatcherTest, RequiresBilateralUnambiguousRatio) {
    const std::vector<Candidate> reverse_ambiguous = {
        {0, 10, 0, 10, 0.1, 0.01},
        {0, 10, 1, 20, 0.2, 0.01},
        {1, 11, 0, 11, 0.1, 0.01},
        {1, 11, 2, 30, 0.2, 0.01},
    };
    const auto rejected = internal::SelectRgbdMapReanchorCandidates(
        reverse_ambiguous, 0.75, 50);
    EXPECT_EQ(rejected.mutual_matches, 0);
    EXPECT_TRUE(rejected.assigned.empty());

    const std::vector<Candidate> unambiguous = {
        {0, 10, 0, 8, 0.2, 0.01},
        {0, 10, 1, 20, 0.1, 0.01},
        {1, 11, 0, 20, 0.1, 0.01},
        {1, 11, 1, 8, 0.2, 0.01},
    };
    const auto accepted = internal::SelectRgbdMapReanchorCandidates(
        unambiguous, 0.75, 50);
    EXPECT_EQ(accepted.mutual_matches, 2);
    ASSERT_EQ(accepted.assigned.size(), 2U);
    EXPECT_EQ(accepted.assigned[0].map_point_index, 0U);
    EXPECT_EQ(accepted.assigned[0].feature_index, 0);
    EXPECT_EQ(accepted.assigned[1].map_point_index, 1U);
    EXPECT_EQ(accepted.assigned[1].feature_index, 1);
}

TEST(RgbdMapReanchorMatcherTest,
     SelectsOneToOneMatchesDeterministicallyAcrossInputOrder) {
    std::vector<Candidate> candidates = {
        {5, 500, 2, 8, 4.0, 0.04},
        {2, 200, 7, 5, 1.0, 0.01},
        {9, 900, 4, 8, 2.0, 0.02},
    };
    const auto forward = internal::SelectRgbdMapReanchorCandidates(
        candidates, 0.75, 50);
    std::reverse(candidates.begin(), candidates.end());
    const auto reversed = internal::SelectRgbdMapReanchorCandidates(
        candidates, 0.75, 50);

    EXPECT_EQ(forward.mutual_matches, 3);
    EXPECT_EQ(reversed.mutual_matches, 3);
    EXPECT_EQ(SelectionSignature(forward), SelectionSignature(reversed));
    EXPECT_EQ(SelectionSignature(forward),
              (std::vector<std::tuple<std::size_t, int, int>>{
                  {2, 7, 5}, {9, 4, 8}, {5, 2, 8}}));
}

TEST(RgbdMapReanchorMatcherTest, NeverReusesAFeatureOrFallsBackAfterMutuality) {
    const std::vector<Candidate> candidates = {
        {0, 100, 0, 4, 0.1, 0.01},
        {0, 100, 1, 20, 0.1, 0.01},
        {1, 101, 0, 10, 0.1, 0.01},
        {1, 101, 2, 20, 0.1, 0.01},
    };
    const auto selection = internal::SelectRgbdMapReanchorCandidates(
        candidates, 0.75, 50);
    EXPECT_EQ(selection.mutual_matches, 1);
    ASSERT_EQ(selection.assigned.size(), 1U);
    EXPECT_EQ(selection.assigned[0].map_point_index, 0U);
    EXPECT_EQ(selection.assigned[0].feature_index, 0);
}

TEST(RgbdMapReanchorMatcherTest, RejectsInvalidViewsAndDeduplicatesMapIds) {
    SyntheticFrame frame =
        MakeFrame({cv::Point2f(50.0F, 50.0F)}, {2.0F});
    const Eigen::Vector3f world_point(0.0F, 0.0F, 2.0F);
    MapPoint* const expected = OpaqueMapPoint(0x2060U);
    std::vector<RgbdMapReanchorMatcher::MapPointView> map_points;
    map_points.push_back(
        MakeMapPoint(0, 20, OpaqueMapPoint(0x2010U), world_point));
    map_points.push_back(
        MakeMapPoint(1, 20, OpaqueMapPoint(0x2020U), world_point));

    auto wrong_map =
        MakeMapPoint(2, 21, OpaqueMapPoint(0x2030U), world_point);
    wrong_map.valid_for_expected_map = false;
    map_points.push_back(std::move(wrong_map));

    auto nonfinite =
        MakeMapPoint(3, 22, OpaqueMapPoint(0x2040U), world_point);
    nonfinite.world_point.x() = std::numeric_limits<float>::quiet_NaN();
    map_points.push_back(std::move(nonfinite));

    auto invalid_descriptor =
        MakeMapPoint(4, 23, OpaqueMapPoint(0x2050U), world_point);
    invalid_descriptor.descriptor_bank = cv::Mat::zeros(1, 31, CV_8U);
    map_points.push_back(std::move(invalid_descriptor));

    map_points.push_back(MakeMapPoint(5, 24, nullptr, world_point));
    map_points.push_back(MakeMapPoint(6, 25, expected, world_point));

    const auto result = RgbdMapReanchorMatcher().Match(
        frame.View(), map_points, autonomy::common::math::SE3f());
    EXPECT_TRUE(result.diagnostics.input_valid);
    EXPECT_EQ(result.diagnostics.input_map_points, 7);
    EXPECT_EQ(result.diagnostics.valid_unique_map_points, 1);
    EXPECT_EQ(result.diagnostics.assigned_matches, 1);
    ASSERT_EQ(result.associations.size(), 1U);
    EXPECT_EQ(result.associations[0], expected);
    EXPECT_FALSE(result.pnp_attempted);

    std::reverse(map_points.begin(), map_points.end());
    const auto reversed = RgbdMapReanchorMatcher().Match(
        frame.View(), map_points, autonomy::common::math::SE3f());
    EXPECT_EQ(reversed.diagnostics.valid_unique_map_points, 1);
    EXPECT_EQ(reversed.diagnostics.assigned_matches, 1);
    ASSERT_EQ(reversed.associations.size(), 1U);
    EXPECT_EQ(reversed.associations[0], expected);
}

TEST(RgbdMapReanchorMatcherTest, RefinesSyntheticMapPoseDeterministically) {
    SyntheticFrame frame;
    frame.intrinsics << 120.0F, 0.0F, 100.0F, 0.0F, 120.0F, 100.0F,
        0.0F, 0.0F, 1.0F;
    constexpr std::array<float, 9> kDepths = {
        2.0F, 2.3F, 2.1F, 2.7F, 2.2F, 3.1F, 2.4F, 3.0F, 2.6F};
    constexpr std::array<float, 3> kCoordinates = {30.0F, 80.0F, 130.0F};
    std::vector<RgbdMapReanchorMatcher::MapPointView> map_points;
    frame.descriptors = cv::Mat::zeros(9, 32, CV_8U);
    frame.keypoints.reserve(9);
    frame.depths.reserve(9);
    map_points.reserve(9);
    int index = 0;
    for (float y : kCoordinates) {
        for (float x : kCoordinates) {
            const cv::Point2f pixel(x, y);
            const float depth = kDepths[index];
            frame.keypoints.emplace_back(pixel, 1.0F);
            frame.depths.push_back(depth);
            map_points.push_back(MakeMapPoint(
                index, 100 + index, OpaqueMapPoint(0x3000U + 0x10U * index),
                BackProject(pixel, depth, frame.intrinsics)));
            ++index;
        }
    }

    const autonomy::common::math::SE3f provisional_camera_from_world(
        Eigen::Matrix3f::Identity(), Eigen::Vector3f(0.005F, -0.004F, 0.002F));
    const RgbdMapReanchorMatcher matcher;
    const auto first = matcher.Match(frame.View(), map_points,
                                     provisional_camera_from_world);
    const auto second = matcher.Match(frame.View(), map_points,
                                      provisional_camera_from_world);

    EXPECT_EQ(first.diagnostics.assigned_matches, 9);
    EXPECT_TRUE(first.pnp_attempted);
    ASSERT_TRUE(first.pnp_succeeded);
    EXPECT_EQ(first.pnp_inliers, 9);
    EXPECT_EQ(first.refined_diagnostics.assigned_matches, 9);
    EXPECT_EQ(first.refined_associations, second.refined_associations);
    EXPECT_TRUE(first.refined_camera_from_world.matrix().isApprox(
        second.refined_camera_from_world.matrix(), 1.0e-6F));
    EXPECT_LT(first.refined_diagnostics.reprojection_rmse_pixels, 1.0)
        << "translation="
        << first.refined_camera_from_world.translation().transpose();
    EXPECT_LT(first.refined_diagnostics.xyz_rmse_meters, 0.02)
        << "translation="
        << first.refined_camera_from_world.translation().transpose();
    // Eigen::isApprox(Zero) is a relative comparison and rejects any nonzero
    // floating-point roundoff. Use an absolute norm for the zero translation.
    EXPECT_LT(first.refined_camera_from_world.translation().norm(), 1.0e-3F);
    EXPECT_LT((first.refined_camera_from_world.rotationMatrix() -
               Eigen::Matrix3f::Identity())
                  .norm(),
              0.01F);
}

}  // namespace
}  // namespace autonomy::localization::atlas
