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

#include "atlas/closure/loop_closer.hpp"

#include <gtest/gtest.h>

#include <array>

#include <Eigen/Geometry>

namespace autonomy::localization::atlas {
namespace {

TEST(LoopCloserTest, ChoosesSensorAndImuStageSpecificSim3ScalePolicy) {
    // Pure monocular keeps scale free.
    EXPECT_FALSE(internal::ShouldFixSim3Scale(false, false, false));

    // Metric stereo/RGB-D keeps scale fixed even before inertial BA2.
    EXPECT_TRUE(internal::ShouldFixSim3Scale(true, false, false));

    // Monocular-inertial scale remains free until BA2 and fixed afterwards.
    EXPECT_FALSE(internal::ShouldFixSim3Scale(true, true, false));
    EXPECT_TRUE(internal::ShouldFixSim3Scale(true, true, true));

    // An explicitly free-scale configuration always takes precedence.
    EXPECT_FALSE(internal::ShouldFixSim3Scale(false, true, true));
}

TEST(LoopCloserTest, ComposeRefinedWorldSim3PreservesOptimizedTransform) {
    constexpr double kScale = 1.7;
    const Eigen::Quaterniond current_from_matched_rotation(
        Eigen::AngleAxisd(0.47, Eigen::Vector3d(1.0, -2.0, 0.5).normalized()));
    const Eigen::Vector3d current_from_matched_translation(0.2, -0.4, 1.1);
    const g2o::Sim3 current_from_matched(current_from_matched_rotation,
                                         current_from_matched_translation,
                                         kScale);

    const Eigen::Quaterniond matched_from_world_rotation(
        Eigen::AngleAxisd(-0.31, Eigen::Vector3d(-0.3, 0.7, 1.0).normalized()));
    const Eigen::Vector3d matched_from_world_translation(2.0, -1.5, 0.8);
    const autonomy::common::math::SE3d matched_from_world(
        matched_from_world_rotation, matched_from_world_translation);

    const g2o::Sim3 current_from_world = internal::ComposeRefinedWorldSim3(
        current_from_matched, matched_from_world);

    const Eigen::Quaterniond expected_rotation =
        (current_from_matched_rotation * matched_from_world_rotation)
            .normalized();
    const Eigen::Vector3d expected_translation =
        kScale *
            (current_from_matched_rotation * matched_from_world_translation) +
        current_from_matched_translation;
    EXPECT_NEAR(current_from_world.scale(), kScale, 1.0e-12);
    EXPECT_TRUE(current_from_world.rotation().toRotationMatrix().isApprox(
        expected_rotation.toRotationMatrix(), 1.0e-12));
    EXPECT_TRUE(current_from_world.translation().isApprox(expected_translation,
                                                          1.0e-12));

    const std::array<Eigen::Vector3d, 3> world_points = {
        Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.2, -3.4, 0.5),
        Eigen::Vector3d(-2.0, 0.25, 4.5)};
    for (const Eigen::Vector3d& world_point : world_points) {
        const Eigen::Vector3d expected =
            current_from_matched.map(matched_from_world * world_point);
        EXPECT_TRUE(
            current_from_world.map(world_point).isApprox(expected, 1.0e-12));
    }
}

TEST(LoopCloserTest, ComposeRefinedWorldSim3PreservesUnitScaleAndOrder) {
    const Eigen::Quaterniond current_from_matched_rotation(
        Eigen::AngleAxisd(0.36, Eigen::Vector3d::UnitX()));
    const Eigen::Vector3d current_from_matched_translation(0.5, -0.2, 0.9);
    const g2o::Sim3 current_from_matched(current_from_matched_rotation,
                                         current_from_matched_translation, 1.0);

    const Eigen::Quaterniond matched_from_world_rotation(
        Eigen::AngleAxisd(-0.42, Eigen::Vector3d::UnitY()));
    const Eigen::Vector3d matched_from_world_translation(-1.0, 0.7, 0.3);
    const autonomy::common::math::SE3d matched_from_world(
        matched_from_world_rotation, matched_from_world_translation);

    const g2o::Sim3 current_from_world = internal::ComposeRefinedWorldSim3(
        current_from_matched, matched_from_world);
    EXPECT_NEAR(current_from_world.scale(), 1.0, 1.0e-12);

    const std::array<Eigen::Vector3d, 2> world_points = {
        Eigen::Vector3d(0.4, -1.2, 2.5), Eigen::Vector3d(-3.0, 0.8, 1.1)};
    for (const Eigen::Vector3d& world_point : world_points) {
        const Eigen::Vector3d expected =
            current_from_matched.map(matched_from_world * world_point);
        EXPECT_TRUE(
            current_from_world.map(world_point).isApprox(expected, 1.0e-12));
    }
}

}  // namespace
}  // namespace autonomy::localization::atlas
