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
 * @file
 * @brief Shared defaults and runtime contracts for ground-robot tracking.
 *
 * Supports two trajectory representations compatible with YOPO-Simple and
 * YOPO-MINCO (planar adaptation):
 *   - simple: prediction channels = 6
 *   - minco:  prediction channels = 12 (2-piece planar MINCO_S3NU params)
 */

#pragma once

#include <string_view>

namespace autonomy::perception::track {

constexpr char kOnnxDepthInputName[] = "depth";
constexpr char kOnnxObservationInputName[] = "observation";
constexpr char kOnnxPredictionOutputName[] = "prediction";

constexpr char kTrajectoryModeSimple[] = "simple";
constexpr char kTrajectoryModeMinco[] = "minco";

// Simple mode: yaw_offset, range_offset, vx, wz, score, objectness.
constexpr int kSimplePredictionChannelCount = 6;
// MINCO mode: inner(2) + tail_pos(2) + tail_vel(2) + tail_acc(2) + durations(2)
//             + score + objectness.
constexpr int kMincoPredictionChannelCount = 12;
constexpr int kObservationChannelCount = 4;
constexpr int kMincoPieceCount = 2;
constexpr int kMincoSpatialDim = 2;  // planar x,y

namespace defaults {

constexpr int kImageHeight = 96;
constexpr int kImageWidth = 160;
constexpr int kHorizontalBinCount = 5;
constexpr int kVerticalBinCount = 1;
constexpr double kCameraHorizontalFieldOfViewDeg = 90.0;
constexpr double kPlanningHorizonM = 3.0;
constexpr double kMaxLinearVelocityMps = 0.5;
constexpr double kMaxYawRateRps = 1.0;
constexpr double kMaxAccelerationMps2 = 1.0;
constexpr double kObjectnessThreshold = 0.35;
constexpr double kSuppressionAngleDeg = 15.0;
constexpr double kFollowDistanceM = 1.5;
constexpr double kDepthScaleToMetres = 0.001;
constexpr double kControlHz = 10.0;
constexpr bool kAllowHeuristicFallback = true;
constexpr char kTrajectoryMode[] = "simple";
constexpr double kMincoPieceDurationS = 1.0;
constexpr double kMincoSampleHorizonS = 0.5;

}  // namespace defaults

constexpr double kDepthNormalizeHorizonFactor = 2.0;
constexpr double kSoftplusClamp = 20.0;
constexpr double kMincoMinPieceDurationS = 0.1;

constexpr double kRangeErrorGain = 0.8;
constexpr double kBearingErrorGain = 1.5;
constexpr double kNearZeroVelocityEpsilon = 1e-3;
constexpr double kReverseInhibitStandoffFraction = 0.5;

constexpr float kHeuristicMinValidNormalizedDepth = 0.02f;
constexpr float kHeuristicMaxPersonNormalizedDepth = 0.95f;
constexpr double kHeuristicMinPersonRangeM = 0.3;
constexpr double kHeuristicMaxPersonRangeStandoffFactor = 3.0;
constexpr double kHeuristicYawCostWeight = 0.2;
constexpr double kHeuristicBackgroundObjectness = 0.05;
constexpr double kHeuristicRowBandLowFraction = 0.25;
constexpr double kHeuristicRowBandHighFraction = 0.75;

inline bool IsMincoTrajectoryMode(std::string_view mode) {
  return mode == kTrajectoryModeMinco;
}

inline int PredictionChannelCountForMode(std::string_view mode) {
  return IsMincoTrajectoryMode(mode) ? kMincoPredictionChannelCount
                                     : kSimplePredictionChannelCount;
}

}  // namespace autonomy::perception::track
