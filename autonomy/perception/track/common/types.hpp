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
 * @brief Shared topic constants and result types for ground-robot tracking.
 */

#pragma once

#include <array>
#include <string>
#include <vector>

#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>

namespace autonomy::perception::track {

constexpr char kTrackVelocityCommandTopic[] = "/cmd_vel";
constexpr char kTrackDebugPathTopic[] = "/tracking/path";
constexpr char kTrackResetTopic[] = "/tracking/reset";
constexpr char kTrackPauseTopic[] = "/tracking/pause";

/**
 * @struct autonomy::perception::track::PlanarPva
 * @brief Planar position / velocity / acceleration in the body frame.
 */
struct PlanarPva {
  double position_x_m{0.0};
  double position_y_m{0.0};
  double velocity_x_mps{0.0};
  double velocity_y_mps{0.0};
  double acceleration_x_mps2{0.0};
  double acceleration_y_mps2{0.0};
};

/**
 * @struct autonomy::perception::track::MincoBoundary
 * @brief 2-piece planar MINCO boundary conditions (YOPO-MINCO adapted).
 */
struct MincoBoundary {
  PlanarPva head;
  PlanarPva tail;
  double inner_x_m{0.0};
  double inner_y_m{0.0};
  std::array<double, 2> durations_s{{1.0, 1.0}};
};

/**
 * @struct autonomy::perception::track::MotionPrimitiveHypothesis
 * @brief One lattice-cell hypothesis after decoding into the body frame.
 *
 * In simple mode, terminal_* and body velocities are populated.
 * In minco mode, |minco| is also filled and terminal_* mirrors the MINCO tail.
 */
struct MotionPrimitiveHypothesis {
  double terminal_x_m{0.0};
  double terminal_y_m{0.0};
  double linear_velocity_mps{0.0};
  double yaw_rate_rps{0.0};
  double trajectory_cost{0.0};
  double objectness_score{0.0};
  int lattice_index{-1};
  bool uses_minco{false};
  MincoBoundary minco;
};

/**
 * @struct autonomy::perception::track::TrackResult
 * @brief Output of one tracking cycle: selected hypothesis and velocity command.
 */
struct TrackResult {
  bool has_target{false};
  int selected_lattice_index{-1};
  MotionPrimitiveHypothesis selected;
  std::vector<MotionPrimitiveHypothesis> hypotheses;
  automsgs::msgs::geometry_msgs::TwistStamped velocity_command;
  // Optional dense samples of the selected MINCO path (body frame) for viz.
  std::vector<std::array<double, 2>> debug_path_xy_m;
};

}  // namespace autonomy::perception::track
