/*
 * Copyright 2026 The Navigation Authors (navigation)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file blackboard_keys.hpp
 * @brief Stable blackboard entry names shared by nodes and tests.
 */

#pragma once

namespace navigation
{
namespace behavior
{
namespace keys
{

inline constexpr char kContext[] = "node_context";
inline constexpr char kGoal[] = "goal";
inline constexpr char kPose[] = "pose";
inline constexpr char kPoseValid[] = "pose_valid";
inline constexpr char kSafetyMode[] = "safety_mode";
inline constexpr char kModuleHealth[] = "module_health";
inline constexpr char kGoalReachable[] = "goal_reachable";
inline constexpr char kRecoveryCount[] = "recovery_count";
inline constexpr char kOperatorAssist[] = "operator_assist";

}  // namespace keys
}  // namespace behavior
}  // namespace navigation
