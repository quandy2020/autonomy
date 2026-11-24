/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file constants.hpp
 * @brief Constants.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_CONSTANTS_HPP_
#define AUTONOMY_TASK_NAVIGATION_CONSTANTS_HPP_

#include <string>
#include <vector>

namespace autonomy {
namespace task {
namespace navigation {

// Autolink node name used by the navigator when it creates its node.
constexpr char kNavigatorNodeName[] = "navigator";

// BT navigator id that drives a single pose goal.
constexpr char kNavigatorNavigateToPose[] = "/navigate_to_pose";

// BT navigator id that drives a goal sequence.
constexpr char kNavigatorNavigateThroughPoses[] = "/navigate_through_poses";

// Smallest path a follower accepts; anything shorter is treated as arrival.
inline constexpr std::size_t kMinPathPoses = 2;

// Timeout applied to a direct (non-BT) navigation request, seconds.
inline constexpr double kDirectNavDefaultTimeoutSec = 300.0;

// Poll period while a behavior tree waits for an action result, milliseconds.
inline constexpr int kBtWaitPollMs = 50;

// Tick rate of the navigator spin loop, hertz.
inline constexpr int kSpinRateHz = 10;

}  // namespace navigation
}  // namespace task
}  // namespace autonomy

#endif  // AUTONOMY_TASK_NAVIGATION_CONSTANTS_HPP_
