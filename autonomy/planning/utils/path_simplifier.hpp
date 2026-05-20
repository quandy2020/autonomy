/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#pragma once

#include "autonomy/commsgs/planning_msgs.hpp"

namespace autonomy {
namespace planning {
namespace utils {

/** Douglas-Peucker path simplification. epsilon <= 0 returns path unchanged. */
commsgs::planning_msgs::Path SimplifyPath(
    const commsgs::planning_msgs::Path& path, double epsilon);

}  // namespace utils
}  // namespace planning
}  // namespace autonomy
