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

#include <string>

#include "autonomy/tasks/proto/task_options.pb.h"

namespace autonomy {
namespace common {
class LuaParameterDictionary;
}
namespace tasks {

/** Runtime overrides from ROS parameters or AutonomyNode::Configure(). */
struct RuntimeOptions {
    bool enable_bt_tasks{true};
    bool use_bt_navigation{true};
    std::string config_directory;
    std::string planner_id;
    std::string controller_id;
    std::string goal_checker_id;
    std::string progress_checker_id;
    std::string global_frame;
    double goal_tolerance{0.15};
};

/** Load TaskOptions from the Lua `tasks` table (not the autonomy.lua root). */
proto::TaskOptions LoadTaskOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary);

/** Load TaskOptions from a root dict that contains a `tasks` sub-table, or empty. */
proto::TaskOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary);

/** Create TaskOptions from configuration directory and Lua basename (e.g.
 * tasks/tasks.lua). */
proto::TaskOptions CreateOptions(const std::string& configuration_directory,
                                 const std::string& configuration_basename);

}  // namespace tasks
}  // namespace autonomy
