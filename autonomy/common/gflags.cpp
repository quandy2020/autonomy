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

#include "autonomy/common/gflags.hpp"

namespace autonomy {
namespace common {

DEFINE_bool(verbose, false, "Show autnomy verison info");

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Autonomy installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");

DEFINE_bool(run_navigate_to_pose, false,
            "If true, run NavigateToPose once at startup (see nav_goal_*).");
DEFINE_double(nav_goal_x, 0.0, "Navigation goal x in global_frame.");
DEFINE_double(nav_goal_y, 0.0, "Navigation goal y in global_frame.");
DEFINE_double(nav_goal_yaw, 0.0, "Navigation goal yaw (rad) in global_frame.");
DEFINE_bool(mock_static_tf, true,
            "Mock localization: static map->odom TF plus cmd_vel integration "
            "for odom->base_link during NavigateToPose (single-process demo).");

}  // namespace common
}  // namespace autonomy