/**
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

// Action client/server API. GoalStatus / terminal ResultCode numeric values and
// state names follow ROS 2 actions (action_msgs/GoalStatus); see types.hpp and
// https://design.ros2.org/articles/actions.html

#pragma once

#include "autolink/action/client.hpp"
#include "autolink/action/client_goal_handle.hpp"
#include "autolink/action/create_client.hpp"
#include "autolink/action/create_server.hpp"
#include "autolink/action/exceptions.hpp"
#include "autolink/action/server.hpp"
#include "autolink/action/server_goal_handle.hpp"
#include "autolink/action/types.hpp"
