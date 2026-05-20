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

#include <chrono>
#include <memory>

#include "autonomy/tasks/common/feedback_utils.hpp"
#include "autonomy/tasks/common/task_context.hpp"
#include "autonomy/tasks/proto/task_options.pb.h"
#include "behaviortree_cpp/blackboard.h"

namespace autonomy {
namespace tasks {
namespace common {

/** Initialize blackboard keys used by navigate_to_pose.xml. */
void SetupNavigateToPoseBlackboard(
    const BT::Blackboard::Ptr& blackboard,
    const std::shared_ptr<TaskContext>& task_context,
    const proto::TaskOptions& options, const FeedbackUtils& feedback);

/** Dispatch blackboard setup by navigator id (see config/tasks/tasks.lua). */
void SetupNavigatorBlackboard(
    const std::string& navigator_id, const BT::Blackboard::Ptr& blackboard,
    const std::shared_ptr<TaskContext>& task_context,
    const proto::TaskOptions& options, const FeedbackUtils& feedback);

}  // namespace common
}  // namespace tasks
}  // namespace autonomy
