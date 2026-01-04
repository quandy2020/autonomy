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

#include "autonomy/tasks/behavior_tree/plugins/condition/are_error_codes_present_condition.hpp"
#include "autonomy/tasks/navigator/proto/action.pb.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

class WouldAControllerRecoveryHelp : public AreErrorCodesPresent
{
    using Action = proto::FollowPathAction;
    using ActionResult = Action::Result;

public:
    WouldAControllerRecoveryHelp(const std::string& condition_name,
                                 const BT::NodeConfiguration& conf);

    WouldAControllerRecoveryHelp() = delete;
};

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy