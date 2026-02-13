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

#include "autonomy/tasks/behavior_tree/plugins/condition/are_error_codes_present_condition.hpp"
#include "autonomy/tasks/navigator/proto/action.pb.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

class WouldAPlannerRecoveryHelp : public AreErrorCodesPresent
{
    using Action = proto::ComputePathToPoseAction;
    using ActionResult = Action::Result;
    using ThroughAction = proto::ComputePathThroughPosesAction;
    using ThroughActionResult = ThroughAction::Result;

public:
    WouldAPlannerRecoveryHelp(const std::string& condition_name, const BT::NodeConfiguration& conf);

    WouldAPlannerRecoveryHelp() = delete;
};

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy