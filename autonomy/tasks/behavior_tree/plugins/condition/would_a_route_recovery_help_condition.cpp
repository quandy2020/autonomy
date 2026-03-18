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

#include "autonomy/tasks/behavior_tree/plugins/condition/would_a_route_recovery_help_condition.hpp"

#include "autonomy/tasks/navigator/proto/action.pb.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace condition {

WouldARouteRecoveryHelp::WouldARouteRecoveryHelp(const std::string& condition_name, const BT::NodeConfiguration& conf)
    : AreErrorCodesPresent(condition_name, conf) {
  error_codes_to_check_ = {static_cast<uint16_t>(proto::COMPUTE_AND_TRACK_ROUTE_ERROR_UNKNOWN),
                           static_cast<uint16_t>(proto::COMPUTE_AND_TRACK_ROUTE_ERROR_NO_VALID_ROUTE),
                           static_cast<uint16_t>(proto::COMPUTE_AND_TRACK_ROUTE_ERROR_TIMEOUT)};
}

}  // namespace condition
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<autonomy::tasks::behavior_tree::plugins::condition::WouldARouteRecoveryHelp>(
      "WouldARouteRecoveryHelp");
}
