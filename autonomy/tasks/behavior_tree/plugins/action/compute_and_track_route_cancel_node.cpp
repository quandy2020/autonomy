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

#include "autonomy/tasks/behavior_tree/plugins/action/compute_and_track_route_cancel_node.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

ComputeAndTrackRouteCancel::ComputeAndTrackRouteCancel(
    const std::string& xml_tag_name, const std::string& action_name,
    const BT::NodeConfiguration& conf)
    : BtCancelActionNode<proto::ComputeAndTrackRouteAction>(
          xml_tag_name, action_name, conf) {}

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
    BT::NodeBuilder builder = [](const std::string& name,
                                 const BT::NodeConfiguration& config) {
        return std::make_unique<autonomy::tasks::behavior_tree::plugins::
                                    action::ComputeAndTrackRouteCancel>(
            name, "compute_and_track_route", config);
    };

    factory.registerBuilder<autonomy::tasks::behavior_tree::plugins::action::
                                ComputeAndTrackRouteCancel>(
        "CancelComputeAndTrackRoute", builder);
}
