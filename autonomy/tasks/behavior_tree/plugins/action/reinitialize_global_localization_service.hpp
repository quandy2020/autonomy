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

#include "autonomy/tasks/behavior_tree/behavior_tree_service_node.hpp"
#include "autonomy/tasks/navigator/proto/srv.pb.h"
#include "behaviortree_cpp/action_node.h"

namespace autonomy {
namespace tasks {
namespace behavior_tree {
namespace plugins {
namespace action {

/**
 * @brief A nav2_behavior_tree::BtServiceNode class that wraps
 * nav2_msgs::srv::Empty
 * @note This is an Asynchronous (long-running) node which may return a RUNNING
 * state while executing. It will re-initialize when halted.
 */
class ReinitializeGlobalLocalizationService : public BtServiceNode<proto::Empty>
{
public:
    /**
     * @brief A constructor for
     * nav2_behavior_tree::ReinitializeGlobalLocalizationService
     * @param service_node_name Service name this node creates a client for
     * @param conf BT node configuration
     */
    ReinitializeGlobalLocalizationService(const std::string& service_node_name,
                                          const BT::NodeConfiguration& conf);
};

}  // namespace action
}  // namespace plugins
}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy
