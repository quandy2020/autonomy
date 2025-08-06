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

#include <unordered_map>

#include "autonomy/tasks/proto/task_options.pb.h"

#include "autonomy/common/macros.hpp"
#include "autonomy/map/map_server.hpp"
#include "autonomy/bridge/bridge_server.hpp"
#include "autonomy/control/controller_server.hpp"
#include "autonomy/planning/planner_server.hpp"

namespace autonomy {
namespace tasks { 

class TaskBridge
{
public:
    /**
     * Define TaskBridge::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(TaskBridge)

    /**
     * @brief A constructor for autonomy::tasks::TaskBridge
     * @param options Additional options to control creation of the node.
     */
    explicit TaskBridge();

    /**
     * @brief A Destructor for autonomy::tasks::MapServer
     */
    ~TaskBridge();

    /**
     * @brief Shutdown 
     */
    void Shutdown();

    /**
     * @brief Handle user navigation task commands
     * 
     * @return True or false
     */
    bool HandleNavigationCommands();

    /**
     * @brief Get bridge_server
     * 
     * @return bridge::BridgeServer pointer
     */
    bridge::BridgeServer* bridge_server() { return bridge_server_.get(); }

    /**
     * @brief Get map_server
     * 
     * @return pointer
     */
    map::MapServer* map_server() { return map_server_.get(); }

    /**
     * @brief Get controller_server
     * 
     * @return control::ControllerServer pointer
     */
    control::ControllerServer* controller_server() { return controller_server_.get(); }

    /**
     * @brief Get planner_server
     * 
     * @return planning::PlannerServer pointer
     */
    planning::PlannerServer* planner_server() { return planner_server_.get();}

private:

    // bridge
    bridge::BridgeServer::SharedPtr bridge_server_{nullptr};

    // costmap
    map::MapServer::SharedPtr map_server_{nullptr};

    // controller
    control::ControllerServer::SharedPtr controller_server_{nullptr};
    
    // planner
    planning::PlannerServer::SharedPtr planner_server_{nullptr};

    // tasks options
    const proto::TaskOptions options_;
};

}   // namespace tasks
}   // namespace autonomy