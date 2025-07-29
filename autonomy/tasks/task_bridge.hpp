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

#include "autonomy/common/macros.hpp"
#include "autonomy/map/map_server.hpp"
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


private:

    // costmap
    map::MapServer::SharedPtr map_server_{nullptr};

    // controller
    control::ControllerServer::SharedPtr controller_server_{nullptr};
    
    // planner
    planning::PlannerServer::SharedPtr planner_server_{nullptr};
};

}   // namespace tasks
}   // namespace autonomy