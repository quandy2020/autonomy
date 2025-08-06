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

// #include "cyber/cyber.h"
#include <string>
#include <unordered_map>

#include "autonomy/common/macros.hpp"
#include "autonomy/map/map_server.hpp"
#include "autonomy/planning/common/planner_interface.hpp"
#include "autonomy/planning/common/planner_server_interface.hpp"

namespace autonomy {
namespace planning {

class PlannerServer : common::PlannerServerInterface
{
public:
    using PlannerMap = std::unordered_map<std::string, common::PlannerInterface::SharedPtr>;

    /**
     * Define TaskBridge::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(PlannerServer)

    /**
     * @brief A constructor for autonomy::planning::PlannerServer
     * @param options Additional options to control creation of the node.
     */
    explicit PlannerServer();

    /**
     * @brief
     */
    explicit PlannerServer(map::common::MapInterface::SharedPtr map);

    /**
     * @brief A Destructor for autonomy::planning::PlannerServer
     */
    ~PlannerServer();

private:

    // All planners
    PlannerMap planners_;
};

}  // namespace planning
}  // namespace autonomy