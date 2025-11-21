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

#include "autonomy/system/proto/autonomy_options.pb.h"

#include "autonomy/map/map_server.hpp"
#include "autonomy/control/controller_server.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/visualization/visualization_server.hpp"

namespace autonomy {
namespace system { 

class AutonomyNode
{
public:
    using TfBuffer = autonomy::transform::Buffer;

    /**
     * Define AutonomyNode::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(AutonomyNode)

    /**
     * @brief A constructor for autonomy::system::AutonomyNode
     * @param options Additional options to control creation of the node.
     */
    AutonomyNode() = default;

    /**
     * @brief Destroy the Autonomy Node object
     * 
     * @param options 
     */
    explicit AutonomyNode(const proto::AutonomyOptions& options);

    /**
     * @brief A Destructor for autonomy::system::AutonomyNode
     */
    ~AutonomyNode() = default;

    /**
     * @brief Starts autonomy tasks
     */
    void Start();

    /**
     * @brief Shutdown autonomy tasks
     */
    void WaitForShutdown();

    // /**
    //  * @brief Get bridge_server
    //  * 
    //  * @return bridge::BridgeServer pointer
    //  */
    // bridge::BridgeServer* bridge_server() { return bridge_server_.get(); }

    // /**
    //  * @brief Get map_server
    //  * 
    //  * @return pointer
    //  */
    // map::MapServer* map_server() { return map_server_.get(); }

    // /**
    //  * @brief Get controller_server
    //  * 
    //  * @return control::ControllerServer pointer
    //  */
    // control::ControllerServer* controller_server() { return controller_server_.get(); }

    // /**
    //  * @brief Get planner_server
    //  * 
    //  * @return planning::PlannerServer pointer
    //  */
    // planning::PlannerServer* planner_server() { return planner_server_.get();}

private:
    // Configuration for auronomy options
    proto::AutonomyOptions options_;

    // Tf2 buffer
    TfBuffer* tf_buffer_{nullptr};

    // map
    map::MapServer::SharedPtr map_server_{nullptr};

    // controller
    control::ControllerServer::SharedPtr controller_server_{nullptr};
    
    // planner
    planning::PlannerServer::SharedPtr planner_server_{nullptr};

    // visualization
    visualization::VisualizationServer::SharedPtr visual_server_{nullptr};
};

AutonomyNode::UniquePtr CreateAutonomy(const proto::AutonomyOptions& options);

}   // namespace system
}   // namespace autonomy