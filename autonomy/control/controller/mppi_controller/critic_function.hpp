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

#include <memory>
#include <string>

#include "autolink/autolink.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/control/controller/mppi_controller/critic_data.hpp"
#include "autonomy/control/proto/mppi_controller.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {

/**
 * @class mppi::critics::CollisionCost
 * @brief Utility for storing cost information
 */
struct CollisionCost {
    float cost{0};
    bool using_footprint{false};
};

/**
 * @class mppi::critics::CriticFunction
 * @brief Abstract critic objective function to score trajectories
 */
class CriticFunction
{
public:
    /**
     * @brief Constructor for mppi::critics::CriticFunction
     */
    CriticFunction() = default;

    /**
     * @brief Destructor for mppi::critics::CriticFunction
     */
    virtual ~CriticFunction() = default;

    /**
     * @brief Configure critic on bringup
     * @param parent WeakPtr to node
     * @param parent_name name of the controller
     * @param name Name of plugin
     * @param costmap_ros Costmap2DROS object of environment
     * @param dynamic_parameter_handler Parameter handler object
     */
    void configure(
        std::shared_ptr<autolink::Node> parent, const std::string& parent_name,
        const std::string& name,
        std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_ros,
        const proto::MPPIControllerOptions* options) {
        parent_ = parent;
        name_ = name;
        parent_name_ = parent_name;
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();
        options_ = options;

        // enabled_ will be set by each critic subclass from proto options
        enabled_ = true;  // Default, will be overridden by specific critic

        initialize();
    }

    /**
     * @brief Main function to score trajectory
     * @param data Critic data to use in scoring
     */
    virtual void score(CriticData& data) = 0;

    /**
     * @brief Initialize critic
     */
    virtual void initialize() = 0;

    /**
     * @brief Get name of critic
     */
    std::string getName() {
        return name_;
    }

protected:
    bool enabled_;
    std::string name_, parent_name_;
    std::shared_ptr<autolink::Node> parent_;
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_ros_;
    map::costmap_2d::Costmap2D* costmap_{nullptr};

    const proto::MPPIControllerOptions* options_;
};

}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy