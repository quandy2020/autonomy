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
#include <vector>

#include "autolink/autolink.hpp"
#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/control/controller/mppi_controller/critic_data.hpp"
#include "autonomy/control/controller/mppi_controller/critic_function.hpp"
#include "autonomy/control/controller/mppi_controller/tools/utils.hpp"
#include "autonomy/control/proto/mppi_controller.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi_controller {

/**
 * @class mppi::CriticManager
 * @brief Manager of objective function plugins for scoring trajectories
 */
class CriticManager
{
public:
    typedef std::vector<std::unique_ptr<CriticFunction>> Critics;
    /**
     * @brief Constructor for mppi::CriticManager
     */
    CriticManager() = default;

    /**
     * @brief Virtual Destructor for mppi::CriticManager
     */
    virtual ~CriticManager() = default;

    /**
     * @brief Configure critic manager on bringup and load plugins
     * @param parent WeakPtr to node
     * @param name Name of plugin
     * @param costmap_ros Costmap2DROS object of environment
     * @param dynamic_parameter_handler Parameter handler object
     */
    void configure(std::shared_ptr<autolink::Node> parent,
                   const std::string& name,
                   std::shared_ptr<map::costmap_2d::Costmap2DWrapper>,
                   const proto::MPPIControllerOptions*);

    /**
     * @brief Score trajectories by the set of loaded critic functions
     * @param CriticData Struct of necessary information to pass to the critic
     * functions
     */
    void evalTrajectoriesScores(CriticData& data) const;

protected:
    /**
     * @brief Get parameters (critics to load)
     */
    void getParams();

    /**
     * @brief Load the critic plugins
     */
    virtual void loadCritics();

    /**
     * @brief Get full-name namespaced critic IDs
     */
    std::string getFullName(const std::string& name);

protected:
    std::shared_ptr<autolink::Node> parent_;
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_ros_;
    std::string name_;

    const proto::MPPIControllerOptions* options_;
    std::vector<std::string> critic_names_;
    autolink::class_loader::ClassLoaderManager loader_;
    Critics critics_;
};

}  // namespace mppi_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy