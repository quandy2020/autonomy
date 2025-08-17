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


#include "autonomy/visualization/proto/visualization_options.pb.h"

#include "autonomy/common/macros.hpp"
#include "autonomy/visualization/common/visualization_interface.hpp"

namespace autonomy {
namespace visualization { 

class VisualizationServer : public common::VisualizationInterface
{
public:
    /**
     * Define VisualizationServer::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(VisualizationServer)

    /**
     * @brief A constructor for autonomy::visualization::VisualizationServer
     * @param options Additional options to control creation of the node.
     */
    explicit VisualizationServer();

    /**
     * @brief A Destructor for autonomy::visualization::VisualizationServer
     */
    ~VisualizationServer();

    /**
     * @brief Shutdown 
     */
    void Shutdown();

private:

    // options for config
    proto::VisualizationOptions options_;


};

}   // namespace visualization
}   // namespace autonomy