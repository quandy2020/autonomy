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

#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace visualization {
namespace common { 

class VisualizationInterface
{
public:
    /**
     * Define VisualizationInterface::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(VisualizationInterface)

    /**
     * @brief A constructor for autonomy::visualization::common::VisualizationInterface
     * @param options Additional options to control creation of the node.
     */
    VisualizationInterface();

    /**
     * @brief A Destructor for autonomy::visualization::common::VisualizationInterface
     */
    virtual ~VisualizationInterface();

};

}   // namespace common 
}   // namespace visualization
}   // namespace autonomy