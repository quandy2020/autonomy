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

#include "autonomy/control/controller/mppi_controller/critic_data.hpp"
#include "autonomy/control/proto/mppi_controller.pb.h"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {
namespace critics {

class CriticFunction
{
public:
    virtual ~CriticFunction() = default;

    virtual void Configure(
        const proto::MPPIControllerOptions& options,
        std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) = 0;

    virtual void score(CriticData& data) = 0;

    virtual std::string name() const = 0;

protected:
    bool enabled_{true};
};

}  // namespace critics
}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
