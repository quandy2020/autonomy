/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

/**
 * @file controller_adjustment_base.hpp
 * @brief The base class for all controllers adjustment.
 */


#include <vector>

#include <bolza/core/types.hpp>
#include <bolza/core/control/linear_controller.hpp>

namespace bolza {

/**
 * The base class for all controllers adjustment.
 */
class ControllerAdjustmentBase {
 public:
  /**
   * Default constructor.
   */
  ControllerAdjustmentBase() = default;

  /**
   * Default destructor.
   */
  virtual ~ControllerAdjustmentBase() = default;

    /**
   * @brief.
   *
   * Adjust the controller based on the last changes in the logic rules.
   *
   * /
   * @param [in] eventTimes: The new event times.
   * @param [in] controllerEventTimes: The control policy stock's event times.
   * @param [in,out] controllersStock: Controllers Stock.
   */
  virtual void AdjustController(const scalar_array_t& eventTimes, const scalar_array_t& controllerEventTimes,
                                std::vector<LinearController>& controllersStock) = 0;
};

}  // namespace bolza
