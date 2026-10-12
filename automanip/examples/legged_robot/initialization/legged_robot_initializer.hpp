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

#include <bolza/pinocchio/centroidal/centroidal_model_info.hpp>
#include <bolza/core/initialization/initializer.hpp>

#include "examples/legged_robot/reference_manager/switched_model_reference_manager.hpp"

namespace bolza {
namespace legged_robot {

class LeggedRobotInitializer final : public Initializer {
 public:
  /*
   * Constructor
   * @param [in] info : The centroidal model information.
   * @param [in] referenceManager : Switched system reference manager.
   * @param [in] extendNormalizedMomentum: If true, it extrapolates the normalized momenta; otherwise sets them to zero.
   */
  LeggedRobotInitializer(CentroidalModelInfo info, const SwitchedModelReferenceManager& referenceManager,
                         bool extendNormalizedMomentum = false);

  ~LeggedRobotInitializer() override = default;
  LeggedRobotInitializer* Clone() const override;

  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

 private:
  LeggedRobotInitializer(const LeggedRobotInitializer& other) = default;

  const CentroidalModelInfo info_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
  const bool extendNormalizedMomentum_;
};

}  // namespace legged_robot
}  // namespace bolza
