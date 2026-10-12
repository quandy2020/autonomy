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
 * @file state_cost_collection.hpp
 * @brief Cost function combining a collection of cost terms.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/misc/collection.hpp>
#include <bolza/core/reference/target_trajectories.hpp>

#include "state_cost.hpp"

namespace bolza {

/**
 * Cost function combining a collection of cost terms.
 *
 * This class collects a variable number of cost terms and provides methods to get the
 * summed cost values and quadratic approximations. Each cost term can be accessed through its
 * string name and can be activated or deactivated.
 */
class StateCostCollection : public Collection<StateCost> {
 public:
  /**
   * @brief Constructs a StateCostCollection.
   */
  StateCostCollection() = default;
  /**
   * @brief Destructor.
   */
  virtual ~StateCostCollection() = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  virtual StateCostCollection* Clone() const;

    /**
   * @brief /** Get state-only cost value */.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  virtual scalar_t GetValue(scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories,
                            const PreComputation& preComp) const;

    /**
   * @brief /** Get state-only cost quadratic approximation */.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] targetTrajectories: Target Trajectories.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  virtual ScalarFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                         const TargetTrajectories& targetTrajectories,
                                                                         const PreComputation& preComp) const;

 protected:
    /**
   * @brief /** Copy constructor */.
   * @param [in] other: Other.
   */
  StateCostCollection(const StateCostCollection& other);
};

}  // namespace bolza
