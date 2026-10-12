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
 * @file state_constraint_collection.hpp
 * @brief Constraint collection class.
 */


#include <bolza/core/pre_computation.hpp>
#include <bolza/core/types.hpp>
#include <bolza/core/constraint/state_constraint.hpp>
#include <bolza/core/misc/collection.hpp>

namespace bolza {

/**
 * Constraint collection class
 *
 * This class collects a variable number of constraint functions and provides methods to get the
 * concatenated constraint vectors and approximations. Each constraint can be accessed through its
 * string name and can be activated or deactivated.
 */
class StateConstraintCollection : public Collection<StateConstraint> {
 public:
  /**
   * @brief Constructs a StateConstraintCollection.
   */
  StateConstraintCollection() = default;
  /**
   * @brief Destructor.
   */
  ~StateConstraintCollection() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  StateConstraintCollection* Clone() const override;

    /**
   * @brief Returns the number of active constraints at a given time. 
   * @param [in] time: Time.
   * @return Result value.
   */
  size_t GetNumConstraints(scalar_t time) const;

    /**
   * @brief Returns the number of active constraints at a given time for each term. If a term is inactive, its size is zero. 
   * @param [in] time: Time.
   * @return Result value.
   */
  size_array_t GetTermsSize(scalar_t time) const;

    /**
   * @brief Get an array of all constraints. If a term is inactive, the corresponding element is a vector of size zero. 
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  virtual vector_array_t GetValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const;

    /**
   * @brief Get the constraint linear approximation 
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  virtual VectorFunctionLinearApproximation GetLinearApproximation(scalar_t time, const vector_t& state,
                                                                   const PreComputation& preComp) const;

    /**
   * @brief Get the constraint quadratic approximation 
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  virtual VectorFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                         const PreComputation& preComp) const;

 protected:
    /**
   * @brief Copy constructor 
   * @param [in] other: Other.
   */
  StateConstraintCollection(const StateConstraintCollection& other);
};

}  // namespace bolza
