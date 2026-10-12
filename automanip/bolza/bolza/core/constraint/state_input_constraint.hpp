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
 * @file state_input_constraint.hpp
 * @brief State-input constraint function base class.
 */


#include <type_traits>

#include <bolza/core/pre_computation.hpp>
#include <bolza/core/types.hpp>
#include <bolza/core/constraint/constraint_order.hpp>

namespace bolza {

/** State-input constraint function base class */
class StateInputConstraint {
 public:
  /**
   * @brief Constructs a StateInputConstraint.
   * @param [in] order: Order.
   */
  explicit StateInputConstraint(ConstraintOrder order) : order_(order) {}
  /**
   * @brief Destructor.
   */
  virtual ~StateInputConstraint() = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  virtual StateInputConstraint* Clone() const = 0;

  /**
   * @brief Get the constraint order (Linear or Quadratic).
   * @return Result value.
   */
  constexpr ConstraintOrder GetOrder() const { return order_; };

    /**
   * @brief /** Check constraint activity */.
   * @param [in] time: Time.
   * @return True on success.
   */
  virtual bool IsActive(scalar_t time) const { return true; }

    /**
   * @brief /** Get the size of the constraint vector at given time */.
   * @param [in] time: Time.
   * @return Result value.
   */
  virtual size_t GetNumConstraints(scalar_t time) const = 0;

    /**
   * @brief /** Get the constraint vector value */.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  virtual vector_t GetValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const = 0;

    /**
   * @brief /** Get the constraint linear approximation */.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  virtual VectorFunctionLinearApproximation GetLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                   const PreComputation& preComp) const {
    if (order_ == ConstraintOrder::Linear) {
      /**
       * @brief Runtime error.
       * @return Result value.
       */
      throw std::runtime_error("[StateInputConstraint] Linear approximation not implemented!");
    } else {
      /**
       * @brief Runtime error.
       * @return Result value.
       */
      throw std::runtime_error("[StateInputConstraint] The class only provides Quadratic approximation! call GetQuadraticApproximation()");
    }
  }

    /**
   * @brief /** Get the constraint quadratic approximation */.
   * @param [in] time: Time.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in] preComp: Pre Comp.
   * @return Result value.
   */
  virtual VectorFunctionQuadraticApproximation GetQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                         const PreComputation& preComp) const {
    if (order_ == ConstraintOrder::Quadratic) {
      /**
       * @brief Runtime error.
       * @return Result value.
       */
      throw std::runtime_error("[StateConstraint] Quadratic approximation not implemented!");
    } else {
      /**
       * @brief Runtime error.
       * @return Result value.
       */
      throw std::runtime_error("[StateConstraint] The class only provides Linear approximation! call GetLinearApproximation()");
    }
  }

 protected:
  /**
   * @brief Constructs a StateInputConstraint.
   * @param [in] rhs: Rhs.
   */
  StateInputConstraint(const StateInputConstraint& rhs) = default;

 private:
  ConstraintOrder order_;
};

// Template for conditional compilation using SFINAE
template <typename T>
using EnableIfStateInputConstraint_t = typename std::enable_if<std::is_same<T, StateInputConstraint>::Value, bool>::type;

}  // namespace bolza
