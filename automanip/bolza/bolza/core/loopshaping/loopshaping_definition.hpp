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
 * @file loopshaping_definition.hpp
 * @brief Loopshaping types:.
 */


#include <bolza/core/types.hpp>
#include <bolza/core/loopshaping/loopshaping_filter.hpp>

namespace bolza {

/**
 * Loopshaping types:
 *      outputpattern : The system inputs remain the inputs of the augmented system.
 *                      Loopshaping inputs are a linear combination of state and system inputs
 *      eliminatepattern : The loopshaping inputs become the inputs of the augmented system.
 *                         The system inputs are retreived through a linear combination of state and system inputs.
 */
enum class LoopshapingType { outputpattern, eliminatepattern };

/**
 *  Class to store and access the loopshaping definition
 */
class LoopshapingDefinition {
 public:
  /**
   * Constructor
   * @param [in] loopshapingType: Type of loopshaping system augmentation to apply
   * @param [in] filter: Filter encoding the dynamics between system and loopshaping (=filtered) inputs
   * @param [in] costMatrix: Quadratic cost term to apply to the filtered inputs. Defaults to identity if not provided.
   */
  LoopshapingDefinition(LoopshapingType loopshapingType, Filter filter, matrix_t costMatrix = matrix_t());

  /**
   * @brief Get the loopshaping type.
   * @return Result value.
   */
  LoopshapingType GetType() const { return loopshapingType_; }

  /**
   * @brief True if all matrices of the loopshaping filter are diagonal.
   * @return True on success.
   */
  bool isDiagonal() const { return diagonal_; }

  /**
   * @brief Get access to the filter specification.
   * @return Result value.
   */
  const Filter& GetInputFilter() const { return filter_; }

    /**
   * @brief /** Compute the cost on the filtered inputs */.
   * @param [in] filteredInput: Filtered Input.
   * @return Result value.
   */
  scalar_t LoopshapingCost(const vector_t& filteredInput) const { /**
  * @brief Dot.
  * @param [in,out] filteredInput: Filtered Input.
  * @return Result value.
  */
 return 0.5 * filteredInput.dot(R_ * filteredInput); }

  /**
   * @brief Get the quadratic cost matrix for the filtered inputs.
   * @return Result value.
   */
  matrix_t& CostMatrix() { return R_; }

  /** Display details of the LoopshapingDefinition  */
  void Print() const;

  /**
   * @brief Get System State.
   * @param [in] state: state of the augmented system
   * @return state of the original system
   */
  vector_t GetSystemState(const vector_t& state) const { /**
  * @brief Head.
  * @return Result value.
  */
 return state.head(state.rows() - filter_.getNumStates()); }
  /**
   * @brief Get System State.
   * @param [in] state: State.
   * @param [in,out] systemState: System State.
   */
  void GetSystemState(const vector_t& state, vector_t& systemState) const {
    /**
     * @brief Head.
     * @return Result value.
     */
    systemState = state.head(state.rows() - filter_.getNumStates());
  }

  /**
   * @brief Get System Input.
   * @param [in] state: state of the augmented system
   * @param [in] input: input of the augmented system
   * @return input of the original system
   */
  vector_t GetSystemInput(const vector_t& state, const vector_t& input) const;
  /**
   * @brief Get System Input.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in,out] systemInput: System Input.
   */
  void GetSystemInput(const vector_t& state, const vector_t& input, vector_t& systemInput) const;

  /**
   * @brief Get Filter State.
   * @param [in] state: state of the augmented system
   * @return state of the loopshaping filter
   */
  vector_t GetFilterState(const vector_t& state) const { /**
  * @brief Tail.
  * @return Result value.
  */
 return state.tail(filter_.getNumStates()); }
  /**
   * @brief Get Filter State.
   * @param [in] state: State.
   * @param [in,out] filterState: Filter State.
   */
  void GetFilterState(const vector_t& state, vector_t& filterState) const { /**
  * @brief Tail.
  * @return Result value.
  */
 filterState = state.tail(filter_.getNumStates()); }

  /**
   * @brief Get Filtered Input.
   * @param [in] state: state of the augmented system
   * @param [in] input: input of the augmented system
   * @return the filtered input
   */
  vector_t GetFilteredInput(const vector_t& state, const vector_t& input) const;
  /**
   * @brief Get Filtered Input.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @param [in,out] filteredInput: Filtered Input.
   */
  void GetFilteredInput(const vector_t& state, const vector_t& input, vector_t& filteredInput) const;

  /**
   * @brief Filter Flow Map.
   * @param [in] filterState: state of the loopshaping filter
   * @param [in] input: input of the augmented system
   * @return time derivative of the filter state
   */
  vector_t FilterFlowMap(const vector_t& filterState, const vector_t& input) const;

  /**
   * @brief Concatenate System And Filter State.
   * @param [in] systemState: state of the original system
   * @param [in] filterState: state of the loopshaping filter
   * @return state of the augmented system
   */
  vector_t ConcatenateSystemAndFilterState(const vector_t& systemState, const vector_t& filterState) const;

  /**
   * @brief Augmented System Input.
   * @param [in] systemInput: input of the original system
   * @param [in] filterInput: the filtered input
   * @return input of the augmented system
   */
  vector_t AugmentedSystemInput(const vector_t& systemInput, const vector_t& filterInput) const;

  /**
   * Finds a loopshaping state and input such that they are in equilibrium with a given system input
   * @param [in] systemInput: input of the original system
   * @param [in] filterState: (return) : state of the loopshaping filter
   * @param [in] filterInput: (return) : the filtered input
   */
  void GetFilterEquilibrium(const vector_t& systemInput, vector_t& filterState, vector_t& filterInput) const;

  /**
   * Finds a loopshaping input such that they are in equilibrium with a given system input
   * @param [in] systemInput: input of the original system
   * @param [in] filterState: state of the loopshaping filter
   * @param [in] filterInput: (return) : the filtered input
   */
  void GetFilterEquilibriumGivenState(const vector_t& systemInput, const vector_t& filterState, vector_t& filterInput) const;

 private:
  Filter filter_;
  LoopshapingType loopshapingType_;
  bool diagonal_;
  matrix_t R_;
};

}  // namespace bolza
