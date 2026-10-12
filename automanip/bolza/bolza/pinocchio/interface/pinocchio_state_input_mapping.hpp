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
 * @file pinocchio_state_input_mapping.hpp
 * @brief Mapping between OCS2 and pinocchio state and input.
 */


#include <bolza/core/types.hpp>
#include "bolza/pinocchio/interface/pinocchio_interface.hpp"

namespace bolza {
/**
 * Mapping between OCS2 and pinocchio state and input
 */
template <typename SCALAR>
class PinocchioStateInputMapping {
 public:
  using vector_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
  using matrix_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>;

  /**
   * @brief Constructs a PinocchioStateInputMapping.
   */
  PinocchioStateInputMapping() = default;
  /**
   * @brief Destructor.
   */
  virtual ~PinocchioStateInputMapping() = default;
  PinocchioStateInputMapping<SCALAR>& operator=(const PinocchioStateInputMapping<SCALAR>& rhs) = delete;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  virtual PinocchioStateInputMapping<SCALAR>* Clone() const = 0;

  /**
   * @brief Get the pinocchio joint configuration from OCS2 state and input vectors.
   * @param [in] state: State.
   * @return Result value.
   */
  virtual vector_t GetPinocchioJointPosition(const vector_t& state) const = 0;

  /**
   * @brief Get the pinocchio joint velocity from OCS2 state and input vectors.
   * @param [in] state: State.
   * @param [in] input: Input.
   * @return Result value.
   */
  virtual vector_t GetPinocchioJointVelocity(const vector_t& state, const vector_t& input) const = 0;

  /**
   * @brief Mapps pinocchio jacobians dfdq, dfdv to OCS2 jacobians dfdx, dfdu.
   * @param [in] state: State.
   * @param [in] Jq: Jq.
   * @param [in] Jv: Jv.
   * @return Result value.
   */
  virtual std::pair<matrix_t, matrix_t> GetOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const = 0;

  /**
   * @brief If the mapping requires PinocchioInterface, use this method and set an updated PinocchioInterface.
   * @param [in] pinocchioInterface: Pinocchio Interface.
   */
  virtual void SetPinocchioInterface(const PinocchioInterfaceTpl<SCALAR>& pinocchioInterface) {}

 protected:
  /**
   * @brief Constructs a PinocchioStateInputMapping.
   * @param [in] rhs: Rhs.
   */
  PinocchioStateInputMapping(const PinocchioStateInputMapping<SCALAR>& rhs) = default;
};

}  // namespace bolza
