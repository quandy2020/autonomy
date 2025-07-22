/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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

#ifndef AUTONOMY_COMMON_MOTION_MODEL_STATIONARY_MOTION_MODEL_HPP_
#define AUTONOMY_COMMON_MOTION_MODEL_STATIONARY_MOTION_MODEL_HPP_

#include "autonomy/common/motion_model/motion_model_interface.hpp"
#include "autonomy/common/state_vector/common_states.hpp"
#include "autonomy/common/state_vector/generic_state.hpp"

namespace autonomy {
namespace common { 
namespace motion_model {

///
/// @brief      This class describes a stationary motion model, i.e., the model that does not change
///             the state.
///
/// @tparam     StateT  State with which the motion model works.
///
template<typename StateT>
class StationaryMotionModel
  : public MotionModelInterface<StationaryMotionModel<StateT>>
{
public:
  using State = StateT;

protected:
  // Allow the CRTP interface to call private functions.
  friend MotionModelInterface<StationaryMotionModel<StateT>>;

  ///
  /// @brief      A crtp-called function that predicts the state forward.
  ///
  /// @param[in]  state  The current state vector
  ///
  /// @return     A const reference to the unchanged input state.
  ///
  inline const State & crtp_predict(
    const State & state,
    const std::chrono::nanoseconds &) const
  {
    return state;
  }

  ///
  /// @brief      A crtp-called function that computes a Jacobian for the stationary motion model.
  ///
  /// @return     An identity matrix.
  ///
  typename State::Matrix crtp_jacobian(const State &, const std::chrono::nanoseconds &) const
  {
    return Eigen::Matrix<typename State::Scalar, State::size(), State::size()>::Identity();
  }
};

}  // namespace motion_model
}  // namespace common 
}  // namespace autonomy

#endif  // AUTONOMY_COMMON_MOTION_MODEL_STATIONARY_MOTION_MODEL_HPP_
