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
 * @file mpcnet_behavioral_controller.hpp
 * @brief A behavioral controller that computes the input based on a mixture of an optimal policy (e.g. implicitly found via MPC).
 */


#include <memory>

#include <bolza/core/control/controller_base.hpp>

#include "bolza/mpcnet/control/mpcnet_controller_base.hpp"

namespace bolza {
namespace mpcnet {

/**
 * A behavioral controller that computes the input based on a mixture of an optimal policy (e.g. implicitly found via MPC)
 * and a learned policy (e.g. explicitly represented by a neural network).
 * The behavioral policy is pi_behavioral = alpha * pi_optimal + (1 - alpha) * pi_learned with alpha in [0, 1].
 */
class MPCNetBehavioralController final : public ControllerBase {
 public:
  /**
   * Constructor.
   * @param [in] alpha: The mixture parameter.
   * @param [in] optimalController: The optimal Controller (this class takes ownership of a clone).
   * @param [in] learnedController: The learned Controller (this class takes ownership of a clone).
   */
  MPCNetBehavioralController(scalar_t alpha, const ControllerBase& optimalController, const MPCNetControllerBase& learnedController)
      /**
       * @brief Alpha.
       * @return Result value.
       */
      : alpha_(alpha), optimalControllerPtr_(optimalController.Clone()), learnedControllerPtr_(learnedController.Clone()) {}

  /**
   * @brief Constructs a MPCNetBehavioralController.
   */
  MPCNetBehavioralController() = default;
  /**
   * @brief Destructor.
   */
  ~MPCNetBehavioralController() override = default;
  /**
   * @brief Clone.
   * @param [in] this: This.
   * @return Pointer; may be null.
   */
  MPCNetBehavioralController* Clone() const override { return new MPCNetBehavioralController(*this); }

  /**
   * Set the mixture parameter.
   * @param [in] alpha: The mixture parameter.
   */
  void SetAlpha(scalar_t alpha) { alpha_ = alpha; }

  /**
   * Set the optimal controller.
   * @param [in] optimalController: The optimal Controller (this class takes ownership of a clone).
   */
  void SetOptimalController(const ControllerBase& optimalController) { optimalControllerPtr_.reset(optimalController.Clone()); }

  /**
   * Set the learned controller.
   * @param [in] learnedController: The learned Controller (this class takes ownership of a clone).
   */
  void SetLearnedController(const MPCNetControllerBase& learnedController) { learnedControllerPtr_.reset(learnedController.Clone()); }

  /**
   * @brief Compute Input.
   * @param [in] t: T.
   * @param [in] x: X.
   * @return Result value.
   */
  vector_t ComputeInput(scalar_t t, const vector_t& x) override;
  /**
   * @brief Get Type.
   * @return Result value.
   */
  ControllerType GetType() const override { return ControllerType::BEHAVIORAL; }

  /**
   * @brief Size.
   * @return Result value.
   */
  int size() const override;
  /**
   * @brief Clear.
   */
  void clear() override;
  /**
   * @brief Empty.
   * @return True on success.
   */
  bool empty() const override;
  /**
   * @brief Concatenate.
   * @param [in] otherController: Other Controller.
   * @param [in] index: Index.
   * @param [in] length: Length.
   */
  void Concatenate(const ControllerBase* otherController, int index, int length) override;

 private:
  MPCNetBehavioralController(const MPCNetBehavioralController& other)
      /**
       * @brief MPCNet Behavioral Controller.
       * @param [in] alpha_: Alpha.
       * @param [in,out] optimalControllerPtr_: Optimal Controller Ptr.
       * @param [in,out] learnedControllerPtr_: Learned Controller Ptr.
       * @return Result value.
       */
      : MPCNetBehavioralController(other.alpha_, *other.optimalControllerPtr_, *other.learnedControllerPtr_) {}

  scalar_t alpha_;
  std::unique_ptr<ControllerBase> optimalControllerPtr_;
  std::unique_ptr<MPCNetControllerBase> learnedControllerPtr_;
};

}  // namespace mpcnet
}  // namespace bolza
