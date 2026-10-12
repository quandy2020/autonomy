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
 * @file slackness_squared_hinge_penalty.hpp
 * @brief Implements the augmented Lagrangian for a single inequality constraint \f$ h \geq 0 \f$ by transforming it to the following form:.
 */


#include "bolza/core/penalties/augmented/augmented_penalty_base.hpp"

namespace bolza {
namespace augmented {

/**
 *  Implements the augmented Lagrangian for a single inequality constraint \f$ h \geq 0 \f$ by transforming it to the following form:
 *
 *  \f[ \left\lbrace
 *              \begin{array}{ll}
 *                  h - s = 0, \\
 *                  s \geq 0.
 *              \end{array}
 *            \right.
 *  \f]
 *
 *  This leads to the following augmented-Lagrangian penalty function (referred to as PHR penalty in the corresponding paper):
 *  \f[
 *      p(h, \lambda) = \frac{1}{2 \rho} (\max\{ 0, \lambda - \rho h \}^2 - \lambda^2).
 *  \f]
 *
 *  where \rho is the scale factor of the penalty. This is then minimized with the DDP algorithm, while the maximization of the
 *  approximate dual function is done by updating the Lagrange multipliers with a gradient ascent step as such:
 * \f[
 *      \lambda^*_{k+1} = \max\{ \lambda^*_k - \alpha h^*_{k+1}, (1 - \frac{\alpha}{\rho}) \lambda^*_k \}.
 * \f]
 */
class SlacknessSquaredHingePenalty final : public AugmentedPenaltyBase {
 public:
  /**
   * Configuration object for the squared hinge penalty.
   * @param [in] scale: scaling factor. In the class description, it is referred to as \pho.
   * @param [in] stepSize: step-size for updating Lagrange multiplier. In the class description, it is referred to as \alpha.
   */
  struct Config {
    /**
     * @brief Constructs a Config.
     */
    Config() : Config(10.0, 1.0) {}
    /**
     * @brief Constructs a Config.
     * @param [in] scaleParam: Scale Param.
     * @param [in] stepSizeParam: Step Size Param.
     */
    Config(scalar_t scaleParam, scalar_t stepSizeParam) : scale(scaleParam), stepSize(stepSizeParam) {}
    scalar_t scale;
    scalar_t stepSize;
  };

  /** Constructor */
  SlacknessSquaredHingePenalty(Config config) : config_(std::move(config)) {}

    /**
   * @brief Factory function 
   * @param [in] config: Config.
   * @return Owning pointer.
   */
  static std::unique_ptr<SlacknessSquaredHingePenalty> Create(Config config) {
    /**
     * @brief Move.
     * @return Result value.
     */
    return std::make_unique<SlacknessSquaredHingePenalty>(std::move(config));
  }

  /**
   * @brief Destructor.
   */
  ~SlacknessSquaredHingePenalty() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  SlacknessSquaredHingePenalty* Clone() const override { /**
  * @brief Slackness Squared Hinge Penalty.
  * @param [in,out] this: This.
  * @return Result value.
  */
 return new SlacknessSquaredHingePenalty(*this); }
  /**
   * @brief Name.
   * @return Result value.
   */
  std::string Name() const override { return "SlacknessSquaredHingePenalty"; }

  /**
   * @brief Get Value.
   * @param [in] t: T.
   * @param [in] l: L.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetValue(scalar_t t, scalar_t l, scalar_t h) const override {
    return (h < l / config_.scale) ? (-l * h + 0.5 * config_.scale * h * h) : (-0.5 * l * l / config_.scale);
  }
  /**
   * @brief Get Derivative.
   * @param [in] t: T.
   * @param [in] l: L.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetDerivative(scalar_t t, scalar_t l, scalar_t h) const override {
    return (h < l / config_.scale) ? (-l + config_.scale * h) : 0.0;
  }
  /**
   * @brief Get Second Derivative.
   * @param [in] t: T.
   * @param [in] l: L.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetSecondDerivative(scalar_t t, scalar_t l, scalar_t h) const override { return (h < l / config_.scale) ? config_.scale : 0.0; }

  /**
   * @brief Update Multiplier.
   * @param [in] t: T.
   * @param [in] l: L.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t UpdateMultiplier(scalar_t t, scalar_t l, scalar_t h) const override {
    /**
     * @brief Max.
     * @return Result value.
     */
    return std::max(0.0, std::max(l - config_.stepSize * config_.scale * h, (1.0 - config_.stepSize) * l));
  }
  /**
   * @brief Initialize Multiplier.
   * @return Result value.
   */
  scalar_t InitializeMultiplier() const override { return 0.0; }

 private:
  /**
   * @brief Constructs a SlacknessSquaredHingePenalty.
   * @param [in] other: Other.
   */
  SlacknessSquaredHingePenalty(const SlacknessSquaredHingePenalty& other) = default;

  const Config config_;
};

}  // namespace augmented
}  // namespace bolza
