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
 * @file quadratic_penalty.hpp
 * @brief Implements the augmented Lagrangian for a single equality constraint \f$ h = 0 \f$. This leads to the following.
 */


#include <memory>

#include "bolza/core/penalties/augmented/augmented_penalty_base.hpp"

namespace bolza {
namespace augmented {

/**
 *  Implements the augmented Lagrangian for a single equality constraint \f$ h = 0 \f$. This leads to the following
 *  augmented Lagrangian:
 *
 *  \f[
 *      L_{A} = L * \lambda h + \frac{\mu}{2} h^2.
 *  \f]
 *
 *  where \f$ L \f$ is the Lagrangian and \f$ \mu \f$ is the scale, while the remaining terms form the penalty
 *  function \f$ p(h, \lambda) \f$.
 *
 *  This is then minimized with the solver, while the Lagrange multipliers are updated as:
 * \f[
 *      \lambda^*_{k+1} = \lambda^*_k - \alpha h^*_{k+1}.
 * \f]
 *
 * with \f$ alpha \f$ is the step-length. In Augmented Lagrangian method this is often set to \f$ \mu \f$.
 */
class QuadraticPenalty final : public AugmentedPenaltyBase {
 public:
  /**
   * Configuration object for the quadratic penalty.
   * scale: scaling factor, see class description
   * stepSize: step-length parameter, see class description
   */
  struct Config {
    /**
     * @brief Constructs a Config.
     */
    Config() : Config(100.0, 0.0) {}
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
  explicit QuadraticPenalty(Config config) : config_(std::move(config)) {}

    /**
   * @brief Factory function 
   * @param [in] config: Config.
   * @return Owning pointer.
   */
  static std::unique_ptr<QuadraticPenalty> Create(Config config) { /**
  * @brief Move.
  * @return Result value.
  */
 return std::make_unique<QuadraticPenalty>(std::move(config)); }

  /**
   * @brief Destructor.
   */
  ~QuadraticPenalty() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  QuadraticPenalty* Clone() const override { /**
  * @brief Quadratic Penalty.
  * @param [in,out] this: This.
  * @return Result value.
  */
 return new QuadraticPenalty(*this); }
  /**
   * @brief Name.
   * @return Result value.
   */
  std::string Name() const override { return "QuadraticPenalty"; }

  /**
   * @brief Get Value.
   * @param [in] t: T.
   * @param [in] l: L.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetValue(scalar_t t, scalar_t l, scalar_t h) const override { return -l * h + 0.5 * config_.scale * h * h; }
  /**
   * @brief Get Derivative.
   * @param [in] t: T.
   * @param [in] l: L.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetDerivative(scalar_t t, scalar_t l, scalar_t h) const override { return -l + config_.scale * h; }
  /**
   * @brief Get Second Derivative.
   * @param [in] t: T.
   * @param [in] l: L.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetSecondDerivative(scalar_t t, scalar_t l, scalar_t h) const override { return config_.scale; }

  /**
   * @brief Update Multiplier.
   * @param [in] t: T.
   * @param [in] l: L.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t UpdateMultiplier(scalar_t t, scalar_t l, scalar_t h) const override { return l - config_.stepSize * config_.scale * h; }
  /**
   * @brief Initialize Multiplier.
   * @return Result value.
   */
  scalar_t InitializeMultiplier() const override { return 0.0; }

 private:
  /**
   * @brief Constructs a QuadraticPenalty.
   * @param [in] other: Other.
   */
  QuadraticPenalty(const QuadraticPenalty& other) = default;

  const Config config_;
};

}  // namespace augmented
}  // namespace bolza
