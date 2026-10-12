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
 * @file smooth_absolute_penalty.hpp
 * @brief Implements the smooth-absolute function for a single equality constraint \f$ h = 0 \f$.
 */


#include "bolza/core/penalties/augmented/augmented_penalty_base.hpp"

namespace bolza {
namespace augmented {

/**
 * Implements the smooth-absolute function for a single equality constraint \f$ h = 0 \f$
 *
 * \f[
 *   p(h) = \mu sqrt(h^2 + \delta^2).
 * \f]
 *
 * where \f$ \mu > 0 \f$, and \f$ \delta > 0 \f$ are scale and relaxation parameters respectively. Note that
 * \f$ \delta \f$ defines the error bound between the absolute function and its approximation:
 *
 * \f[
 *   | x - sqrt(h^2 + \delta^2) | \leq \delta, \quad \forall x \in R
 * \f]
 *
 *  This is then minimized with the solver, while the Lagrange multipliers are updated as:
 * \f[
 *      \lambda^*_{k+1} = \lambda^*_k - \alpha h^*_{k+1}.
 * \f]
 *
 * with \f$ alpha \f$ is the step-length. In Augmented Lagrangian method this is often set to \f$ \mu \f$.
 */
class SmoothAbsolutePenalty final : public AugmentedPenaltyBase {
 public:
  /**
   * Configuration object for the smooth absolute penalty.
   * scale: scaling factor, see class description
   * relaxation: relaxation parameter, see class description
   * stepLenght: step-length parameter, see class description
   */
  struct Config {
    /**
     * @brief Constructs a Config.
     */
    Config() : Config(100.0, 1e-2, 0.0) {}
    /**
     * @brief Constructs a Config.
     * @param [in] scaleParam: Scale Param.
     * @param [in] relaxationParam: Relaxation Param.
     * @param [in] stepSizeParam: Step Size Param.
     */
    Config(scalar_t scaleParam, scalar_t relaxationParam, scalar_t stepSizeParam)
        /**
         * @brief Scale.
         * @param [in] stepSizeParam: Step Size Param.
         * @return Result value.
         */
        : scale(scaleParam), relaxation(relaxationParam), stepSize(stepSizeParam) {}
    scalar_t scale;
    scalar_t relaxation;
    scalar_t stepSize;
  };

  /** Constructor */
  explicit SmoothAbsolutePenalty(Config config) : config_(std::move(config)) {}

    /**
   * @brief Factory function 
   * @param [in] config: Config.
   * @return Owning pointer.
   */
  static std::unique_ptr<SmoothAbsolutePenalty> Create(Config config) { /**
  * @brief Move.
  * @return Result value.
  */
 return std::make_unique<SmoothAbsolutePenalty>(std::move(config)); }

  /**
   * @brief Destructor.
   */
  ~SmoothAbsolutePenalty() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  SmoothAbsolutePenalty* Clone() const override { /**
  * @brief Smooth Absolute Penalty.
  * @param [in,out] this: This.
  * @return Result value.
  */
 return new SmoothAbsolutePenalty(*this); }
  /**
   * @brief Name.
   * @return Result value.
   */
  std::string Name() const override { return "SmoothAbsolutePenalty"; }

  /**
   * @brief Get Value.
   * @param [in] t: T.
   * @param [in] l: L.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetValue(scalar_t t, scalar_t l, scalar_t h) const override {
    /**
     * @brief Sqrt.
     * @param [in,out] relaxation: Relaxation.
     * @return Pointer; may be null.
     */
    return -l * h + config_.scale * sqrt(h * h + config_.relaxation * config_.relaxation);
  }
  /**
   * @brief Get Derivative.
   * @param [in] t: T.
   * @param [in] l: L.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetDerivative(scalar_t t, scalar_t l, scalar_t h) const override {
    /**
     * @brief Sqrt.
     * @param [in,out] relaxation: Relaxation.
     * @return Result value.
     */
    return -l + config_.scale * h / sqrt(h * h + config_.relaxation * config_.relaxation);
  }
  /**
   * @brief Get Second Derivative.
   * @param [in] t: T.
   * @param [in] l: L.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetSecondDerivative(scalar_t t, scalar_t l, scalar_t h) const override {
    const scalar_t deltaSquare = config_.relaxation * config_.relaxation;
    /**
     * @brief Pow.
     * @param [in,out] deltaSquare: Delta Square.
     * @return Result value.
     */
    return config_.scale * deltaSquare / pow(h * h + deltaSquare, 1.5);
  }

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
   * @brief Constructs a SmoothAbsolutePenalty.
   * @param [in] other: Other.
   */
  SmoothAbsolutePenalty(const SmoothAbsolutePenalty& other) = default;

  const Config config_;
};

}  // namespace augmented
}  // namespace bolza
