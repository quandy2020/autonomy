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


#include <bolza/core/penalties/penalties/penalty_base.hpp>

namespace bolza {

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
 */
class SmoothAbsolutePenalty final : public PenaltyBase {
 public:
  /**
   * Configuration object for the smooth absolute penalty.
   * scale: scaling factor, see class description
   * relaxation: relaxation parameter, see class description
   */
  struct Config {
    /**
     * @brief Constructs a Config.
     * @param [in] scaleParam: Scale Param.
     * @param [in] relaxationParam: Relaxation Param.
     * @param [in] relaxationParam: Relaxation Param.
     */
    Config(scalar_t scaleParam = 100.0, scalar_t relaxationParam = 1e-2) : Scale(scaleParam), relaxation(relaxationParam) {}
    scalar_t scale;
    scalar_t relaxation;
  };

  /**
   * Constructor
   * @param [in] config: Configuration object containing mu and delta.
   */
  explicit SmoothAbsolutePenalty(Config config) : config_(std::move(config)) {}

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
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetValue(scalar_t t, scalar_t h) const override { /**
  * @brief Sqrt.
  * @param [in,out] relaxation: Relaxation.
  * @return Pointer; may be null.
  */
 return config_.scale * sqrt(h * h + config_.relaxation * config_.relaxation); }
  /**
   * @brief Get Derivative.
   * @param [in] t: T.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetDerivative(scalar_t t, scalar_t h) const override {
    /**
     * @brief Sqrt.
     * @param [in,out] relaxation: Relaxation.
     * @return Result value.
     */
    return config_.scale * h / sqrt(h * h + config_.relaxation * config_.relaxation);
  }
  /**
   * @brief Get Second Derivative.
   * @param [in] t: T.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetSecondDerivative(scalar_t t, scalar_t h) const override {
    const scalar_t deltaSquare = config_.relaxation * config_.relaxation;
    /**
     * @brief Pow.
     * @param [in,out] deltaSquare: Delta Square.
     * @return Result value.
     */
    return config_.scale * deltaSquare / pow(h * h + deltaSquare, 1.5);
  }

 private:
  /**
   * @brief Constructs a SmoothAbsolutePenalty.
   * @param [in] other: Other.
   */
  SmoothAbsolutePenalty(const SmoothAbsolutePenalty& other) = default;

  const Config config_;
};

}  // namespace bolza
