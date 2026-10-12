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
 * @file squared_hinge_penalty.hpp
 * @brief Implements the squared-hinge function for a single inequality constraint \f$ h \geq 0 \f$.
 */


#include <bolza/core/penalties/penalties/penalty_base.hpp>

namespace bolza {

/**
 * Implements the squared-hinge function for a single inequality constraint \f$ h \geq 0 \f$
 *
 * \f[
 *   p(h)=\left\lbrace
 *               \begin{array}{ll}
 *                 \frac{\mu}{2} (h - \delta)^2 & if \quad  h < \delta, \\
 *                 0 & otherwise,
 *               \end{array}
 *             \right.
 * \f]
 *
 * where \f$ \mu > 0 \f$, and \f$ \delta \in R \f$ are user defined parameters.
 */
class SquaredHingePenalty final : public PenaltyBase {
 public:
  /**
   * Configuration object for the squared hinge penalty.
   * mu : scaling factor
   * delta: relaxation parameter, see class description
   */
  struct Config {
    /**
     * @brief Constructs a Config.
     */
    Config() : Config(100.0, 1e-1) {}
    /**
     * @brief Constructs a Config.
     * @param [in] muParam: Mu Param.
     * @param [in] deltaParam: Delta Param.
     */
    Config(scalar_t muParam, scalar_t deltaParam) : mu(muParam), delta(deltaParam) {}
    scalar_t mu;
    scalar_t delta;
  };

  /**
   * Constructor
   * @param [in] config: Configuration object containing mu and delta.
   */
  explicit SquaredHingePenalty(Config config) : config_(std::move(config)) {}

  /**
   * @brief Destructor.
   */
  ~SquaredHingePenalty() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  SquaredHingePenalty* Clone() const override { /**
  * @brief Squared Hinge Penalty.
  * @param [in,out] this: This.
  * @return Result value.
  */
 return new SquaredHingePenalty(*this); }
  /**
   * @brief Name.
   * @return Result value.
   */
  std::string Name() const override { return "SquaredHingePenalty"; }

  /**
   * @brief Get Value.
   * @param [in] t: T.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetValue(scalar_t t, scalar_t h) const override;
  /**
   * @brief Get Derivative.
   * @param [in] t: T.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetDerivative(scalar_t t, scalar_t h) const override;
  /**
   * @brief Get Second Derivative.
   * @param [in] t: T.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetSecondDerivative(scalar_t t, scalar_t h) const override;

 private:
  /**
   * @brief Constructs a SquaredHingePenalty.
   * @param [in] other: Other.
   */
  SquaredHingePenalty(const SquaredHingePenalty& other) = default;

  Config config_;
};

}  // namespace bolza
