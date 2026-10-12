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
 * @file relaxed_barrier_penalty.hpp
 * @brief Implements the relaxed barrier function for a single inequality constraint \f$ h \geq 0 \f$.
 */


#include <bolza/core/penalties/penalties/penalty_base.hpp>

namespace bolza {

/**
 * Implements the relaxed barrier function for a single inequality constraint \f$ h \geq 0 \f$
 *
 * \f[
 *   p(h)=\left\lbrace
 *               \begin{array}{ll}
 *                 -\mu \ln(h) & if \quad  h > \delta, \\
 *                 -\mu \ln(\delta) + \mu \frac{1}{2} \left( \left( \frac{h-2\delta}{\delta} \right)^2 - 1 \right) & otherwise,
 *               \end{array}
 *             \right.
 * \f]
 *
 * where \f$ \mu \geq 0 \f$, and \f$ \delta \geq 0 \f$ are user defined parameters.
 */
class RelaxedBarrierPenalty final : public PenaltyBase {
 public:
  /**
   * Configuration object for the relaxed barrier penalty.
   * mu : scaling factor
   * delta: relaxation parameter, see class description
   */
  struct Config {
    /**
     * @brief Constructs a Config.
     */
    Config() : Config(1.0, 1e-3) {}
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
  explicit RelaxedBarrierPenalty(Config config) : config_(std::move(config)) {}

  /**
   * @brief Destructor.
   */
  ~RelaxedBarrierPenalty() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  RelaxedBarrierPenalty* Clone() const override { /**
  * @brief Relaxed Barrier Penalty.
  * @param [in,out] this: This.
  * @return Result value.
  */
 return new RelaxedBarrierPenalty(*this); }
  /**
   * @brief Name.
   * @return Result value.
   */
  std::string Name() const override { return "RelaxedBarrierPenalty"; }

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
   * @brief Constructs a RelaxedBarrierPenalty.
   * @param [in] other: Other.
   */
  RelaxedBarrierPenalty(const RelaxedBarrierPenalty& other) = default;

  const Config config_;
};

}  // namespace bolza
