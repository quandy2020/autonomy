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
 * @file modified_relaxed_barrier_penalty.hpp
 * @brief Implements the augmented Lagrangian for a single inequality constraint \f$ h \geq 0 \f$ through the modified-log-barrier method.
 */


#include "bolza/core/penalties/augmented/augmented_penalty_base.hpp"

namespace bolza {
namespace augmented {

/**
 *  Implements the augmented Lagrangian for a single inequality constraint \f$ h \geq 0 \f$ through the modified-log-barrier method.
 *
 *  This leads to the following augmented-Lagrangian penalty function (referred to as the smooth-PHR penalty in the corresponding paper):
 *  \f[
 *      p(h, \lambda) = \frac{\lambda^2}{\rho} \psi\left(\frac{\rho h}{\lambda}\right).
 *  \f]
 *
 *  where \f$ \pho \f$ is the scale. \f$ \psi(.) \f$ is defined as a shifted quadratically-relaxed log barrier function.
 *  Unlike the relaxed log barrier penalty, this function is defined over the domain \f$ x > -1 \f$. Therefore, the value of the
 *  relaxation parameter has to also belong to this domain.
 *
 *  This is then minimized by the solver, while the maximization of the approximate dual function is done by updating the Lagrange
 *  multipliers with the following update rule:
 *  \f[
 *      \lambda^*_{k+1} = -\alpha \lambda^*_k \psi'\left(\frac{\pho h^*_{k+1}}{\lambda^*_k}\right).
 *  \f]
 *
 * where \f$ \psi'(.) \f$ is the total derivative of \f$ \psi(.) \f$.
 */
class ModifiedRelaxedBarrierPenalty final : public AugmentedPenaltyBase {
 public:
  /**
   * Configuration object for the modified relaxed barrier penalty.
   * scale: scaling factor, see class description
   * relaxation: relaxation parameter, see class description
   * stepLenght: step-length parameter, see class description
   */
  struct Config {
    /**
     * @brief Constructs a Config.
     */
    Config() : Config(10.0, 0.0, 1.0) {}
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

    /**
   * @brief Constructor 
   * @param [in] config_: Config.
   */
  ModifiedRelaxedBarrierPenalty(Config config) : config_(std::move(config)), quadCoeff_(config_) {}

    /**
   * @brief Factory function 
   * @param [in] config: Config.
   * @return Owning pointer.
   */
  static std::unique_ptr<ModifiedRelaxedBarrierPenalty> Create(Config config) {
    /**
     * @brief Move.
     * @return Result value.
     */
    return std::make_unique<ModifiedRelaxedBarrierPenalty>(std::move(config));
  }

  /**
   * @brief Destructor.
   */
  ~ModifiedRelaxedBarrierPenalty() override = default;
  /**
   * @brief Clone.
   * @return Pointer; may be null.
   */
  ModifiedRelaxedBarrierPenalty* Clone() const override { /**
  * @brief Modified Relaxed Barrier Penalty.
  * @param [in,out] this: This.
  * @return Result value.
  */
 return new ModifiedRelaxedBarrierPenalty(*this); }
  /**
   * @brief Name.
   * @return Result value.
   */
  std::string Name() const override { return "ModifiedRelaxedBarrierPenalty"; }

  /**
   * @brief Get Value.
   * @param [in] t: T.
   * @param [in] l: L.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetValue(scalar_t t, scalar_t l, scalar_t h) const override {
    /**
     * @brief VFunc.
     * @param [in] l: L.
     * @param [in] h: H.
     * @return Result value.
     */
    const scalar_t v = VFunc(l, h);
    if (v > config_.relaxation) {
      /**
       * @brief WFunc.
       * @param [in,out] v: V.
       * @return Result value.
       */
      return -WFunc(l) * log(1.0 + v);
    } else {
      const scalar_t vDelta = v - config_.relaxation;
      /**
       * @brief WFunc.
       * @param [in,out] c0: C0.
       * @return Result value.
       */
      return WFunc(l) * (0.5 * quadCoeff_.c2 * vDelta * vDelta + quadCoeff_.c1 * vDelta + quadCoeff_.c0);
    }
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
     * @brief VFunc.
     * @param [in] l: L.
     * @param [in] h: H.
     * @return Result value.
     */
    const scalar_t v = VFunc(l, h);
    if (v > config_.relaxation) {
      /**
       * @brief WFunc.
       * @param [in,out] l: L.
       * @return Result value.
       */
      return -WFunc(l) / (1.0 + v) * DvdhFunc(l);
    } else {
      /**
       * @brief WFunc.
       * @param [in,out] l: L.
       * @return Result value.
       */
      return WFunc(l) * (quadCoeff_.c2 * (v - config_.relaxation) + quadCoeff_.c1) * DvdhFunc(l);
    }
  }

  /**
   * @brief Get Second Derivative.
   * @param [in] t: T.
   * @param [in] l: L.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t GetSecondDerivative(scalar_t t, scalar_t l, scalar_t h) const override {
    /**
     * @brief VFunc.
     * @param [in] l: L.
     * @param [in] h: H.
     * @return Result value.
     */
    const scalar_t v = VFunc(l, h);
    /**
     * @brief Dvdh Func.
     * @param [in] l: L.
     * @return Result value.
     */
    const scalar_t dvdh = DvdhFunc(l);
    if (v > config_.relaxation) {
      return WFunc(l) / ((1.0 + v) * (1.0 + v)) * dvdh * dvdh;
    } else {
      return WFunc(l) * quadCoeff_.c2 * dvdh * dvdh;
    }
  }

  /**
   * @brief Update Multiplier.
   * @param [in] t: T.
   * @param [in] l: L.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t UpdateMultiplier(scalar_t t, scalar_t l, scalar_t h) const override {
    /**
     * @brief VFunc.
     * @param [in] l: L.
     * @param [in] h: H.
     * @return Result value.
     */
    const scalar_t v = VFunc(l, h);
    constexpr scalar_t lambdaMin = 1e-4;
    if (v > config_.relaxation) {
      /**
       * @brief Max.
       * @param [in] lambdaMin: Lambda Min.
       * @return Result value.
       */
      return std::max(lambdaMin, WFunc(l) * DvdhFunc(l) / (1 + v));
    } else {
      /**
       * @brief Max.
       * @param [in] lambdaMin: Lambda Min.
       * @return Result value.
       */
      return std::max(lambdaMin, config_.stepSize * WFunc(l) * (-quadCoeff_.c2 * (v - config_.relaxation) - quadCoeff_.c1) * DvdhFunc(l));
    }
  }

  /**
   * @brief Initialize Multiplier.
   * @return Result value.
   */
  scalar_t InitializeMultiplier() const override { return 1.0; }

 private:
  /**
   * @brief Constructs a ModifiedRelaxedBarrierPenalty.
   * @param [in] other: Other.
   */
  ModifiedRelaxedBarrierPenalty(const ModifiedRelaxedBarrierPenalty& other) = default;

  /**
   * @brief WFunc.
   * @param [in] l: L.
   * @return Result value.
   */
  scalar_t WFunc(scalar_t l) const { return l * l / config_.scale; }
  /**
   * @brief Dvdh Func.
   * @param [in] l: L.
   * @return Result value.
   */
  scalar_t DvdhFunc(scalar_t l) const { return config_.scale / l; }
  /**
   * @brief VFunc.
   * @param [in] l: L.
   * @param [in] h: H.
   * @return Result value.
   */
  scalar_t VFunc(scalar_t l, scalar_t h) const { return config_.scale * h / l; }

  struct QuadCoeff {
    /**
     * @brief Constructs a QuadCoeff.
     * @param [in] config: Config.
     */
    QuadCoeff(const Config& config) {
      /**
       * @brief Pow.
       * @param [in] relaxation: Relaxation.
       * @return Result value.
       */
      c2 = 1.0 / std::pow(1.0 + config.relaxation, 2);
      c1 = -1.0 / (1.0 + config.relaxation);
      /**
       * @brief Log.
       * @param [in] relaxation: Relaxation.
       * @return Result value.
       */
      c0 = -log(1.0 + config.relaxation);
    }

    scalar_t c2 = 0.0, c1 = 0.0, c0 = 0.0;
  };

  const Config config_;
  const QuadCoeff quadCoeff_;
};

}  // namespace augmented
}  // namespace bolza
