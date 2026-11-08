/*
 * Copyright 2026 The Navigation Authors (navigation)
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

/**
 * @file perceptual_uncertainty_estimator.hpp
 * @brief Combines depth validity, feature norm and temporal jitter.
 */

#pragma once

namespace navigation
{
namespace perception
{

/** @brief Weights for the three uncertainty terms. */
struct UncertaintyOptions
{
    /** @brief Weight of the invalid-depth ratio. */
    double invalid_weight{0.5};
    /** @brief Weight of the feature-norm deficit. */
    double norm_weight{0.3};
    /** @brief Weight of temporal jitter. */
    double jitter_weight{0.2};
    /** @brief Token norm considered fully confident. */
    double token_norm_reference{1.0};
};

class PerceptualUncertaintyEstimator
{
public:
    /** @brief Construct with uncertainty weights. */
    explicit PerceptualUncertaintyEstimator(UncertaintyOptions options = {});

    /**
     * @brief Estimate uncertainty in [0,1].
     *
     * @param[in] invalid_ratio Fraction of invalid depth pixels.
     * @param[in] token_norm L2 norm of the pooled feature.
     * @param[in] temporal_jitter Normalized frame-to-frame jitter.
     * @return Weighted, clamped uncertainty.
     */
    [[nodiscard]] float Estimate(double invalid_ratio, double token_norm,
                                 double temporal_jitter) const;

private:
    /** @brief Uncertainty weights. */
    UncertaintyOptions options_;
};

}  // namespace perception
}  // namespace navigation
