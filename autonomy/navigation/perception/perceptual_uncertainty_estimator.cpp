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

#include "navigation/perception/perceptual_uncertainty_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace navigation
{
namespace perception
{

PerceptualUncertaintyEstimator::PerceptualUncertaintyEstimator(UncertaintyOptions options)
    : options_(options)
{
}

float PerceptualUncertaintyEstimator::Estimate(double invalid_ratio, double token_norm,
                                               double temporal_jitter) const
{
    const double safe_invalid = std::clamp(invalid_ratio, 0.0, 1.0);
    const double reference = std::max(1e-6, options_.token_norm_reference);
    const double norm_term = std::clamp(1.0 - token_norm / reference, 0.0, 1.0);
    const double jitter = std::clamp(temporal_jitter, 0.0, 1.0);
    const double total = options_.invalid_weight * safe_invalid +
                         options_.norm_weight * norm_term +
                         options_.jitter_weight * jitter;
    return static_cast<float>(std::clamp(total, 0.0, 1.0));
}

}  // namespace perception
}  // namespace navigation
