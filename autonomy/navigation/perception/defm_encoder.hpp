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
 * @file defm_encoder.hpp
 * @brief DepthFoundationModelEncoder: preprocess -> model -> pool.
 */

#pragma once

#include <memory>

#include "navigation/perception/attention_pooler.hpp"
#include "navigation/perception/defm_model.hpp"
#include "navigation/perception/depth_preprocessor.hpp"
#include "navigation/perception/observation_encoder.hpp"
#include "navigation/perception/perceptual_uncertainty_estimator.hpp"

namespace navigation
{
namespace perception
{

/**
 * @brief Encodes depth into DeFM tokens and a pooled embedding.
 *
 * @par Threading
 * Called from the navigation tick thread; not thread-safe.
 */
class DepthFoundationModelEncoder final : public ObservationEncoder
{
public:
    /** @brief Encoder policy knobs. */
    struct Options
    {
        /** @brief Minimum fraction of valid depth pixels; below it Encode fails. */
        double min_valid_ratio{0.15};
    };

    /**
     * @brief Construct from pipeline components.
     *
     * @param[in] preprocessor Depth decoder/letterbox/normalizer.
     * @param[in] model Frozen DeFM inference backend.
     * @param[in] pooler Token pooling implementation.
     * @param[in] uncertainty Uncertainty estimator.
     * @param[in] options Encoder thresholds.
     */
    DepthFoundationModelEncoder(std::unique_ptr<DepthPreprocessor> preprocessor,
                                std::unique_ptr<DefmModel> model,
                                std::unique_ptr<AttentionPooler> pooler,
                                std::unique_ptr<PerceptualUncertaintyEstimator> uncertainty,
                                Options options);

    /**
     * @brief Encode one observation into a DepthFeature.
     *
     * @param[in] observation Fused observation with depth.
     * @param[out] feature Tokens/pooled vector/uncertainty on success.
     * @return kOk, kInvalidArgument for malformed depth, kUnavailable for
     *         insufficient valid pixels, kInternal for model/pooling failures.
     */
    common::Status Encode(const observation::Observation& observation,
                          observation::DepthFeature* feature) override;

    /** @brief Drop cached state; currently a no-op. */
    void Reset() override;

private:
    /** @brief Depth decoding/normalization stage. */
    std::unique_ptr<DepthPreprocessor> preprocessor_;
    /** @brief Frozen DeFM backend. */
    std::unique_ptr<DefmModel> model_;
    /** @brief Token pooling stage. */
    std::unique_ptr<AttentionPooler> pooler_;
    /** @brief Uncertainty estimator. */
    std::unique_ptr<PerceptualUncertaintyEstimator> uncertainty_;
    /** @brief Encoder policy knobs. */
    Options options_;
};

}  // namespace perception
}  // namespace navigation
