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

#include "navigation/perception/defm_encoder.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "automsgs/msgs/time_utils.hpp"

namespace navigation
{
namespace perception
{

DepthFoundationModelEncoder::DepthFoundationModelEncoder(
    std::unique_ptr<DepthPreprocessor> preprocessor, std::unique_ptr<DefmModel> model,
    std::unique_ptr<AttentionPooler> pooler,
    std::unique_ptr<PerceptualUncertaintyEstimator> uncertainty, Options options)
    : preprocessor_(std::move(preprocessor)),
      model_(std::move(model)),
      pooler_(std::move(pooler)),
      uncertainty_(std::move(uncertainty)),
      options_(options)
{
}

common::Status DepthFoundationModelEncoder::Encode(
    const observation::Observation& observation, observation::DepthFeature* feature)
{
    if (feature == nullptr || preprocessor_ == nullptr || model_ == nullptr ||
        pooler_ == nullptr || uncertainty_ == nullptr)
    {
        return common::Status::InvalidArgument("encoder is not configured");
    }
    if (!observation.has_depth())
    {
        return common::Status::InvalidArgument("observation has no depth");
    }
    std::vector<float> input;
    DepthStats stats;
    if (!preprocessor_->Process(observation.depth(), &input, &stats))
    {
        return common::Status::InvalidArgument("unsupported depth image");
    }
    if (stats.valid_ratio < options_.min_valid_ratio)
    {
        return common::Status::Unavailable("insufficient valid depth");
    }

    DefmOutput output;
    if (!model_->Run(input, &output))
    {
        return common::Status::Internal("DeFM inference failed");
    }
    if (output.token_count == 0 || output.token_dim == 0 ||
        output.tokens.size() !=
            static_cast<std::size_t>(output.token_count) * output.token_dim ||
        (!output.pooled.empty() && output.pooled.size() != output.token_dim))
    {
        return common::Status::Internal("DeFM output shape is invalid");
    }

    std::vector<float> pooled;
    double token_norm = 0.0;
    if (!output.pooled.empty())
    {
        pooled = output.pooled;
        double norm_squared = 0.0;
        for (const float value : pooled)
        {
            norm_squared += static_cast<double>(value) * value;
        }
        token_norm = std::sqrt(norm_squared);
    }
    else if (!pooler_->Pool(output.tokens, output.token_count, output.token_dim, &pooled,
                            &token_norm))
    {
        return common::Status::Internal("DeFM pooling failed");
    }

    feature->clear_tokens();
    feature->clear_pooled();
    for (float value : output.tokens)
    {
        feature->add_tokens(value);
    }
    for (float value : pooled)
    {
        feature->add_pooled(value);
    }
    feature->set_token_count(output.token_count);
    feature->set_token_dim(output.token_dim);
    feature->set_uncertainty(
        uncertainty_->Estimate(1.0 - stats.valid_ratio, token_norm, 0.0));
    feature->set_stamp_ns(static_cast<std::int64_t>(
        automsgs::msgs::builtin_interfaces::TimeToNanoseconds(
            observation.header().stamp())));
    feature->set_sequence(observation.sequence());
    return common::Status{};
}

void DepthFoundationModelEncoder::Reset()
{
}

}  // namespace perception
}  // namespace navigation
