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

#include "navigation/perception/defm_engine_model.hpp"

#include <algorithm>

#include "navigation/common/configuration_loader.hpp"

namespace navigation
{
namespace perception
{

bool EngineDefmModel::Load(const config::ModelManifest& manifest, std::string* error)
{
    if (manifest.path().empty() || manifest.inputs_size() == 0 ||
        manifest.outputs_size() < 2)
    {
        if (error != nullptr)
        {
            *error = "DeFM manifest needs a path, one input and two outputs";
        }
        return false;
    }
    image_input_ = manifest.inputs(0).name();
    tokens_output_ = manifest.outputs(0).name();
    pooled_output_ = manifest.outputs(1).name();
    if (image_input_.empty() || tokens_output_.empty() || pooled_output_.empty())
    {
        if (error != nullptr)
        {
            *error = "DeFM manifest has an empty tensor name";
        }
        return false;
    }
    const auto& input_shape = manifest.inputs(0).shape();
    if (input_shape.size() != 4 || input_shape[0] != 1 || input_shape[1] != 3 ||
        input_shape[2] <= 0 || input_shape[3] <= 0)
    {
        if (error != nullptr)
        {
            *error = "DeFM input must be [1, 3, H, W]";
        }
        return false;
    }
    const auto& token_shape = manifest.outputs(0).shape();
    const auto& pooled_shape = manifest.outputs(1).shape();
    if (token_shape.size() != 3 || token_shape[0] != 1 ||
        pooled_shape.size() != 2 || pooled_shape[0] != 1 ||
        token_shape[2] != pooled_shape[1])
    {
        if (error != nullptr)
        {
            *error = "DeFM token/pooled shapes are inconsistent";
        }
        return false;
    }
    autonomy::common::network::InferenceOptions inference;
    inference.model_path = common::ResolveConfigPath(manifest.path());
    inference.backend_id = manifest.backend().empty() ? "onnx" : manifest.backend();
    engine_ = autonomy::common::network::Engine::CreateEngine(inference, error);
    return engine_ != nullptr;
}

bool EngineDefmModel::Run(const std::vector<float>& input, DefmOutput* output)
{
    if (engine_ == nullptr || output == nullptr || input.empty())
    {
        return false;
    }
    autonomy::common::network::FloatTensorMap tensors;
    tensors[image_input_] = input;
    autonomy::common::network::FloatTensorMap results;
    if (!engine_->Run(tensors, &results))
    {
        return false;
    }
    const auto tokens = results.find(tokens_output_);
    const auto pooled = results.find(pooled_output_);
    if (tokens == results.end() || pooled == results.end() || pooled->second.empty())
    {
        return false;
    }
    output->tokens = tokens->second;
    output->pooled = pooled->second;
    output->token_dim = static_cast<std::uint32_t>(pooled->second.size());
    if (output->token_dim == 0 || output->tokens.size() % output->token_dim != 0)
    {
        return false;
    }
    output->token_count =
        static_cast<std::uint32_t>(output->tokens.size() / output->token_dim);
    return output->token_count > 0;
}

std::string_view EngineDefmModel::Name() const
{
    return "defm_engine";
}

std::unique_ptr<DefmModel> CreateEngineDefmModel(const config::ModelManifest& manifest,
                                                 std::string* error)
{
    auto model = std::make_unique<EngineDefmModel>();
    if (!model->Load(manifest, error))
    {
        return nullptr;
    }
    return model;
}

}  // namespace perception
}  // namespace navigation
