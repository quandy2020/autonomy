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

#include "navigation/planning/plugins/policy/vntpa/engine_model.hpp"

#include <algorithm>
#include <utility>

namespace navigation
{
namespace planning
{
namespace vntpa
{

EngineVntpaModel::EngineVntpaModel(plugins::VntpaPolicyOptions options)
    : options_(std::move(options))
{
}

bool EngineVntpaModel::Load(std::string* error)
{
    autonomy::common::network::InferenceOptions inference;
    inference.model_path = options_.model_path();
    inference.backend_id = options_.execution_provider() == "tensorrt" ? "tensorrt" : "onnx";
    engine_ = autonomy::common::network::Engine::CreateEngine(inference, error);
    return engine_ != nullptr;
}

bool EngineVntpaModel::Run(const ModelInputs& inputs, std::vector<float>* actions,
                           std::uint32_t* chunk_length)
{
    if (engine_ == nullptr || actions == nullptr || chunk_length == nullptr)
    {
        return false;
    }
    autonomy::common::network::FloatTensorMap tensors;
    tensors[options_.depth_input_name()] = inputs.depth_feature;
    tensors[options_.pose_input_name()] = inputs.query;
    tensors[options_.context_input_name()] = inputs.context;
    tensors[options_.context_pose_input_name()] = inputs.context_pose;
    tensors[options_.context_mask_input_name()] =
        std::vector<float>(inputs.context_mask.begin(), inputs.context_mask.end());

    autonomy::common::network::FloatTensorMap outputs;
    if (!engine_->Run(tensors, &outputs))
    {
        return false;
    }
    const auto iterator = outputs.find(options_.output_name());
    if (iterator == outputs.end() || iterator->second.empty())
    {
        return false;
    }
    *actions = iterator->second;
    *chunk_length = options_.action_chunk_length() == 0 ? 8 : options_.action_chunk_length();
    return true;
}

EngineVntpaModelLoader::EngineVntpaModelLoader(plugins::VntpaPolicyOptions options)
    : ModelLoader(std::move(options))
{
}

bool EngineVntpaModelLoader::Load(std::string* error)
{
    auto model = std::make_unique<EngineVntpaModel>(options_);
    if (!model->Load(error))
    {
        return false;
    }
    model_ = std::move(model);
    return true;
}

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
