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

#include "navigation/perception/frontiernet_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "navigation/common/configuration_loader.hpp"
#include "navigation/common/model_integrity.hpp"

namespace navigation
{
namespace perception
{
namespace
{

/** @brief True when the manifest describes a square FrontierNet graph. */
bool ReadShape(const config::TensorSpec& spec, std::uint32_t* size,
               std::uint32_t* classes, std::size_t* count)
{
    const auto& shape = spec.shape();
    if (shape.size() != 4 || shape[0] != 1)
    {
        return false;
    }
    if (shape[1] <= 0 || shape[2] <= 0 || shape[3] != shape[2])
    {
        return false;
    }
    *classes = static_cast<std::uint32_t>(shape[1]);
    *size = static_cast<std::uint32_t>(shape[2]);
    *count = static_cast<std::size_t>(shape[1]) *
             static_cast<std::size_t>(shape[2]) *
             static_cast<std::size_t>(shape[3]);
    return true;
}

}  // namespace

bool FrontierNetModel::Load(const config::ModelManifest& manifest,
                            std::string* error)
{
    engine_.reset();
    if (manifest.path().empty() || manifest.inputs_size() != 1 ||
        manifest.outputs_size() != 2)
    {
        if (error != nullptr)
        {
            *error = "FrontierNet manifest needs a path, one input and two outputs";
        }
        return false;
    }
    depth_input_ = manifest.inputs(0).name();
    df_output_ = manifest.outputs(0).name();
    masks_output_ = manifest.outputs(1).name();
    if (depth_input_.empty() || df_output_.empty() || masks_output_.empty())
    {
        if (error != nullptr)
        {
            *error = "FrontierNet manifest has an empty tensor name";
        }
        return false;
    }
    std::uint32_t input_channels = 0;
    std::uint32_t input_size = 0;
    std::size_t input_count = 0;
    if (!ReadShape(manifest.inputs(0), &input_size, &input_channels,
                   &input_count) ||
        input_channels != 3)
    {
        if (error != nullptr)
        {
            *error = "FrontierNet input must be [1, 3, S, S]";
        }
        return false;
    }
    std::uint32_t df_size = 0;
    std::uint32_t df_channels = 0;
    std::size_t df_count = 0;
    std::uint32_t mask_size = 0;
    std::uint32_t mask_channels = 0;
    std::size_t mask_count = 0;
    if (!ReadShape(manifest.outputs(0), &df_size, &df_channels, &df_count) ||
        df_channels != 1 || df_size != input_size ||
        !ReadShape(manifest.outputs(1), &mask_size, &mask_channels,
                   &mask_count) ||
        mask_size != input_size || mask_channels < 2)
    {
        if (error != nullptr)
        {
            *error = "FrontierNet outputs must be [1,1,S,S] and [1,C,S,S]";
        }
        return false;
    }
    autonomy::common::network::InferenceOptions inference;
    inference.model_path = common::ResolveConfigPath(manifest.path());
    const auto integrity =
        common::ValidateModelIntegrity(inference.model_path, manifest.sha256());
    if (!integrity.Ok())
    {
        if (error != nullptr)
        {
            *error = integrity.message;
        }
        return false;
    }
    inference.backend_id =
        manifest.backend().empty() ? "onnx" : manifest.backend();
    engine_ = autonomy::common::network::Engine::CreateEngine(inference, error);
    if (engine_ == nullptr)
    {
        return false;
    }
    auto matches = [](const auto& specifications, const auto& actual) {
        if (specifications.size() != static_cast<int>(actual.size()))
        {
            return false;
        }
        for (const auto& specification : specifications)
        {
            const auto info = std::find_if(
                actual.begin(), actual.end(), [&](const auto& value) {
                    return value.name == specification.name();
                });
            if (info == actual.end() || specification.dtype() != "float32" ||
                info->element_type !=
                    autonomy::common::network::ElementType::kFloat32 ||
                info->shape.Dims() !=
                    std::vector<std::int64_t>(specification.shape().begin(),
                                              specification.shape().end()))
            {
                return false;
            }
        }
        return true;
    };
    if (!matches(manifest.inputs(), engine_->GetInputInfos()) ||
        !matches(manifest.outputs(), engine_->GetOutputInfos()))
    {
        if (error != nullptr)
        {
            *error = "FrontierNet tensor metadata does not match its manifest";
        }
        engine_.reset();
        return false;
    }
    input_count_ = input_count;
    df_count_ = df_count;
    masks_count_ = mask_count;
    size_ = input_size;
    classes_ = mask_channels;
    return true;
}

bool FrontierNetModel::Run(const std::vector<float>& input,
                           FrontierNetOutput* output)
{
    if (engine_ == nullptr || output == nullptr || input.size() != input_count_ ||
        !std::all_of(input.begin(), input.end(),
                     [](float value) { return std::isfinite(value); }))
    {
        return false;
    }
    autonomy::common::network::FloatTensorMap tensors;
    tensors[depth_input_] = input;
    autonomy::common::network::FloatTensorMap results;
    if (!engine_->Run(tensors, &results))
    {
        return false;
    }
    const auto df = results.find(df_output_);
    const auto masks = results.find(masks_output_);
    if (df == results.end() || masks == results.end() ||
        df->second.size() != df_count_ || masks->second.size() != masks_count_)
    {
        return false;
    }
    output->df = df->second;
    output->masks = masks->second;
    output->classes = classes_;
    output->size = size_;
    return true;
}

bool FrontierNetModel::Ready() const
{
    return engine_ != nullptr;
}

std::uint32_t FrontierNetModel::InputSize() const
{
    return size_;
}

std::uint32_t FrontierNetModel::Classes() const
{
    return classes_;
}

std::string_view FrontierNetModel::Name() const
{
    return "frontiernet_engine";
}

std::unique_ptr<FrontierNetModel> CreateFrontierNetModel(
    const config::ModelManifest& manifest, std::string* error)
{
    auto model = std::make_unique<FrontierNetModel>();
    if (!model->Load(manifest, error))
    {
        return nullptr;
    }
    return model;
}

}  // namespace perception
}  // namespace navigation
