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

#include "navigation/planning/engine_plugins.hpp"

#include <memory>
#include <string>

#include "navigation/common/configuration_loader.hpp"
#include "navigation/perception/defm_engine_model.hpp"
#include "navigation/perception/defm_encoder.hpp"
#include "navigation/planning/plugin_manager.hpp"
#include "navigation/planning/plugins/policy/vntpa/engine_model.hpp"
#include "navigation/planning/plugins/policy/vntpa/policy.hpp"

namespace navigation
{
namespace planning
{
namespace
{

std::unique_ptr<perception::ObservationEncoder> CreateEngineEncoder(
    const config::NavigationConfig& configuration)
{
    config::ModelManifest manifest;
    const std::string path = configuration.perception().manifest_path().empty()
                                 ? "conf/models/vntpa.pb.txt"
                                 : configuration.perception().manifest_path();
    if (!common::LoadTextProto(path, &manifest).Ok())
    {
        return nullptr;
    }
    if (!configuration.perception().model_path().empty())
    {
        manifest.set_path(configuration.perception().model_path());
    }
    std::string error;
    auto model = perception::CreateEngineDefmModel(manifest, &error);
    if (model == nullptr)
    {
        return nullptr;
    }
    perception::DepthPreprocessorOptions preprocess;
    if (manifest.mean_size() == 3 && manifest.std_size() == 3)
    {
        for (int i = 0; i < 3; ++i)
        {
            preprocess.mean[static_cast<std::size_t>(i)] = manifest.mean(i);
            preprocess.std[static_cast<std::size_t>(i)] = manifest.std(i);
        }
    }
    if (manifest.inputs_size() > 0 && manifest.inputs(0).shape_size() >= 3)
    {
        const auto size = manifest.inputs(0).shape(2);
        if (size > 0)
        {
            preprocess.input_size = static_cast<std::uint32_t>(size);
        }
    }
    return std::make_unique<perception::DepthFoundationModelEncoder>(
        std::make_unique<perception::DepthPreprocessor>(preprocess),
        std::move(model), std::make_unique<perception::AttentionPooler>(),
        std::make_unique<perception::PerceptualUncertaintyEstimator>(),
        perception::DepthFoundationModelEncoder::Options{
            configuration.perception().min_valid_ratio()});
}

std::unique_ptr<PolicyInterface> CreateEnginePolicy(
    const config::NavigationConfig& configuration)
{
    if (!configuration.policy().has_vntpa())
    {
        return nullptr;
    }
    const auto& options = configuration.policy().vntpa();
    auto loader = std::make_unique<vntpa::EngineVntpaModelLoader>(options);
    config::ModelManifest manifest;
    const std::string path = configuration.perception().manifest_path().empty()
                                 ? "conf/models/vntpa.pb.txt"
                                 : configuration.perception().manifest_path();
    std::uint32_t embedding_dim = 384;
    if (common::LoadTextProto(path, &manifest).Ok() && manifest.embedding_dim() > 0)
    {
        embedding_dim = manifest.embedding_dim();
    }
    const std::uint32_t context = options.context_size_k() == 0 ? 64 : options.context_size_k();
    const double forward_step = options.forward_step() > 0.0 ? options.forward_step() : 0.25;
    const double turn_step = options.turn_step_rad() > 0.0 ? options.turn_step_rad() : 0.5235987756;
    const double dt_s = options.action_dt_s() > 0.0 ? options.action_dt_s() : 0.5;
    return std::make_unique<vntpa::Policy>(
        options, std::move(loader),
        std::make_unique<vntpa::Preprocessor>(context, embedding_dim, 64),
        std::make_unique<vntpa::Postprocessor>(
            vntpa::PostprocessorOptions{options.action_chunk_length(), dt_s, 0.5, 0.0, 1.0,
                                        forward_step, turn_step}),
        std::make_unique<vntpa::ActionDecoder>(
            vntpa::ActionDecoderOptions{4, forward_step, turn_step, dt_s}));
}

}  // namespace

void RegisterEnginePlugins()
{
    PluginManager& plugins = PluginManager::Instance();
    plugins.RegisterEncoder("defm", [](const config::NavigationConfig& configuration) {
        return CreateEngineEncoder(configuration);
    });
    plugins.RegisterPolicy("vntpa", [](const config::NavigationConfig& configuration) {
        return CreateEnginePolicy(configuration);
    });
}

}  // namespace planning
}  // namespace navigation
