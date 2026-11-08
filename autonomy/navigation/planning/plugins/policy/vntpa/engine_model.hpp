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
 * @file engine_model.hpp
 * @brief VNTPA backend on autonomy::common::network::Engine.
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/common/network/backend/engine.hpp"
#include "navigation/planning/plugins/policy/vntpa/model_loader.hpp"

namespace navigation
{
namespace planning
{
namespace vntpa
{

class EngineVntpaModel final : public VntpaModel
{
public:
    /** @brief Construct with plugin options. */
    explicit EngineVntpaModel(plugins::VntpaPolicyOptions options);

    /** @brief Load the Engine described by the manifest. */
    bool Load(std::string* error);

    bool Run(const ModelInputs& inputs, std::vector<float>* actions,
             std::uint32_t* chunk_length) override;

private:
    plugins::VntpaPolicyOptions options_;
    std::unique_ptr<autonomy::common::network::Engine> engine_{nullptr};
};

/** @brief Loader used when NAVIGATION_WITH_ENGINE is enabled. */
class EngineVntpaModelLoader final : public ModelLoader
{
public:
    /** @brief Construct the Engine-backed loader. */
    explicit EngineVntpaModelLoader(plugins::VntpaPolicyOptions options);

    /** @brief Load the Engine and expose it to the policy. */
    bool Load(std::string* error) override;
};

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
