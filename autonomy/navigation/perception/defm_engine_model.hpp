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
 * @file defm_engine_model.hpp
 * @brief DeFM backend on autonomy::common::network::Engine.
 */

#pragma once

#include <memory>
#include <string>

#include "autonomy/common/network/backend/engine.hpp"
#include "navigation/perception/defm_model.hpp"

namespace navigation
{
namespace perception
{

/**
 * @brief DeFM backend on autonomy::common::network::Engine.
 *
 * @par Threading
 * Load once; Run from the navigation tick thread only.
 */
class EngineDefmModel final : public DefmModel
{
public:
    /** @brief Load the ONNX/TensorRT engine described by the manifest. */
    bool Load(const config::ModelManifest& manifest, std::string* error) override;

    /** @brief Run one forward pass through the Engine. */
    bool Run(const std::vector<float>& input, DefmOutput* output) override;

    /** @brief Backend name ("defm_engine"). */
    [[nodiscard]] std::string_view Name() const override;

private:
    /** @brief Owned inference engine. */
    std::unique_ptr<autonomy::common::network::Engine> engine_{nullptr};
    /** @brief Manifest input name for the depth tensor. */
    std::string image_input_;
    /** @brief Manifest output name for tokens. */
    std::string tokens_output_;
    /** @brief Manifest output name for the pooled embedding. */
    std::string pooled_output_;
};

/** @brief Factory used when a model manifest is available. */
std::unique_ptr<DefmModel> CreateEngineDefmModel(const config::ModelManifest& manifest,
                                                 std::string* error);

}  // namespace perception
}  // namespace navigation
