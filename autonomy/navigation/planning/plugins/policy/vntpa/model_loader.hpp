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
 * @file model_loader.hpp
 * @brief Loads the VNTPA transformer behind autonomy::common::network::Engine.
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "navigation/planning/plugins/policy/vntpa/preprocessor.hpp"
#include "navigation/proto/plugins.pb.h"

namespace navigation
{
namespace planning
{
namespace vntpa
{

/** @brief VNTPA transformer backend. */
class VntpaModel
{
public:
    virtual ~VntpaModel() = default;

    virtual bool Run(const ModelInputs& inputs, std::vector<float>* actions,
                     std::uint32_t* chunk_length) = 0;
};

/**
 * @brief Owns the inference engine and validates configured tensor names.
 *
 * @par Ownership
 * Policy owns one ModelLoader; the model is never shared across policies.
 */
class ModelLoader
{
public:
    /** @brief Construct with plugin options. */
    explicit ModelLoader(plugins::VntpaPolicyOptions options);

    virtual ~ModelLoader() = default;

    /** @brief Load the backend; base implementation reports it is unlinked. */
    virtual bool Load(std::string* error);

    /** @brief Loaded model, or nullptr. */
    [[nodiscard]] virtual VntpaModel* Model();

    /** @brief Options used by this loader. */
    [[nodiscard]] const plugins::VntpaPolicyOptions& Options() const;

protected:
    /** @brief Options snapshot. */
    plugins::VntpaPolicyOptions options_;
    /** @brief Owned model; created by Load. */
    std::unique_ptr<VntpaModel> model_{nullptr};
};

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
