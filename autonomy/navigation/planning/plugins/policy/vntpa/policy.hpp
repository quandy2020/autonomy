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
 * @file policy.hpp
 * @brief VNTPA end-to-end policy plugin.
 */

#pragma once

#include <memory>
#include <string_view>

#include "navigation/planning/interface/policy_interface.hpp"
#include "navigation/planning/plugins/policy/vntpa/action_decoder.hpp"
#include "navigation/planning/plugins/policy/vntpa/manipulation_reachability_adjuster.hpp"
#include "navigation/planning/plugins/policy/vntpa/model_loader.hpp"
#include "navigation/planning/plugins/policy/vntpa/postprocessor.hpp"
#include "navigation/planning/plugins/policy/vntpa/preprocessor.hpp"
#include "navigation/proto/plugins.pb.h"

namespace navigation
{
namespace planning
{
namespace vntpa
{

class Policy final : public PolicyInterface
{
public:
    /**
     * @brief Construct from plugin options and owned stages.
     *
     * @param[in] options VNTPA options snapshot.
     * @param[in] model_loader Engine/model loader.
     * @param[in] preprocessor Input assembler.
     * @param[in] postprocessor Output clamp/chunk builder.
     * @param[in] action_decoder Optional post-net decoder.
     * @param[in] adjuster Optional docking reachability adjuster.
     */
    Policy(plugins::VntpaPolicyOptions options,
           std::unique_ptr<ModelLoader> model_loader,
           std::unique_ptr<Preprocessor> preprocessor,
           std::unique_ptr<Postprocessor> postprocessor,
           std::unique_ptr<ActionDecoder> action_decoder,
           std::unique_ptr<ManipulationReachabilityAdjuster> adjuster = nullptr);

    /** @brief Load the model and validate the plugin name/options. */
    bool Configure(const config::PolicyOptions& options) override;

    /** @brief Build inputs, run the transformer, and decode a chunk. */
    common::StatusOr<NavigationIntent> Query(
        const observation::Observation& observation,
        const observation::DepthFeature& depth_feature, const GoalSpec& goal,
        const memory::ContextView& context) override;

    /** @brief Reset recurrent/cached state. */
    void Reset() override;

    /** @brief Plugin name ("vntpa"). */
    [[nodiscard]] std::string_view Name() const override;

private:
    /** @brief Options snapshot. */
    plugins::VntpaPolicyOptions options_;
    /** @brief Model/engine loader. */
    std::unique_ptr<ModelLoader> model_loader_{nullptr};
    /** @brief Input assembler. */
    std::unique_ptr<Preprocessor> preprocessor_{nullptr};
    /** @brief Output clamp/chunk builder. */
    std::unique_ptr<Postprocessor> postprocessor_{nullptr};
    /** @brief Optional action decoder. */
    std::unique_ptr<ActionDecoder> action_decoder_{nullptr};
    /** @brief Optional docking adjuster. */
    std::unique_ptr<ManipulationReachabilityAdjuster> adjuster_{nullptr};
    /** @brief True after Configure+Load succeed. */
    bool configured_{false};
};

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
