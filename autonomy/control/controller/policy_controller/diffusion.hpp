/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

/**
 * @file diffusion.hpp
 * @brief Iterative denoising loop for INFERENCE_DIFFUSION.
 */

#pragma once

#include <string>
#include <vector>

#include "autonomy/common/network/common/tensor.hpp"
#include "autonomy/control/controller/policy_controller/config.hpp"
#include "autonomy/control/controller/policy_controller/model.hpp"
#include "autonomy/control/controller/policy_controller/types.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

/**
 * @brief Run diffusion / iterative generative inference for one control tick.
 *
 * Expected step graph (see @c Diffusion):
 * - Inputs: observation_* + sample + timestep
 * - Output: prediction (SAMPLE=x0 or EPSILON)
 *
 * Builds a working input map once and only refreshes sample / timestep slots
 * each denoising iteration.
 *
 * @param[in] model Loaded denoising-step policy model.
 * @param[in] condition_inputs Observation (and other fixed) tensors.
 * @param[out] action_sequence Flattened [horizon * dim].
 * @param[out] error Optional failure reason.
 * @return true on success.
 */
bool RunDiffusionInference(
    const PolicyModel& model,
    const common::network::FloatTensorMap& condition_inputs,
    std::vector<float>* action_sequence, std::string* error = nullptr,
    InferBudget* budget = nullptr);

/**
 * @brief Fill defaults for diffusion options (called at config load).
 * @param proto Raw policy config proto.
 * @return Options with empty names / non-positive dims filled.
 */
proto::Diffusion MakeEffectiveDiffusion(const proto::PolicyConfig& proto);

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
