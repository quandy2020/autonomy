/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

/**
 * @file depth_encoder.hpp
 * @brief Pack a depth-image history into a float tensor (SanD-Planner).
 */

#pragma once

#include <string>
#include <vector>

#include "autonomy/control/controller/policy_controller/types.hpp"
#include "autonomy/control/proto/policy_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

/**
 * @brief Encode depth history as [1, T, H, W] metres (oldest frame first).
 *
 * Accepts 32FC1 (metres) or 16UC1 (millimetres). Missing early frames repeat
 * the oldest available image. @p context.sand_planner supplies T/H/W.
 *
 * @param[in] snapshot Must contain depth_history or a depth image.
 * @param[in] input Policy input spec (shape overrides T/H/W when set).
 * @param[in] options Effective SanD-Planner geometry.
 * @param[out] output Flattened NCHW float tensor.
 * @param[out] error Optional failure reason.
 */
bool EncodeDepth(const ObservationSnapshot& snapshot,
                 const proto::ObservationInput& input,
                 const proto::SandPlanner& options, std::vector<float>* output,
                 std::string* error);

/**
 * @brief Encode the path-tail goal in the robot frame as [1, 3].
 */
bool EncodeGoal(const ObservationSnapshot& snapshot,
                const proto::ObservationInput& input, std::vector<float>* output,
                std::string* error);

/**
 * @brief Encode the previous-plan heading token as [1, 3].
 *
 * Uses @p context.previous_plan_heading when set; otherwise a zero vector
 * (null token at t = 0).
 */
bool EncodePreviousVelocity(const ObservationEncodeContext& context,
                            const proto::ObservationInput& input,
                            std::vector<float>* output, std::string* error);

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
