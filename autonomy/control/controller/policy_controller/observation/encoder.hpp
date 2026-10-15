/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

/**
 * @file encoder.hpp
 * @brief Dispatch observation modalities into a FloatTensorMap.
 */

#pragma once

#include <string>

#include "autonomy/common/network/common/tensor.hpp"
#include "autonomy/control/controller/policy_controller/config.hpp"
#include "autonomy/control/controller/policy_controller/types.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

/**
 * @brief Encode all enabled modalities listed in @p config into @p inputs.
 *
 * For each @c ObservationInput:
 * - Skip if modality missing and not required
 * - Optionally enforce observation age
 * - Dispatch to kinematic / path / costmap / laser / image encoders
 * - Apply optional normalization statistics
 *
 * @param[in] snapshot Current observation view.
 * @param[in] config Loaded policy config (expanded inputs).
 * @param[in] context Runtime encode knobs.
 * @param[out] inputs Named float tensors (cleared then filled).
 * @param[out] error Optional failure reason.
 * @param[out] zero_filled Set when a required slot was zero-filled under grace.
 * @return false if a required modality is missing / stale / encode fails.
 */
bool EncodeObservations(const ObservationSnapshot& snapshot,
                        const PolicyConfig& config,
                        const ObservationEncodeContext& context,
                        ::autonomy::common::network::FloatTensorMap* inputs,
                        std::string* error, bool* zero_filled = nullptr);

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
