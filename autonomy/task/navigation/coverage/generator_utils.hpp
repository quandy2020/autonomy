// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file generator_utils.hpp
 * @brief Numeric validation shared by coverage stages. */
#pragma once
#include <cmath>
#include <limits>
#include <set>

#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/** @brief Reject nonpositive or nonfinite lengths and speed parameters. */
inline void Positive(double value, const char* name) {
  if (!std::isfinite(value) || value <= 0.0)
    throw RequestException(std::string(name) + " must be positive and finite");
}
/** @brief Reject a nonfinite angle or coordinate. */
inline void Angle(double value) {
  if (!std::isfinite(value)) throw RequestException("Angle must be finite");
}

}  // namespace autonomy::task::coverage
