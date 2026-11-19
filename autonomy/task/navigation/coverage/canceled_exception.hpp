// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file canceled_exception.hpp
 * @brief Coverage-stage error category. */
#pragma once
#include <stdexcept>
namespace autonomy::task::coverage {
/** @brief CanceledException with a diagnostic reason. */
class CanceledException : public std::runtime_error {
 public:
  CanceledException() : std::runtime_error("Coverage request canceled") {}
};

}  // namespace autonomy::task::coverage
