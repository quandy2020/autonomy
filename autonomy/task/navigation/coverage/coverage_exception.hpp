// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file coverage_exception.hpp
 * @brief Coverage-stage error category. */
#pragma once
#include <stdexcept>
namespace autonomy::task::coverage {
/** @brief CoverageException with a diagnostic reason. */
class CoverageException : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

}  // namespace autonomy::task::coverage
