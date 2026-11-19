// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file request_exception.hpp
 * @brief Coverage-stage error category. */
#pragma once
#include <stdexcept>
namespace autonomy::task::coverage {
/** @brief RequestException with a diagnostic reason. */
class RequestException : public std::invalid_argument {
 public:
  using std::invalid_argument::invalid_argument;
};

}  // namespace autonomy::task::coverage
