// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file segment.hpp
 * @brief One bounded native path with uniform tool and driving direction. */
#pragma once
#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/** @brief Exact geometry chunk and its operational semantics. */
struct Segment {
  NavPath path;          ///< Exact bounded path geometry.
  bool work{false};      ///< Tool operates along this segment.
  bool backward{false};  ///< Vehicle driving direction.
  double velocity{0};    ///< Requested speed magnitude in metres per second.
  double length{0};      ///< Segment arclength in metres.
};
}  // namespace autonomy::task::coverage
