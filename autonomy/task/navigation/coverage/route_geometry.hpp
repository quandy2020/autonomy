// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file route_geometry.hpp
 * @brief Preserve per-cell route geometry while joining coverage components. */
#pragma once
#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {

#define AWARN_UNUSED(...) \
  AWARN << "Coverage route used a checked geometry fallback"
F2CRoute RouteThroughCell(const Field&, const Swaths&, double, const int&);
F2CRoute StitchCellRoutes(const F2CCells&, const std::vector<F2CRoute>&,
                          const std::optional<F2CPoint>&, double, const int&);

}  // namespace autonomy::task::coverage
