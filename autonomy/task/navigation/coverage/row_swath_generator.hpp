// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file row_swath_generator.hpp
 * @brief Convert annotated rows to operational swaths. */
#pragma once
#include "autonomy/task/navigation/coverage/types.hpp"
namespace autonomy::task::coverage {
/** @brief Generate CENTER, OFFSET or ROWSARESWATHS paths with skip identifiers.
 */
F2CSwaths GenerateRowSwaths(const proto::CoverageRequest&,
                            const F2CPoint& reference);

}  // namespace autonomy::task::coverage
