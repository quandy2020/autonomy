// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file path_segments.hpp
 * @brief Split without deleting route geometry or direction transitions. */
#pragma once
#include "autonomy/task/navigation/coverage/coverage_server.hpp"
#include "autonomy/task/navigation/coverage/segment.hpp"
namespace autonomy::task::coverage {
/** @brief Return exact, bounded geometry chunks and reject unsupported reverse
 * driving. */
std::vector<Segment> BuildSegments(const ComputeAction::Result&, std::size_t,
                                   bool);

}  // namespace autonomy::task::coverage
