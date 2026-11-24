/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file path_segments.hpp
 * @brief Split without deleting route geometry or direction transitions.
 */

#ifndef AUTONOMY_TASK_NAVIGATION_COVERAGE_PATH_SEGMENTS_HPP_
#define AUTONOMY_TASK_NAVIGATION_COVERAGE_PATH_SEGMENTS_HPP_

#include "autonomy/task/navigation/coverage/coverage_server.hpp"
#include "autonomy/task/navigation/coverage/segment.hpp"
namespace autonomy::task::coverage {
/**
 * @brief Splits a planned result into bounded, exact geometry chunks.
 *
 * Splitting never drops a state and never reorders a segment: the concatenation
 * of the returned segments equals the planned path, so the follower sees the
 * same geometry the planner validated.
 *
 * @param[in] result Planned coverage result to split.
 * @param[in] max_poses Upper bound of poses per segment (at least one).
 * @param[in] allow_reverse Whether segments may drive backwards; when false a
 *            reverse segment is rejected instead of being flattened.
 * @return Segments in path order, each with its tool state and direction.
 * @throws CoverageException when a reverse segment is present and reverse
 *         driving is not allowed.
 */
std::vector<Segment> BuildSegments(const ComputeAction::Result& result,
                                   std::size_t max_poses, bool allow_reverse);

}  // namespace autonomy::task::coverage

#endif  // AUTONOMY_TASK_NAVIGATION_COVERAGE_PATH_SEGMENTS_HPP_
