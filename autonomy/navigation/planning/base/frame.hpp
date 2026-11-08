/*
 * Copyright 2026 The Navigation Authors (navigation)
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
 * @file frame.hpp
 * @brief Minimal pose/velocity frame used by stage helpers.
 */

#pragma once

#include <cstdint>

#include "navigation/common/pose2d.hpp"
#include "navigation/common/twist2d.hpp"

namespace navigation
{
namespace planning
{

/** @brief Minimal pose/velocity frame. */
struct Frame
{
    /** @brief Map-frame planar pose. */
    common::Pose2D pose;
    /** @brief Measured planar velocity. */
    common::Twist2D velocity;
    /** @brief Monotonic timestamp in nanoseconds. */
    std::uint64_t stamp_ns{0};
};

}  // namespace planning
}  // namespace navigation
