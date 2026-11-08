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
 * @file pose_attention.hpp
 * @brief Fixed-frequency Fourier encoding of relative poses.
 */

#pragma once

#include <cstdint>
#include <vector>

#include "navigation/common/pose2d.hpp"

namespace navigation
{
namespace planning
{
namespace vntpa
{

class PoseAttention
{
public:
    /** @brief Construct with the output feature dimension. */
    explicit PoseAttention(std::uint32_t output_dim = 64);

    /** @brief Encode a relative pose into a Fourier feature vector. */
    bool Encode(const common::Pose2D& relative_pose, std::vector<float>* encoding) const;

    /** @brief Configured output dimension. */
    [[nodiscard]] std::uint32_t OutputDim() const;

private:
    std::uint32_t output_dim_{64};
};

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
