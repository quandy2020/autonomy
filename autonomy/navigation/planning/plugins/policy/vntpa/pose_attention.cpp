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

#include "navigation/planning/plugins/policy/vntpa/pose_attention.hpp"

#include <cmath>

namespace navigation
{
namespace planning
{
namespace vntpa
{

PoseAttention::PoseAttention(std::uint32_t output_dim)
    : output_dim_(output_dim < 4 ? 4 : output_dim)
{
}

bool PoseAttention::Encode(const common::Pose2D& relative_pose,
                           std::vector<float>* encoding) const
{
    if (encoding == nullptr)
    {
        return false;
    }
    encoding->assign(output_dim_, 0.0F);
    const double values[4] = {relative_pose.x(), relative_pose.y(),
                              std::sin(relative_pose.theta()),
                              std::cos(relative_pose.theta())};
    for (std::uint32_t dim = 0; dim < output_dim_; ++dim)
    {
        const std::uint32_t base = dim % 4U;
        const std::uint32_t band = dim / 4U;
        const double frequency = std::pow(2.0, static_cast<double>(band) / 4.0);
        const double scaled = values[base] * frequency;
        (*encoding)[dim] = static_cast<float>((base < 2U) ? std::tanh(scaled)
                                                          : std::sin(scaled));
    }
    return true;
}

std::uint32_t PoseAttention::OutputDim() const
{
    return output_dim_;
}

}  // namespace vntpa
}  // namespace planning
}  // namespace navigation
