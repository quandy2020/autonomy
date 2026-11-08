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

#include "navigation/perception/observation_builder.hpp"

namespace navigation
{
namespace perception
{

bool ObservationBuilder::Build(const ObservationInputs& inputs,
                               observation::Observation* observation)
{
    if (observation == nullptr || inputs.depth == nullptr ||
        inputs.depth->width() == 0 || inputs.depth->height() == 0 ||
        inputs.depth->data().empty())
    {
        return false;
    }
    observation->Clear();
    *observation->mutable_depth() = *inputs.depth;
    *observation->mutable_header() = inputs.depth->header();
    observation->set_sequence(++sequence_);
    observation->set_map_frame("map");
    observation->set_camera_frame("camera_optical_frame");

    std::uint32_t validity = observation::OBSERVATION_FLAG_DEPTH_VALID;
    if (inputs.camera_info != nullptr)
    {
        *observation->mutable_camera_info() = *inputs.camera_info;
        validity |= observation::OBSERVATION_FLAG_CAMERA_INFO_VALID;
    }
    if (inputs.odom != nullptr)
    {
        *observation->mutable_odom() = *inputs.odom;
        validity |= observation::OBSERVATION_FLAG_ODOM_VALID;
    }
    if (inputs.imu != nullptr)
    {
        *observation->mutable_imu() = *inputs.imu;
        validity |= observation::OBSERVATION_FLAG_IMU_VALID;
    }
    if (inputs.tf_valid)
    {
        validity |= observation::OBSERVATION_FLAG_TF_VALID;
    }
    observation->set_validity(validity);
    return true;
}

void ObservationBuilder::Reset()
{
    sequence_ = 0;
}

std::uint64_t ObservationBuilder::LastSequence() const
{
    return sequence_;
}

}  // namespace perception
}  // namespace navigation
