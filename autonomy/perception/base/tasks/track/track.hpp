/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AUTONOMY_PERCEPTION_BASE_TASKS_TRACK_TRACK_HPP_
#define AUTONOMY_PERCEPTION_BASE_TASKS_TRACK_TRACK_HPP_

#include "autonomy/perception/base/frame.hpp"
#include "autonomy/perception/base/proto/base.pb.h"

#include <automsgs/msgs/sensor_msgs/image.pb.h>

#include <string>

namespace autonomy {
namespace common {
namespace network {
class Engine;
}  // namespace network
}  // namespace common

namespace perception {
namespace base {
namespace track {

/**
 * @brief Associate detections across frames into stable track ids.
 *
 * May reuse the detect graph; temporal association lives here.
 */
bool Decode(const common::network::Engine& engine,
            const proto::BaseOptions& options,
            const automsgs::msgs::sensor_msgs::Image& rgb, Outputs* outputs,
            std::string* error);

}  // namespace track
}  // namespace base
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_BASE_TASKS_TRACK_TRACK_HPP_
