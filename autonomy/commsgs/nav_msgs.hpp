/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/proto/nav_msgs.pb.h"
#include "autonomy/commsgs/std_msgs.hpp"

namespace autonomy {
namespace commsgs {
namespace nav_msgs {

struct SpeedLimit {
    AUTONOMY_SMART_PTR_DEFINITIONS(SpeedLimit)

    std_msgs::Header header;
    bool percentage{false};
    float speed_limit{0.0F};
};

proto::nav_msgs::SpeedLimit ToProto(const SpeedLimit& data);
SpeedLimit FromProto(const proto::nav_msgs::SpeedLimit& proto);

}  // namespace nav_msgs
}  // namespace commsgs
}  // namespace autonomy
