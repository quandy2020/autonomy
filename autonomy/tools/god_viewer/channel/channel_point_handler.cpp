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

#include "autonomy/tools/god_viewer/channel/channel_point_handler.hpp"

#include <vector>
#include <cmath>
#include <random>
#include <fstream>
#include <chrono>

#include "autonomy/common/logging.hpp"
#include "autonomy/common/math/random.hpp"

namespace autonomy {
namespace tools { 
namespace god_viewer { 
namespace channel {

PointHandler::PointHandler(ServerHander::SharedPtr options, const std::string& topic)
    : topic_{topic}
{
    LOG(INFO) << "Init handler topic: " << topic_;
    channel_ = std::make_unique<foxglove::schemas::Point3Channel>(
        foxglove::schemas::Point3Channel::create(topic_).value());
}

bool PointHandler::SendTest()
{
    commsgs::geometry_msgs::Point msgs;
    msgs.x = common::math::RandomUniformInteger(0, 100);
    msgs.y = common::math::RandomUniformInteger(0, 50);
    msgs.z = common::math::RandomUniformInteger(0, 60);

    Send(msgs);

    return true;
}

bool PointHandler::Send(const commsgs::geometry_msgs::Point& msgs)
{
    auto data = FromCommsgs(msgs);
    channel_->log(data);
    return true;
}


foxglove::schemas::Point3 PointHandler::FromCommsgs(const commsgs::geometry_msgs::Point& msgs)
{
    foxglove::schemas::Point3 point;
    point.x = msgs.x;
    point.y = msgs.y;
    point.z = msgs.z;
    return point;
}

}   // channel
}   // god_viewer
}   // namespace tools
}   // namespace autonomy