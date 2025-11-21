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

#include "autonomy/common/logging.hpp"
#include "autonomy/common/math/random.hpp"
#include "autonomy/visualization/displays/laser_scan.hpp"
#include "autonomy/visualization/msgs_converter.hpp"

namespace autonomy {
namespace visualization {
namespace displays {

LaserScanHandler::LaserScanHandler(const std::string& topic)
    : channel::ChannelBase(topic)
{
    LOG(INFO) << "Init handler topic: " << this->topic();
    channel_ = std::make_unique<foxglove::schemas::LaserScanChannel>(
        foxglove::schemas::LaserScanChannel::create(this->topic()).value());
}

bool LaserScanHandler::SendTest()
{
    commsgs::sensor_msgs::LaserScan msgs;
    msgs.header.frame_id = "base_link";
    msgs.angle_min = -M_PI/2;
    msgs.angle_max = M_PI/2;
    msgs.angle_increment = M_PI/180;
    msgs.ranges = {1.0, 1.1, 1.2, 1.3, 1.4};
    msgs.intensities = {10.0, 11.0, 12.0, 13.0, 14.0};

    return Send(msgs);
}

bool LaserScanHandler::Send(const commsgs::sensor_msgs::LaserScan& msgs)
{
    if (channel_->log(FromCommsgs(msgs)) != foxglove::FoxgloveError::Ok) {
        LOG(ERROR) << "Send commsgs::geometry_msgs::Point msgs error.";
        return false;
    }
    return true;
}

}   // namespace displays 
}   // namespace visualization
}   // namespace autonomy