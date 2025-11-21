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
#include "autonomy/visualization/displays/tf.hpp"
#include "autonomy/visualization/msgs_converter.hpp"

namespace autonomy {
namespace visualization {
namespace displays {

FrameTransformsHandler::FrameTransformsHandler(const std::string& topic)
    : channel::ChannelBase(topic)
{
    LOG(INFO) << "Init handler topic: " << this->topic();
    channel_ = std::make_unique<foxglove::schemas::FrameTransformsChannel>(
        foxglove::schemas::FrameTransformsChannel::create(this->topic()).value());
}

bool FrameTransformsHandler::SendTest()
{
    commsgs::geometry_msgs::TransformStampeds msgs;
    msgs.header.stamp = Time::Now();
    msgs.header.frame_id = "map";

    commsgs::geometry_msgs::TransformStamped transform;
    transform.header.stamp = Time::Now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    msgs.transforms.push_back(transform);


    transform.header.stamp = Time::Now();
    transform.header.frame_id = "base_link";
    transform.child_frame_id = "lidar";
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.5;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;


    msgs.transforms.push_back(transform);
    return Send(msgs);
}

bool FrameTransformsHandler::Send(const commsgs::geometry_msgs::TransformStampeds& msgs)
{
    if (channel_->log(FromCommsgs(msgs)) != foxglove::FoxgloveError::Ok) {
        LOG(ERROR) << "Send commsgs::geometry_msgs::TransformStampeds msgs error.";
        return false;
    }
    return true;
}

}   // namespace displays 
}   // namespace visualization
}   // namespace autonomy
