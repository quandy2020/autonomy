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

#include <chrono>

#include "autonomy/common/logging.hpp"
#include "autonomy/visualization/displays/image.hpp"
#include "autonomy/visualization/msgs_converter.hpp"

namespace autonomy {
namespace visualization {
namespace displays {

ImageHandler::ImageHandler(const std::string& topic)
    : channel::ChannelBase(topic)
{
    LOG(INFO) << "Init handler topic: " << this->topic();
    channel_ = std::make_unique<foxglove::schemas::RawImageChannel>(
        foxglove::schemas::RawImageChannel::create(this->topic()).value());
}

bool ImageHandler::Send(const commsgs::sensor_msgs::Image& msgs)
{
    auto data = FromCommsgs(msgs);
    channel_->log(data);
    return true;
}

bool ImageHandler::Send(const cv::Mat& mat)
{
    channel_->log(FromCommsgs(mat));
    return true;
}

}   // namespace displays 
}   // namespace visualization
}   // namespace autonomy