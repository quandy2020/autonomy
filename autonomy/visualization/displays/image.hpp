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

#include <vector>
#include <string>
#include <map>
#include <thread>
#include <atomic>
#include <chrono>
#include <memory>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <type_traits>

#include <foxglove/foxglove.hpp>
#include <foxglove/context.hpp>
#include <foxglove/error.hpp>
#include <foxglove/mcap.hpp>
#include <foxglove/server.hpp>

#include <opencv2/opencv.hpp>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/visualization/channel/channel_base.hpp"
 
namespace autonomy {
namespace visualization {
namespace displays {

class ImageHandler : public channel::ChannelBase
{
public:
   /**
    * Define ImageHandler::SharedPtr type
    */
   AUTONOMY_SMART_PTR_DEFINITIONS(ImageHandler)

   /**
    * @brief Construct a new ImageHandler object
    * 
    * @param topic 
    */
   ImageHandler(const std::string& topic);

   /**
    * @brief send path
    */
   bool Send(const commsgs::sensor_msgs::Image& msgs);

   /**
    * @brief
    */
   bool Send(const cv::Mat& msgs);

private:
   // RawImageChannel
   std::unique_ptr<::foxglove::schemas::RawImageChannel> channel_{nullptr};
};

}   // namespace displays 
}   // namespace visualization
}   // namespace autonomy