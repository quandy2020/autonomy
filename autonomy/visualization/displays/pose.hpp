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

#include <memory>
#include <string>

#include <foxglove/foxglove.hpp>
#include <foxglove/context.hpp>
#include <foxglove/error.hpp>
#include <foxglove/mcap.hpp>
#include <foxglove/server.hpp>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/visualization/channel/channel_base.hpp"

namespace autonomy {
namespace visualization {
namespace displays {

class PoseHandler : public channel::ChannelBase
{
public:
   /**
    * Define PoseHandler::SharedPtr type
    */
   AUTONOMY_SMART_PTR_DEFINITIONS(PoseHandler)

   /**
    * @brief Construct a new PoseHandler object
    * 
    * @param topic 
    */
   PoseHandler(const std::string& topic);

   /**
    * @brief Demo show
    */
   bool SendTest();

   /**
    * @brief send path
    */
   bool Send(const commsgs::geometry_msgs::Pose& msgs);

private:
    // PoseChannel 
    std::unique_ptr<foxglove::schemas::PoseChannel> channel_{nullptr};
};

}   // namespace displays 
}   // namespace visualization
}   // namespace autonomy