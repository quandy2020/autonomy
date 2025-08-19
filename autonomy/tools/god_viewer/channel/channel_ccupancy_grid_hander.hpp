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

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/tools/god_viewer/channel/channel_base.hpp"

namespace autonomy {
namespace tools { 
namespace god_viewer { 
namespace channel {

class OccupancyGridHandler : public ChannelBase
{
public:
    /**
     * Define OccupancyGridHandler::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(OccupancyGridHandler)

    /**
     * @brief Construct a new Pathhandler object
     * 
     * @param topic 
     */
    OccupancyGridHandler(ServerHander::SharedPtr options, const std::string& topic);

    /**
     * @brief Demo show
     */
    bool SendTest();

    /**
     * @brief send path
     */
    bool Send(const commsgs::map_msgs::OccupancyGrid& msgs);

private:

    /**
     * @brief convert msgs form commsgs
     */
    foxglove::schemas::Grid FromCommsgs(const commsgs::map_msgs::OccupancyGrid& msgs);

    // channel topic name 
    std::string topic_;
    std::unique_ptr<foxglove::schemas::GridChannel> channel_{nullptr};

};

}   // channel
}   // god_viewer
}   // namespace tools
}   // namespace autonomy