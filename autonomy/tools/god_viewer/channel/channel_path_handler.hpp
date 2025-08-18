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
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/tools/god_viewer/channel/channel_base.hpp"

namespace autonomy {
namespace tools { 
namespace god_viewer { 
namespace channel {

class PathHandler : public ChannelBase
{
public:
    /**
     * Define PathHandler::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(PathHandler)

    /**
     * @brief Construct a new Pathhandler object
     * 
     * @param topic 
     */
    PathHandler(ServerHander::SharedPtr options, const std::string& topic);


    bool Send();

    /**
     * @brief send path
     */
    bool Send(const commsgs::planning_msgs::Path& msgs);

    /**
     * @brief demo path
     */
    commsgs::planning_msgs::Path GenerateCircularPath(double radius);

private:

    /**
     * @brief convert msgs form commsgs
     */
    foxglove::schemas::LinePrimitive FromCommsgs(const commsgs::planning_msgs::Path& msgs);

    // channel topic name 
    std::string topic_;
    std::unique_ptr<foxglove::schemas::SceneUpdateChannel> channel_{nullptr};

};

}   // channel
}   // god_viewer
}   // namespace tools
}   // namespace autonomy