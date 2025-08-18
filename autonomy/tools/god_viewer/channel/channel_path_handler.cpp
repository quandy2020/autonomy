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

#include "autonomy/tools/god_viewer/channel/channel_path_handler.hpp"

#include <chrono>

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace tools { 
namespace god_viewer { 
namespace channel {

PathHandler::PathHandler(ServerHander::SharedPtr options, const std::string& topic)
    : topic_{topic}
{
    LOG(INFO) << "Init Pathhandler";

    channel_ = std::make_unique<foxglove::schemas::SceneUpdateChannel>(
        foxglove::schemas::SceneUpdateChannel::create(topic_).value());
}

bool PathHandler::Send()
{
    foxglove::schemas::CubePrimitive cube;
    cube.size = foxglove::schemas::Vector3{1, 1, 2};
    cube.color = foxglove::schemas::Color{1, 0, 0, 1};

    foxglove::schemas::SceneEntity entity;
    entity.id = "box";
    entity.cubes.push_back(cube);

    foxglove::schemas::SceneUpdate scene_update;
    scene_update.entities.push_back(entity);

    channel_->log(scene_update);
    return true;
}

bool PathHandler::Send(const commsgs::planning_msgs::Path& msgs)
{
    auto line = FromCommsgs(msgs);
    return true;
}

foxglove::schemas::LinePrimitive PathHandler::FromCommsgs(const commsgs::planning_msgs::Path& msgs)
{
    foxglove::schemas::LinePrimitive line;
    return line;
}

}   // channel
}   // god_viewer
}   // namespace tools
}   // namespace autonomy