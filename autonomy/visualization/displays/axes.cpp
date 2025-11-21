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
#include "autonomy/visualization/displays/axes.hpp"
#include "autonomy/visualization/msgs_converter.hpp"

namespace autonomy {
namespace visualization {
namespace displays {

AxesHandler::AxesHandler(const std::string& topic)
    : channel::ChannelBase(topic)
{
    LOG(INFO) << "Init handler topic: " << this->topic();
    channel_ = std::make_unique<foxglove::schemas::SceneUpdateChannel>(
        foxglove::schemas::SceneUpdateChannel::create(this->topic()).value());
}

bool AxesHandler::SendTest()
{
    commsgs::planning_msgs::Path msgs;
    msgs.header.frame_id = "map";

    const float CENTER_X = 0.5f;
    const float CENTER_Y = 0.0f;
    const float RADIUS = 2.8f;
    const int NUM_POINTS = 100;
    for (int i = 0; i < NUM_POINTS; ++i) {

        commsgs::geometry_msgs::PoseStamped pose;
        float angle = 2.0f * M_PI * i / NUM_POINTS;
        pose.pose.position.x = CENTER_X + RADIUS * cos(angle);
        pose.pose.position.y = CENTER_Y + RADIUS * sin(angle);
        pose.pose.position.z = 0.0;
        msgs.poses.push_back(pose);
    }
    return Send(msgs);
}

bool AxesHandler::Send(const commsgs::planning_msgs::Path& msgs)
{
    foxglove::schemas::SceneEntity entity;
    entity.frame_id = msgs.header.frame_id;
    entity.id = "line";
    entity.lines.push_back(FromCommsgs(msgs));
    foxglove::schemas::SceneUpdate scene_update;
    scene_update.entities.push_back(entity);
    if (channel_->log(scene_update) != foxglove::FoxgloveError::Ok) {
        LOG(ERROR) << "Send commsgs::geometry_msgs::Point msgs error.";
        return false;
    }
    return true;
}

}   // namespace displays 
}   // namespace visualization
}   // namespace autonomy