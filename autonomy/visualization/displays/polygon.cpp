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
#include "autonomy/visualization/displays/polygon.hpp"
#include "autonomy/visualization/msgs_converter.hpp"

namespace autonomy {
namespace visualization {
namespace displays {

PolygonHandler::PolygonHandler(const std::string& topic)
    : channel::ChannelBase(topic)
{
    LOG(INFO) << "Init handler topic: " << this->topic();
    channel_ = std::make_unique<foxglove::schemas::SceneUpdateChannel>(
        foxglove::schemas::SceneUpdateChannel::create(this->topic()).value());
}

bool PolygonHandler::SendTest()
{
    commsgs::geometry_msgs::PolygonStamped msgs;
    msgs.header.frame_id = "map";
    msgs.header.stamp = Time::Now();

    msgs.polygon.points.push_back({
        0.0,
        0.0,
        0.0,
    });
    msgs.polygon.points.push_back({
        1.0,
        0.0,
        0.0,
    });

     msgs.polygon.points.push_back({
        1.0,
        1.0,
        0.0,
    });

     msgs.polygon.points.push_back({
        0.0,
        1.0,
        0.0,
    });
    return Send(msgs);
}

bool PolygonHandler::Send(const commsgs::geometry_msgs::PolygonStamped& msgs)
{
    foxglove::schemas::SceneEntity entity;
    entity.frame_id = msgs.header.frame_id;
    entity.id = "line";
    entity.lines.push_back(FromCommsgs(msgs));
    foxglove::schemas::SceneUpdate scene_update;
    scene_update.entities.push_back(entity);
    if (channel_->log(scene_update) != foxglove::FoxgloveError::Ok) {
        LOG(ERROR) << "Send commsgs::geometry_msgs::PolygonStamped msgs error.";
        return false;
    }
    return true;
}

}   // namespace displays 
}   // namespace visualization
}   // namespace autonomy