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

#include "autonomy/visualization/message_handlers.hpp"
#include "autonomy/visualization/constants.hpp"
#include "autonomy/visualization/visualization_server.hpp"
#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace visualization {

MessageHandlers::MessageHandlers(
    VisualizationServer* visualization_server,
    const proto::VisualizationOptions& options)
    : options_(options)
    , visualization_server_(visualization_server)
{
    LOG(INFO) << "MessageHandlers created";
}

bool MessageHandlers::Initialize()
{
    if (!visualization_server_) {
        LOG(ERROR) << "Visualization server is not initialized";
        return false;
    }

    LOG(INFO) << "Initializing message handlers with " 
              << options_.subscriptions_size() << " subscriptions";

    // Create subscriptions for each configured topic
    int success_count = 0;
    for (const auto& sub : options_.subscriptions()) {
        if (SubscribeToTopic(sub.topic_name(), sub.message_type())) {
            success_count++;
            LOG(INFO) << "  [OK] Subscribed to topic: " << sub.topic_name() 
                      << " (Type: " << sub.message_type() << ")";
        } else {
            LOG(ERROR) << "  [FAILED] Failed to subscribe to topic: " << sub.topic_name()
                       << " (Type: " << sub.message_type() << ")";
        }
    }

    LOG(INFO) << "Successfully created " << success_count << "/" 
              << options_.subscriptions_size() << " subscriptions";

    return success_count > 0;
}

bool MessageHandlers::SubscribeToTopic(const std::string& topic_name, const std::string& message_type)
{
    LOG(WARNING) << "SubscribeToTopic disabled (DDS removed): " << topic_name << ", " << message_type;
    return false;
}

// Message callback implementations
void MessageHandlers::HandleMapCallback(const commsgs::map_msgs::OccupancyGrid& msg)
{
    if (visualization_server_) {
        visualization_server_->Publish("map", 
            std::move(std::remove_cv_t<commsgs::map_msgs::OccupancyGrid>(msg)));
    }
}

void MessageHandlers::HandlePathCallback(const commsgs::planning_msgs::Path& msg)
{
    if (visualization_server_) {
        visualization_server_->Publish("path", 
            std::move(std::remove_cv_t<commsgs::planning_msgs::Path>(msg)));
    }
}

void MessageHandlers::HandleImageCallback(const commsgs::sensor_msgs::Image& msg)
{
    if (visualization_server_) {
        visualization_server_->Publish("image", 
            std::move(std::remove_cv_t<commsgs::sensor_msgs::Image>(msg)));
    }
}

void MessageHandlers::HandlePoseCallback(const commsgs::geometry_msgs::Pose& msg)
{
    if (visualization_server_) {
        visualization_server_->Publish("pose", 
            std::move(std::remove_cv_t<commsgs::geometry_msgs::Pose>(msg)));
    }
}

void MessageHandlers::HandlePointCallback(const commsgs::geometry_msgs::Point& msg)
{
    if (visualization_server_) {
        visualization_server_->Publish("point", 
            std::move(std::remove_cv_t<commsgs::geometry_msgs::Point>(msg)));
    }
}

void MessageHandlers::HandlePolygonCallback(const commsgs::geometry_msgs::PolygonStamped& msg)
{
    if (visualization_server_) {
        visualization_server_->Publish("polygon", 
            std::move(std::remove_cv_t<commsgs::geometry_msgs::PolygonStamped>(msg)));
    }
}

}   // namespace visualization
}   // namespace autonomy