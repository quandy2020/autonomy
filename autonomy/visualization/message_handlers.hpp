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

#include <string>
#include <vector>
#include <memory>

#include "autolink/autolink.hpp"

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/sensor_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/visualization/proto/visualization_options.pb.h"

namespace autonomy {
namespace visualization {

class VisualizationServer;

/**
 * @brief MessageHandlers manages all topic subscriptions and forwards messages to visualization
 * 
 * This class dynamically creates subscriptions based on configuration and handles
 * various message types from the commsgs package.
 */
class MessageHandlers 
{
public:

    /**
     * Define MessageHandlers::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(MessageHandlers)

    /**
     * @brief Constructor
     * @param visualization_server Pointer to the visualization server
     * @param node DDS node for creating subscriptions
     * @param options Configuration options containing subscription list
     */
    explicit MessageHandlers(
        VisualizationServer* visualization_server,
        const proto::VisualizationOptions& options);

    /**
     * @brief Destructor
     */
    ~MessageHandlers() = default;

    /**
     * @brief Initialize subscriptions based on configuration
     * @return true if successful, false otherwise
     */
    bool Initialize();

private:

    /**
     * @brief Subscribe to a specific message type based on type string
     * @param topic_name Topic name to subscribe
     * @param message_type Message type string
     * @return true if subscription created successfully
     */
    bool SubscribeToTopic(const std::string& topic_name, const std::string& message_type);

    // Message type specific callbacks
    void HandleMapCallback(const commsgs::map_msgs::OccupancyGrid& msg);
    void HandlePathCallback(const commsgs::planning_msgs::Path& msg);
    void HandleImageCallback(const commsgs::sensor_msgs::Image& msg);
    void HandlePoseCallback(const commsgs::geometry_msgs::Pose& msg);
    void HandlePointCallback(const commsgs::geometry_msgs::Point& msg);
    void HandlePolygonCallback(const commsgs::geometry_msgs::PolygonStamped& msg);

    // Configuration
    proto::VisualizationOptions options_;

    // DDS Node (removed)

    // Visualization server
    VisualizationServer* visualization_server_{nullptr};

    // // Map subscriptions
    //     map_subscriptions_;
    // 
    // // Sensor subscriptions
    //     image_subscriptions_;
    // 
    // // Planning subscriptions
    //     path_subscriptions_;
    // 
    // // Geometry subscriptions
    //     pose_subscriptions_;
    //     point_subscriptions_;
    //     polygon_subscriptions_;
    
    // Topic name storage for callbacks
    std::vector<std::string> topic_names_;
};

}   // namespace visualization
}   // namespace autonomy