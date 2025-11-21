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

#include "autonomy/visualization/visualization_server.hpp"

#include <thread>
#include <chrono>

#include "autonomy/common/logging.hpp"
#include "autonomy/visualization/constants.hpp"

namespace autonomy {
namespace visualization {

VisualizationServer::VisualizationServer(const proto::VisualizationOptions& options)
    : options_{options}
{
    LOG(INFO) << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━";
    LOG(INFO) << "  Initializing Visualization Server";
    LOG(INFO) << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━";
    
    // LOG(INFO) << "Creating DDS node: " << kNodeName;
    // if (!node_) {
    //     LOG(ERROR) << "Failed to create DDS node";
    //     return;
    // }
    // LOG(INFO) << "  [OK] DDS node created";

    // Create foxglove bridge
    LOG(INFO) << "Creating Foxglove Bridge";
    LOG(INFO) << "  Host: " << options_.host();
    LOG(INFO) << "  Port: " << options_.port();
    LOG(INFO) << "  MCAP Recording: " << (options_.write_mcap_data() ? "Enabled" : "Disabled");
    if (options_.write_mcap_data()) {
        LOG(INFO) << "  MCAP File: " << options_.mcap_filename();
    }
    
    foxglove_bridge_ = std::make_shared<FoxgloveBridge>(
        options_.host(), 
        options_.port(), 
        options_.mcap_filename(), 
        options_.write_mcap_data()
    );
    if (!foxglove_bridge_) {
        LOG(ERROR) << "Failed to create Foxglove bridge";
        return;
    }
    LOG(INFO) << "  [OK] Foxglove bridge created";

    // Create message handlers with options
    LOG(INFO) << "Creating message handlers";
    message_handlers_ = std::make_shared<MessageHandlers>(this, options_);
    if (!message_handlers_) {
        LOG(ERROR) << "Failed to create message handlers";
        return;
    }
    
    // Initialize message handlers (create subscriptions)
    if (!message_handlers_->Initialize()) {
        LOG(WARNING) << "Message handlers initialization had some failures";
    } else {
        LOG(INFO) << "  [OK] Message handlers initialized";
    }
    
    LOG(INFO) << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━";
    LOG(INFO) << "  Visualization Server Initialized Successfully";
    LOG(INFO) << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━";
}

void VisualizationServer::Start()
{
    if (!foxglove_bridge_ || !message_handlers_) {
        LOG(ERROR) << "Visualization server not properly initialized, cannot start";
        return;
    }
    
    LOG(INFO) << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━";
    LOG(INFO) << "  Visualization Server Started";
    LOG(INFO) << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━";
    LOG(INFO) << "";
    LOG(INFO) << "Visualization server is now running and ready to receive messages.";
    LOG(INFO) << "Connect to Foxglove Studio at: ws://" << options_.host() << ":" << options_.port();
    LOG(INFO) << "";
    LOG(INFO) << "Subscribed Topics:";
    for (int i = 0; i < options_.subscriptions_size(); ++i) {
        const auto& sub = options_.subscriptions(i);
        LOG(INFO) << "  [" << (i + 1) << "] " << sub.topic_name() 
                  << " (" << sub.message_type() << ")";
    }
    LOG(INFO) << "";
    LOG(INFO) << "Press Ctrl+C to stop...";
    LOG(INFO) << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━";
    
    // Keep the server running
    // The actual message processing happens in the DDS callbacks
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void VisualizationServer::WaitForShutdown()
{
    if (foxglove_bridge_) {
        LOG(INFO) << "Shutting down Foxglove bridge...";
        foxglove_bridge_->WaitForShutdown();
    }
    
    LOG(INFO) << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━";
    LOG(INFO) << "  Visualization Server Stopped";
    LOG(INFO) << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━";
}

}   // namespace visualization
}   // namespace autonomy