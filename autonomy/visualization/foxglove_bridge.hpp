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

#include <unordered_set>

#include "autonomy/visualization/proto/visualization_options.pb.h"

#include "autonomy/common/macros.hpp"
#include "autonomy/visualization/foxglove_bridge.hpp"
#include "autonomy/visualization/msgs_traits.hpp"
#include "autonomy/visualization/common/server_handler.hpp"
#include "autonomy/visualization/channel/channel_manager.hpp"

namespace autonomy {
namespace visualization { 

class FoxgloveBridge
{
public:

    /**
     * Define FoxgloveBridge::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(FoxgloveBridge)

    /**
     * @brief A constructor for FoxgloveBridge
     */
    FoxgloveBridge(const std::string& host = "0.0.0.0", const uint32_t& port = 8765, 
        const std::string& mcap_filename = "", bool write_mcap_data = false);

    /**
     * @brief Shutdown 
     */
    void WaitForShutdown();

    /**
     * @brief Publish commsgs msgs
     */
    template <typename M>
    void Publish(const std::string& topic, M&& msgs)
    {
        static_assert(is_pubslish_message<M>::value, "Publish msgs type not support.");
        if (channel_manager_->Contain<M>(topic)) {
            auto channel = channel_manager_->Find<M>(topic);
            if (channel == nullptr) {
                LOG(ERROR) << "Get channel error, check channel is nullptr ?";
               return;
            } else {
                auto sent = std::dynamic_pointer_cast<typename 
                    channel::ChannelTraits<M>::type>(channel)->Send(std::forward<M>(msgs));
                if (!sent) {
                    LOG(ERROR) << "Publish topic: " << topic << "error, msgs type("
                               <<  message_traits<M>::name << ").";
                }
            }
        } else {
            bool sent = std::dynamic_pointer_cast<typename channel::ChannelTraits<M>::type>(
                channel_manager_->Add<M>(topic))->Send(std::forward<M>(msgs));
            if (!sent) {
                LOG(ERROR) << "Create & Publish topic:(" << topic << ") error, msgs type("
                           <<  message_traits<M>::name << ").";
            }
        }
    }

private:

    // Foxglove server options
    ServerHandler::UniquePtr server_handler_{nullptr};

    // channel manager
    channel::ChannelManager::UniquePtr channel_manager_{nullptr};
};

}   // namespace visualization
}   // namespace autonomy