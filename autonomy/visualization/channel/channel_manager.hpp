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

#include <unordered_map>
#include "autonomy/visualization/channel/channel_base.hpp"
#include "autonomy/visualization/channel/channel_traits.hpp"

namespace autonomy {
namespace visualization { 
namespace channel { 

/**
 * Channel manager class for handling topic channels
 */
class ChannelManager 
{
public:

    /**
     * Define ChannelManager::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(ChannelManager)
    
    /**
     * @brief Construct a ChannelManager
     */
    ChannelManager();

    /**
     * @brief Deconstruct a ChannelManager
     */
    ~ChannelManager();

    /**
     * @brief send msgs
     */
    template <typename MsgsType>
    bool Send(const MsgsType& msgs);

    /**
     * @brief Add topic channel
     */
    template <typename MsgsType>
    ChannelBase::SharedPtr Add(const std::string& topic) {
        if (channels_.find(topic) != channels_.end()) {
            return nullptr; 
        }
        channels_[topic] = std::make_shared<typename ChannelTraits<MsgsType>::type>(topic);
        return channels_[topic];
    }

    /**
     * @brief Remove topic channel
     */
    bool Remove(const std::string& topic) {
        auto it = channels_.find(topic);
        if (it != channels_.end()) {
            channels_.erase(it);
            return true;
        }
        return false;
    }

    /**
     * @brief Check topic channel is exist
     */
    template <typename MsgsType>
    bool Contain(const std::string& topic) {
        return channels_.find(topic) != channels_.end();
    }

    /**
     * @brief Find topic channel
     */
     template <typename MsgsType>
    ChannelBase::SharedPtr Find(const std::string& topic){
        auto it = channels_.find(topic);
        if (it != channels_.end()) {
            return it->second;
        }
        return nullptr;
    }

private:

    ///< Collection of topic channels
    std::unordered_map<std::string, ChannelBase::SharedPtr> channels_;
};

}   // namespace channel
}   // namespace visualization
}   // namespace autonomy