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

#include "autonomy/visualization/channel/channel_manager.hpp"
#include "autonomy/common/logging.hpp"


namespace autonomy {
namespace visualization { 
namespace channel { 
namespace {

// void InitializeFactory() 
// {
//     auto& factory = ChannelFactoryAccessor<ChannelBase>::GetInstance();
    
//     // Register all topic types
//     // REGISTER_TOPIC_TYPE(factory, channel::PathTopic);
//     // REGISTER_TOPIC_TYPE(factory, Map2DTopic);
//     // REGISTER_TOPIC_TYPE(factory, Costmap2DTopic);
//     // REGISTER_TOPIC_TYPE(factory, IMUTopic);
//     // REGISTER_TOPIC_TYPE(factory, Lidar2DTopic);
    
//     // 或者使用自定义类型名称注册
//     // REGISTER_TOPIC_TYPE_WITH_NAME(factory, IMUTopic, "CustomIMU");
// }

} // namespace

ChannelManager::ChannelManager() 
{

}

ChannelManager::~ChannelManager()
{

}

// void ChannelManager::LoadConfig(const std::vector<TopicConfig>& configs)
// {
//     auto& factory = ChannelFactoryAccessor<channel::ChannelBase>::GetInstance();
//     for (const auto& config : configs) {
//         auto topic = factory.Create(config.type, config.name, config.publish_frequency);
        
//         if (topic) {
//             topics_.push_back(std::move(topic));
//         } else {
//             LOG(ERROR) << "Failed to create topic: " << config.name 
//                        << " with type: " << config.type;
//         }
//     }
// }


}   // namespace channel
}   // namespace visualization
}   // namespace autonomy