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
#include <unordered_map>

#include "autonomy/common/macros.hpp"
#include "autonomy/common/thread_pool.hpp"
#include "autonomy/common/eventpp/eventqueue.h"
#include "autonomy/common/eventpp/callbacklist.h"
#include "autonomy/common/blocking_queue.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/tools/god_viewer/server_options.hpp"
#include "autonomy/tools/god_viewer/channel/channel_base.hpp"

namespace autonomy {
namespace tools { 
namespace god_viewer { 

class ViewerBridge
{
public:
    /**
     * Define ViewerBridge::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(ViewerBridge)

    /**
     * @brief A constructor for ViewerBridge
     * @param options Additional options to control creation of the node.
     */
    ViewerBridge(const std::string& configuration_directory, const std::string& configuration_basename);

    /**
     * @brief A Destructor for ViewerBridge
     */
    ~ViewerBridge() = default;

    /**
     * @brief Run the main task for god viewer
     */
    void Run();

    /**
     * @brief Shutdown foxglove server
     */
    void ShutDown();

private:

    /**
     * @brief Init foxglove server
     */
    bool LoadOptions(const std::string& configuration_directory, const std::string& configuration_basename);

    // Foxglove server options
    ServerHander::SharedPtr server_handler_{nullptr};

    // thread_pool
    std::shared_ptr<common::ThreadPool> thread_pool_{nullptr};
    std::unordered_map<std::string, channel::ChannelBase::SharedPtr> channels_handler_;
};

}   // namespace god_viewer
}   // namespace tools
}   // namespace autonomy