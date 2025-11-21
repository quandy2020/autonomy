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

#include <vector>
#include <string>
#include <map>
#include <thread>
#include <atomic>
#include <chrono>
#include <memory>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <type_traits>

#include <foxglove/foxglove.hpp>
#include <foxglove/context.hpp>
#include <foxglove/error.hpp>
#include <foxglove/mcap.hpp>
#include <foxglove/server.hpp>

#include "autonomy/common/macros.hpp"
#include "autonomy/common/helper_functions/crtp.hpp"

namespace autonomy {
namespace visualization { 
namespace channel {

class ChannelBase 
{
public:
    /**
     * Define ChannelBase::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(ChannelBase)

    /**
     * @brief Construct a new ChannelBase object
     * 
     * @param topic Topic name
     * @param publish_frequency Publish topic frequency
     */
    ChannelBase(const std::string& topic) 
        : topic_(topic) {}

    /**
     * @brief A Destructor for ChannelBase
     */
    virtual ~ChannelBase() {}

protected:

    /**
     * @brief Get channel's name topic
     */
    std::string topic() { return topic_; }

    ///< Name or identifier of the topic being published. Used for logging and
    ///< identification purposes within the system.
    std::string topic_;
};

}   // namespace channel
}   // namespace visualization
}   // namespace autonomy

