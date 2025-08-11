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

#include <algorithm>
#include <string>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace transform {

/** 
 * \brief This class provides an easy way to publish coordinate frame transform
 * information.
 * It will handle all the messaging and stuffing of messages.  And the function
 * prototypes lay out all the
 * necessary data needed for each message. 
 */

class TransformBroadcaster 
{
public:
    /** 
     * \brief Send a TransformStamped message
     * The stamped data structure includes frame_id, and time, and parent_id
     * already.  
     */
    void SendTransform(const commsgs::geometry_msgs::TransformStamped& transform);

    /** 
     * \brief Send a vector of TransformStamped messages
     * The stamped data structure includes frame_id, and time, and parent_id
     * already.  
     */
    void SendTransform(const std::vector<commsgs::geometry_msgs::TransformStamped>& transforms);
};

}  // namespace transform
}  // namespace autonomy