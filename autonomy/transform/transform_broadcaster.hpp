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
#include <string>
#include <vector>
#include <mutex>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace transform {

/** 
 * @brief TransformBroadcaster provides an easy way to publish coordinate frame 
 * transform information.
 * 
 * This class handles all the messaging and stuffing of messages. The function
 * prototypes lay out all the necessary data needed for each message.
 * 
 * Usage example:
 * @code
 *   auto node = std::make_shared<Node>("my_node");
 *   TransformBroadcaster broadcaster(node);
 *   
 *   // Create a transform
 *   commsgs::geometry_msgs::TransformStamped transform;
 *   transform.header.frame_id = "world";
 *   transform.child_frame_id = "base_link";
 *   transform.header.stamp = Time::Now();
 *   transform.transform.translation.x = 1.0;
 *   transform.transform.rotation.w = 1.0;
 *   
 *   // Send the transform
 *   broadcaster.SendTransform(transform);
 * @endcode
 */
class TransformBroadcaster 
{
public:
    /**
     * Define TransformBroadcaster::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(TransformBroadcaster)
    
    /**
     * @brief Constructor 
     * 
     * @param topic_name TF message topic name, defaults to "/tf"
     */
    explicit TransformBroadcaster(
        const std::string& topic_name = "/tf");
    
    /**
     * @brief Destructor
     */
    ~TransformBroadcaster();
    
    /** 
     * @brief Send a single TransformStamped message
     * 
     * @param transform The transform to send, includes frame_id, timestamp, and child_frame_id
     * 
     * Usage example:
     * @code
     *   TransformStamped tf;
     *   tf.header.frame_id = "map";
     *   tf.child_frame_id = "odom";
     *   tf.header.stamp = Time::Now();
     *   broadcaster.SendTransform(tf);
     * @endcode
     */
    void SendTransform(const commsgs::geometry_msgs::TransformStamped& transform);

    /** 
     * @brief Send multiple TransformStamped messages
     * 
     * @param transforms Vector of transforms to send
     * 
     * This method is more efficient than calling SendTransform multiple times,
     * as it sends all transforms in a single batch.
     * 
     * Usage example:
     * @code
     *   std::vector<TransformStamped> tfs;
     *   // ... populate transforms ...
     *   broadcaster.SendTransform(tfs);
     * @endcode
     */
    void SendTransform(const std::vector<commsgs::geometry_msgs::TransformStamped>& transforms);
    
    /**
     * @brief Get the topic name for publishing
     * 
     * @return The current topic name being used
     */
    std::string GetTopicName() const;
    
    /**
     * @brief Get the total number of transforms sent
     * 
     * @return Total number of transform messages sent since creation
     */
    size_t GetPublishedCount() const;
    
    /**
     * @brief Reset the published message counter
     */
    void ResetPublishedCount();
    
    /**
     * @brief Check if the broadcaster is initialized
     * 
     * @return true if properly initialized, false otherwise
     */
    bool IsInitialized() const;

private:
    /**
     * @brief Internal publish implementation
     * 
     * @param transforms List of transforms to publish
     */
    void PublishTransforms(const std::vector<commsgs::geometry_msgs::TransformStamped>& transforms);

    std::string topic_name_;                  ///< TF message topic name
    mutable std::mutex mutex_;                ///< Thread safety mutex
    size_t published_count_;                  ///< Published message counter
    bool initialized_;                        ///< Initialization flag
};

} // namespace transform
} // namespace autonomy
