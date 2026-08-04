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

#include "autonomy/common/macros.hpp"
#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>

namespace autonomy {
namespace transform {
class TransformBroadcaster
{
public:
    /**
     * Define TransformBroadcaster::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(TransformBroadcaster)

    /**
     * @brief Constructor
     */
    explicit TransformBroadcaster();

    /**
     * @brief Send a TransformStamped message
     *     The stamped data structure includes frame_id, and time, and parent_id
     * already.
     * @param transform The transform to send
     */
    void SendTransform(
        const automsgs::msgs::geometry_msgs::TransformStamped& transform);

    /**
     * @brief Send a vector of TransformStamped messages
     *       The stamped data structure includes frame_id, and time, and
     * parent_id already.
     * @param transforms The transforms to send
     */
    void SendTransform(
        const std::vector<automsgs::msgs::geometry_msgs::TransformStamped>&
            transforms);

    const automsgs::msgs::geometry_msgs::TransformStampeds& GetTransformStampeds()
        const {
        return transform_stampeds_;
    }

private:
    automsgs::msgs::geometry_msgs::TransformStampeds transform_stampeds_;
};

}  // namespace transform
}  // namespace autonomy
