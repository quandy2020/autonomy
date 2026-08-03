/*
 * Copyright 2026 The Openbot Authors
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

#include "autolink/node/node.hpp"
#include "autonomy/common/macros.hpp"
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>

namespace autonomy {
namespace task {

/** Feeds Autolink TF topics into the process-local transform::Buffer. */
class AutolinkTfListener
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(AutolinkTfListener)

    AutolinkTfListener() = default;
    ~AutolinkTfListener();

    /**
     * @param tf_message_topic e.g. /tf (fakedata, Foxglove convention)
     * @param transform_stampeds_topic e.g. tf (TransformStampeds publishers)
     */
    bool Start(const std::shared_ptr<autolink::Node>& node,
               const std::string& tf_message_topic = "/tf",
               const std::string& transform_stampeds_topic = "tf");

    void Stop();

private:
    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<autolink::Reader<::automsgs::msgs::tf2_msgs::TFMessage>>
        tf_message_reader_;
    std::shared_ptr<autolink::Reader<automsgs::msgs::geometry_msgs::TransformStampeds>>
        stampeds_reader_;
};

}  // namespace task
}  // namespace autonomy
