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

#include "autonomy/transform/transform_broadcaster.hpp"

#include <glog/logging.h>

namespace autonomy {
namespace transform {

TransformBroadcaster::TransformBroadcaster(
    const std::string& topic_name)
    : topic_name_(topic_name)
    , published_count_(0)
    , initialized_(false)
{
    LOG(INFO) << "TransformBroadcaster created with topic: " << topic_name_;
}

TransformBroadcaster::~TransformBroadcaster()
{
    LOG(INFO) << "TransformBroadcaster destroyed. Published " 
              << published_count_ << " transform messages.";
}

void TransformBroadcaster::SendTransform(
    const commsgs::geometry_msgs::TransformStamped& transform) 
{
    std::vector<commsgs::geometry_msgs::TransformStamped> transforms;
    transforms.push_back(transform);
    SendTransform(transforms);
}

void TransformBroadcaster::SendTransform(
    const std::vector<commsgs::geometry_msgs::TransformStamped>& transforms) 
{
}

std::string TransformBroadcaster::GetTopicName() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return topic_name_;
}

size_t TransformBroadcaster::GetPublishedCount() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return published_count_;
}

void TransformBroadcaster::ResetPublishedCount()
{
    std::lock_guard<std::mutex> lock(mutex_);
    published_count_ = 0;
    LOG(INFO) << "Published count reset to 0";
}

bool TransformBroadcaster::IsInitialized() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return initialized_;
}

void TransformBroadcaster::PublishTransforms(
    const std::vector<commsgs::geometry_msgs::TransformStamped>& transforms)
{
}

} // namespace transform
} // namespace autonomy
