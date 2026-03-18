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

// vision_msgs converter module
// Placeholder for future implementation

#include "autonomy/commsgs/proto/vision_msgs.pb.h"
#include "foxglove/schemas.hpp"

namespace autonomy {
namespace visualization {
namespace converter {
namespace impl {

// vision_msgs conversion functions
// Detection messages
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::vision_msgs::Detection2D& message);
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::vision_msgs::Detection2DArray& message);
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::vision_msgs::Detection3D& message);
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::vision_msgs::Detection3DArray& message);

// Bounding box messages
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::vision_msgs::BoundingBox2D& message);
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::vision_msgs::BoundingBox2DArray& message);
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::vision_msgs::BoundingBox3D& message);
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::vision_msgs::BoundingBox3DArray& message);

// Classification messages
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::vision_msgs::Classification& message);

// Object hypothesis messages
foxglove::schemas::SceneUpdate ToFoxgloveImpl(const autonomy::commsgs::proto::vision_msgs::ObjectHypothesis& message);
foxglove::schemas::SceneUpdate ToFoxgloveImpl(
    const autonomy::commsgs::proto::vision_msgs::ObjectHypothesisWithPose& message);

}  // namespace impl
}  // namespace converter
}  // namespace visualization
}  // namespace autonomy
