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

// std_msgs converter module
// Note: Most std_msgs types (Header, ColorRGBA) are typically used as part of
// other messages and don't require standalone conversion. MultiArray types are
// deprecated. String messages have no spatial information for visualization.

#include "autonomy/commsgs/proto/std_msgs.pb.h"
#include "foxglove/schemas.hpp"

namespace autonomy {
namespace visualization {
namespace converter {
namespace impl {

// std_msgs conversion functions
// Note: Header and ColorRGBA are typically embedded in other messages and
// handled by ExtractTimestamp, ExtractFrameId, and CreateColor helpers.
// MultiArray types are deprecated as of ROS Foxy.
// String messages have no spatial information for visualization.

// If needed, conversion functions can be added here, for example:
// foxglove::schemas::SceneUpdate ToFoxgloveImpl(
//     const autonomy::commsgs::proto::std_msgs::String& message);

}  // namespace impl
}  // namespace converter
}  // namespace visualization
}  // namespace autonomy
