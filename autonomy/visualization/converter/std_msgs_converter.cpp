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

#include "autonomy/visualization/converter/std_msgs_converter.hpp"

#include "autolink/autolink.hpp"
#include "autonomy/visualization/converter/converter_detail.hpp"

namespace autonomy {
namespace visualization {
namespace converter {
namespace impl {

using detail::SetEntityHeader;

// std_msgs conversion implementations
// Note: Most std_msgs types don't require standalone visualization conversion:
//
// - Header: Used in ExtractTimestamp() and ExtractFrameId() helper functions
// - ColorRGBA: Used in CreateColor() helper function
// - MultiArray types: Deprecated as of ROS Foxy
// - String: Has no spatial information for visualization
//
// If conversion functions are needed in the future, they can be added here.
//
// Example implementation for String (if text visualization is needed):
//
// foxglove::schemas::SceneUpdate ToFoxgloveImpl(
//     const autonomy::commsgs::proto::std_msgs::String& message) {
//     foxglove::schemas::SceneUpdate scene_update;
//     // String messages typically don't have spatial information
//     // Text visualization would require additional context (position, etc.)
//     AWARN << "String messages have no spatial information, "
//               << "returning empty SceneUpdate";
//     return scene_update;
// }

}  // namespace impl
}  // namespace converter
}  // namespace visualization
}  // namespace autonomy
