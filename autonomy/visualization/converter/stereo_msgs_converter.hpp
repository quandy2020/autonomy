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

// stereo_msgs converter module
// Note: Currently stereo_msgs.proto is empty, so no conversion functions are
// defined. When DisparityImage and other stereo_msgs messages are added to the
// proto file, corresponding conversion functions should be added here.

#include "autonomy/commsgs/proto/stereo_msgs.pb.h"
#include "foxglove/schemas.hpp"

namespace autonomy {
namespace visualization {
namespace converter {
namespace impl {

// stereo_msgs conversion functions will be implemented here when message types
// are added Example: DisparityImage conversion foxglove::schemas::RawImage
// ToFoxgloveImpl(
//     const autonomy::commsgs::proto::stereo_msgs::DisparityImage& message);

}  // namespace impl
}  // namespace converter
}  // namespace visualization
}  // namespace autonomy
