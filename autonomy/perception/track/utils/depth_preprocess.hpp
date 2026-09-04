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

/**
 * @file
 * @brief Depth image resize and normalization for the track network.
 */

#pragma once

#include <vector>

#include <automsgs/msgs/sensor_msgs/image.pb.h>

namespace autonomy::perception::track {

/**
 * @brief Resizes a depth image to |output_height|×|output_width| floats in [0, 1].
 *
 * Supported encodings: 16UC1 / mono16 (scaled by |depth_scale_to_metres|),
 * and 32FC1 (already metres; scale ignored).
 *
 * @param max_depth_m Clip and normalize against this range.
 * @return False when arguments are invalid, encoding unsupported, or empty.
 */
bool NormalizeDepthImage(const automsgs::msgs::sensor_msgs::Image& image,
                         int output_height, int output_width,
                         double depth_scale_to_metres, double max_depth_m,
                         std::vector<float>* normalized_depth);

}  // namespace autonomy::perception::track
