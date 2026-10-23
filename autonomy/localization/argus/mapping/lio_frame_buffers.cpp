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

#include "argus/map/lio_frame_buffers.hpp"

namespace argus {
namespace map {

void LioFrameBuffers::EnsureDefaultClouds() {
    if (!feats_undistort) {
        feats_undistort = sensor::PointCloudXYZI::make_shared();
    }
    if (!feats_down_body) {
        feats_down_body = sensor::PointCloudXYZI::make_shared();
    }
    if (!feats_down_world) {
        feats_down_world = sensor::PointCloudXYZI::make_shared();
    }
    if (!world_lidar) {
        world_lidar = sensor::PointCloud<sensor::PointXYZI>::make_shared();
    }
}

}  // namespace map
}  // namespace argus
