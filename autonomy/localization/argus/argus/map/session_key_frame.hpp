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
 * @file session_key_frame.hpp
 * @brief Session-local keyframe pose sample.
 */

#ifndef ARGUS_MAP_SESSION_KEY_FRAME_HPP_
#define ARGUS_MAP_SESSION_KEY_FRAME_HPP_

#include "argus/common/math/transform/rigid_transform.hpp"

namespace argus {
namespace map {

/** @brief One keyframe in a session trajectory. */
struct SessionKeyFrame {
    int local_id = 0;
    double stamp_sec = 0.0;
    /** @brief Body pose in the session frame. */
    transform::Rigid3d T_session_b = transform::Rigid3d::Identity();
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_SESSION_KEY_FRAME_HPP_
