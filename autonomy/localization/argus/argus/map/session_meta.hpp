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
 * @file session_meta.hpp
 * @brief Multi-session registry metadata.
 */

#ifndef ARGUS_MAP_SESSION_META_HPP_
#define ARGUS_MAP_SESSION_META_HPP_

#include <string>

#include "argus/common/math/transform/rigid_transform.hpp"

namespace argus {
namespace map {

/** @brief Logical map-format version (bump on breaking layout changes). */
constexpr int kMapFormatVersion = 1;

/** @brief Session id, time span, and world anchor. */
struct SessionMeta {
    std::string session_id;
    double t_start_sec = 0.0;
    double t_end_sec = 0.0;
    /** @brief Session frame in world: \(p_w = R p_s + t\). */
    transform::Rigid3d T_world_session = transform::Rigid3d::Identity();
    double mean_confidence = 1.0;
    /** @brief Pose-graph node id offset for this session's local keyframe ids.
     */
    int node_id_base = 0;
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_SESSION_META_HPP_
