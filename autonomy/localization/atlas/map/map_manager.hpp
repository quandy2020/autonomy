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
 * @file map_manager.hpp
 * @brief Lightweight map manager: Keyframe / Landmark lists for frontend handoff.
 *
 * Unlike ORB-SLAM3 `Map`/`Atlas`, this class targets a simplified VO/frontend
 * pipeline and uses value types `Keyframe` / `Landmark` from
 * `common/types.hpp`, not the `KeyFrame` / `MapPoint` graph.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MAP_MAP_MANAGER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MAP_MAP_MANAGER_HPP_

#include <mutex>
#include <vector>

#include "autonomy/localization/atlas/common/config.hpp"
#include "autonomy/localization/atlas/common/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

/**
 * @class autonomy::localization::atlas::MapManager
 * @brief Thread-safe keyframe and landmark cache for frontend / fusion modules.
 *
 * @note No direct ORB-SLAM3 counterpart; for full graph optimization use
 * `Map` / `MultiMap` / `LocalMapping`.
 *
 * @code{.cpp}
 * MapManager mgr;
 * mgr.Init(config);
 * mgr.AddKeyframe(kf);
 * mgr.UpdateFromOdometry(odom);
 * @endcode
 */
class MapManager {
public:
    /**
     * @brief Initialize from config and `Reset`.
     * @param config Atlas runtime config.
     * @return Currently always `true`.
     */
    bool Init(const AtlasConfig& config);
    /** @brief Clear keyframes and landmarks; reset ID counter. */
    void Reset();

    /**
     * @brief Append a keyframe (may auto-assign ID).
     * @param keyframe Frontend keyframe value.
     */
    void AddKeyframe(const Keyframe& keyframe);
    /**
     * @brief Update the latest keyframe pose etc. from odometry.
     * @param odometry Frontend odometry output.
     */
    void UpdateFromOdometry(const OdometryResult& odometry);

    /**
     * @brief Stored keyframe list.
     * @return Const reference.
     */
    const std::vector<Keyframe>& Keyframes() const { return keyframes_; }
    /**
     * @brief Stored landmark list.
     * @return Const reference.
     */
    const std::vector<Landmark>& Landmarks() const { return landmarks_; }

private:
    AtlasConfig config_;                 ///< Config copy
    mutable std::mutex mutex_;           ///< List lock
    std::vector<Keyframe> keyframes_;    ///< Keyframe cache
    std::vector<Landmark> landmarks_;    ///< Landmark cache
    int next_keyframe_id_ = 0;           ///< Next keyframe ID
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MAP_MAP_MANAGER_HPP_
