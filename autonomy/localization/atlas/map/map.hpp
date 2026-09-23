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
 *
 * Structure adapted from ORB-SLAM3 Map.
 */

/**
 * @file map.hpp
 * @brief Single-map container Map: keyframe / map-point sets and IMU init flags.
 *
 * Corresponds to ORB-SLAM3 `Map`. `MultiMap` (Atlas) holds multiple instances;
 * the active map is read/written by Tracking / LocalMapping / LoopClosing.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MAP_MAP_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MAP_MAP_HPP_

#include <memory>
#include <mutex>
#include <set>
#include <vector>

#include "autonomy/localization/atlas/map/keyframe.hpp"
#include "autonomy/localization/atlas/map/map_point.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

/**
 * @class autonomy::localization::atlas::Map
 * @brief One SLAM map: KeyFrame / MapPoint sets and inertial BA stage flags.
 *
 * Corresponds to ORB-SLAM3 `Map`. Set ops are guarded by `mutex_map_`; pose
 * scale alignment uses `ApplyScaledRotation` (IMU init gravity alignment).
 *
 * @note Basic usage
 * @code{.cpp}
 * Map map;
 * map.AddKeyFrame(kf);
 * map.AddMapPoint(mp);
 * map.SetImuInitialized(true);
 * @endcode
 */
class Map {
public:
    /** @brief Default construct; assign a global `id`. */
    Map();
    /**
     * @brief Construct with an init keyframe ID hint (kept for ORB / MultiMap API).
     * @param init_keyframe_id Mapping-start keyframe ID (interface alignment only).
     */
    explicit Map(long unsigned int init_keyframe_id);

    /**
     * @brief Insert a keyframe.
     * @param keyframe Keyframe.
     */
    void AddKeyFrame(const std::shared_ptr<KeyFrame>& keyframe);
    /**
     * @brief Insert a map point.
     * @param map_point Map point.
     */
    void AddMapPoint(const std::shared_ptr<MapPoint>& map_point);
    /**
     * @brief Remove a map point from the set (does not call `SetBadFlag`).
     * @param map_point Map point.
     */
    void EraseMapPoint(const std::shared_ptr<MapPoint>& map_point);
    /**
     * @brief Remove a keyframe from the set.
     * @param keyframe Keyframe.
     */
    void EraseKeyFrame(const std::shared_ptr<KeyFrame>& keyframe);

    /**
     * @brief Set reference map points (visualization / local-map reference).
     * @param map_points Reference point list.
     */
    void SetReferenceMapPoints(
        const std::vector<std::shared_ptr<MapPoint>>& map_points);

    /**
     * @brief Snapshot of all keyframes.
     * @return Keyframe vector.
     */
    std::vector<std::shared_ptr<KeyFrame>> GetAllKeyFrames()
        const;
    /**
     * @brief Snapshot of all map points.
     * @return Map-point vector.
     */
    std::vector<std::shared_ptr<MapPoint>> GetAllMapPoints()
        const;
    /**
     * @brief Number of map points.
     * @return Count.
     */
    long unsigned int MapPointsInMap() const;
    /**
     * @brief Number of keyframes.
     * @return Count.
     */
    long unsigned int KeyFramesInMap() const;

    /**
     * @brief Apply gravity-alignment rotation and scale to all KFs/MPs (IMU init).
     * @param Twg Gravity-alignment transform.
     * @param scale Scale factor.
     * @param update_velocity Whether to scale velocities as well.
     *
     * Corresponds to ORB-SLAM3 `Map::ApplyScaledRotation`.
     */
    void ApplyScaledRotation(const SE3& Twg, double scale,
                             bool update_velocity = true);

    /**
     * @brief Set whether IMU is initialized.
     * @param initialized Flag.
     */
    void SetImuInitialized(bool initialized) { imu_initialized_ = initialized; }
    /**
     * @brief Whether IMU is initialized.
     * @return Flag.
     */
    bool isImuInitialized() const { return imu_initialized_; }

    /**
     * @brief Mark inertial BA stage 1 done (ORB `mbImuBA1`).
     * @param v Flag, default `true`.
     */
    void SetInertialBA1(bool v = true) { inertial_ba1_ = v; }
    /**
     * @brief Mark inertial BA stage 2 done (ORB `mbImuBA2`, VIBA).
     * @param v Flag, default `true`.
     */
    void SetInertialBA2(bool v = true) { inertial_ba2_ = v; }
    /**
     * @brief Whether inertial BA1 is done.
     * @return Flag.
     */
    bool GetInertialBA1() const { return inertial_ba1_; }
    /**
     * @brief Whether inertial BA2 is done.
     * @return Flag.
     */
    bool GetInertialBA2() const { return inertial_ba2_; }

    /** @brief Bump the big-change counter (loop, merge, or BA). */
    void IncreaseChangeIndex();
    /**
     * @brief Big-change counter (ORB `GetLastBigChangeIdx`).
     * @return Monotonic index.
     */
    int GetLastBigChangeIdx() const;

    /** @brief Clear keyframe/map-point sets and reset flags. */
    void clear();

    long unsigned int id = 0;              ///< Global map ID
    static long unsigned int next_id;      ///< Next available map ID

    std::vector<std::shared_ptr<KeyFrame>> keyframe_origins;  ///< Mapping origin KFs

private:
    mutable std::mutex mutex_map_;         ///< Set lock
    std::set<std::shared_ptr<KeyFrame>> keyframes_;   ///< Keyframe set
    std::set<std::shared_ptr<MapPoint>> map_points_;  ///< Map-point set
    std::vector<std::shared_ptr<MapPoint>> reference_map_points_;  ///< Reference points
    long unsigned int max_keyframe_id_ = 0;  ///< Max keyframe ID seen
    bool imu_initialized_ = false;         ///< IMU initialization done
    bool inertial_ba1_ = false;            ///< VIBA stage 1
    bool inertial_ba2_ = false;            ///< VIBA stage 2
    int big_change_index_ = 0;             ///< Loop / BA change counter
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MAP_MAP_HPP_
