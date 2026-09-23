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
 * Multi-map manager (ORB-SLAM3 Atlas counterpart).
 */

/**
 * @file multi_map.hpp
 * @brief Multi-map manager MultiMap (ORB-SLAM3 Atlas counterpart).
 *
 * Holds multiple `Map` instances; default read/write go to the current active
 * map. Supports create, switch, mark-bad, and cleanup.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MAP_MULTI_MAP_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MAP_MULTI_MAP_HPP_

#include <memory>
#include <mutex>
#include <set>
#include <vector>

#include "autonomy/localization/atlas/map/keyframe.hpp"
#include "autonomy/localization/atlas/map/map.hpp"
#include "autonomy/localization/atlas/map/map_point.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

/**
 * @class autonomy::localization::atlas::MultiMap
 * @brief Multi-map container: current map + historical set (ORB-SLAM3 `Atlas`).
 *
 * Used for rebuild after tracking failure and multi-session scenarios before
 * map merge. `AddKeyFrame` / `AddMapPoint` etc. forward to `GetCurrentMap()`
 * by default.
 *
 * @note Switch active map
 * @code{.cpp}
 * MultiMap atlas;
 * atlas.CreateNewMap();
 * atlas.AddKeyFrame(kf);
 * atlas.ChangeMap(other_map);
 * @endcode
 */
class MultiMap {
public:
    /** @brief Construct and immediately `CreateNewMap()`. */
    MultiMap();
    /**
     * @brief Construct with an initial keyframe ID.
     * @param init_keyframe_id Stored as `last_init_keyframe_id_`.
     */
    explicit MultiMap(long unsigned int init_keyframe_id);
    /** @brief Default destructor. */
    ~MultiMap() = default;

    /**
     * @brief Create a new map and make it the current active map.
     *
     * Corresponds to ORB-SLAM3 `Atlas::CreateNewMap`.
     */
    void CreateNewMap();
    /**
     * @brief Switch the current active map.
     * @param map Target map (must already be in the set or about to be added).
     */
    void ChangeMap(const std::shared_ptr<Map>& map);

    /**
     * @brief Add a keyframe to the current map.
     * @param keyframe Keyframe.
     */
    void AddKeyFrame(const std::shared_ptr<KeyFrame>& keyframe);
    /**
     * @brief Add a map point to the current map.
     * @param map_point Map point.
     */
    void AddMapPoint(const std::shared_ptr<MapPoint>& map_point);

    /**
     * @brief Set reference map points on the current map.
     * @param map_points Reference point list.
     */
    void SetReferenceMapPoints(
        const std::vector<std::shared_ptr<MapPoint>>& map_points);

    /**
     * @brief Get the current active map (lazy-create if needed).
     * @return Shared pointer to the current `Map`.
     */
    std::shared_ptr<Map> GetCurrentMap();
    /**
     * @brief Mutable raw pointer to the current map (ORB `GetCurrentMap` style).
     * @return `Map*`; may be null if none.
     */
    Map* mutable_current_map();
    /**
     * @brief Snapshot of all active maps.
     * @return Map vector.
     */
    std::vector<std::shared_ptr<Map>> GetAllMaps() const;
    /**
     * @brief Number of active maps.
     * @return Count.
     */
    int CountMaps() const;

    /**
     * @brief All keyframes of the current map.
     * @return Keyframe vector.
     */
    std::vector<std::shared_ptr<KeyFrame>> GetAllKeyFrames()
        const;
    /**
     * @brief All map points of the current map.
     * @return Map-point vector.
     */
    std::vector<std::shared_ptr<MapPoint>> GetAllMapPoints()
        const;
    /**
     * @brief Point count of the current map.
     * @return Count.
     */
    long unsigned int MapPointsInMap() const;
    /**
     * @brief Keyframe count of the current map.
     * @return Count.
     */
    long unsigned int KeyFramesInMap() const;

    /**
     * @brief Move a map into the bad-map set.
     * @param map Bad map.
     */
    void SetMapBad(const std::shared_ptr<Map>& map);
    /** @brief Physically remove maps in the bad-map set. */
    void RemoveBadMaps();
    /** @brief Clear current map contents (keep MultiMap structure). */
    void clearMap();
    /** @brief Clear all maps and the bad-map set. */
    void clearAtlas();

    /**
     * @brief Initial keyframe ID used in the most recent mapping start.
     * @return ID.
     */
    long unsigned int GetLastInitKeyFrameId() const {
        return last_init_keyframe_id_;
    }

private:
    mutable std::mutex mutex_;                 ///< Set / current-map lock
    std::set<std::shared_ptr<Map>> maps_;      ///< Active maps
    std::set<std::shared_ptr<Map>> bad_maps_;  ///< Bad maps pending delete
    std::shared_ptr<Map> current_map_;         ///< Current active map
    long unsigned int last_init_keyframe_id_ = 0;  ///< Last init KF ID
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MAP_MULTI_MAP_HPP_
