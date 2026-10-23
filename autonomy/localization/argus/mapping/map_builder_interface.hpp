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
 * @file map_builder_interface.hpp
 * @brief Offline / online map assembly interface (session + geometry export).
 */

#ifndef ARGUS_MAPPING_MAP_BUILDER_INTERFACE_HPP_
#define ARGUS_MAPPING_MAP_BUILDER_INTERFACE_HPP_

#include <string>

#include "argus/common/math/transform/rigid_transform.hpp"
#include "argus/map/map_session.hpp"
#include "argus/sensor/point_types.hpp"

namespace argus {
namespace mapping {

/** @brief Keyframe payload for incremental map assembly. */
struct MapBuilderKeyFrame {
    int local_id = 0;
    double stamp_sec = 0.0;
    transform::Rigid3d T_w_b = transform::Rigid3d::Identity();
    /** @brief Body-frame or world-frame cloud (see `cloud_in_world`). */
    sensor::PointCloudXYZI cloud;
    bool cloud_in_world = false;
};

/**
 * @brief Interface for offline or online map building and session persistence.
 */
class MapBuilderInterface
{
public:
    virtual ~MapBuilderInterface() = default;

    /**
     * @brief Start a new mapping session.
     * @param session_id Unique session identifier for registry and export.
     */
    virtual void Reset(const std::string& session_id) = 0;

    /**
     * @brief Integrate one keyframe into the active map.
     * @param kf Pose, timestamp, and point cloud (body or world per flag).
     */
    virtual void AddKeyFrame(const MapBuilderKeyFrame& kf) = 0;

    /**
     * @brief Persist session registry and downsampled world cloud.
     * @param directory Output directory path.
     * @return True on successful write.
     */
    virtual bool Save(const std::string& directory) const = 0;

    /**
     * @brief Restore map state from a saved directory.
     * @param directory Directory previously passed to Save().
     * @return True on successful load.
     */
    virtual bool Load(const std::string& directory) = 0;

    /**
     * @brief Active session metadata (ids, transforms, registry).
     * @return Const reference to the session object.
     */
    virtual const map::MapSession& session() const = 0;

    /**
     * @brief Number of keyframes integrated in the current session.
     * @return Keyframe count.
     */
    virtual std::size_t keyframe_count() const = 0;

    /**
     * @brief Approximate number of map points after fusion / downsampling.
     * @return Map point count.
     */
    virtual std::size_t map_point_count() const = 0;
};

}  // namespace mapping
}  // namespace argus

#endif  // ARGUS_MAPPING_MAP_BUILDER_INTERFACE_HPP_
