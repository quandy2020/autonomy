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
 * @file map_session.hpp
 * @brief Multi-session registry: metadata, trajectory, world anchors.
 *
 * Related types: `session_meta.hpp`, `session_key_frame.hpp`.
 */

#ifndef ARGUS_MAP_MAP_SESSION_HPP_
#define ARGUS_MAP_MAP_SESSION_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "argus/common/macros.hpp"
#include "argus/common/math/transform/rigid_transform.hpp"
#include "argus/map/session_key_frame.hpp"
#include "argus/map/session_meta.hpp"

namespace argus {
namespace map {

/**
 * @brief In-memory multi-session registry with lightweight file IO.
 *
 * Full Protobuf map blobs are deferred; this stores header + trajectory
 * sufficient for cross-session pose-graph linking (see map_format.md).
 */
class MapSession
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(MapSession);

    /** @brief Remove all sessions and trajectories. */
    void Clear();

    /**
     * @brief Register or replace session metadata.
     * @param meta Session id, time span, and world anchor.
     * @return false if `session_id` is empty.
     */
    bool Register(const SessionMeta& meta);
    /**
     * @brief Test session presence.
     * @param session_id Session identifier.
     * @return true if registered.
     */
    bool Has(const std::string& session_id) const;
    /**
     * @brief Lookup session metadata.
     * @param session_id Session identifier.
     * @return Pointer to meta, or nullptr if unknown.
     */
    const SessionMeta* Get(const std::string& session_id) const;
    /**
     * @brief Mutable metadata lookup.
     * @param session_id Session identifier.
     * @return Pointer to meta, or nullptr if unknown.
     */
    SessionMeta* GetMutable(const std::string& session_id);

    /**
     * @brief Replace trajectory for an existing session.
     * @param session_id Registered session id.
     * @param traj Ordered keyframes in session frame.
     */
    void SetTrajectory(const std::string& session_id,
                       std::vector<SessionKeyFrame> traj);
    /**
     * @brief Trajectory for a session.
     * @param session_id Session identifier.
     * @return Keyframe list, or empty static vector if unknown.
     */
    const std::vector<SessionKeyFrame>& Trajectory(
        const std::string& session_id) const;

    /**
     * @brief World pose of a session keyframe: \(T_{w b}=T_{w s} T_{s b}\).
     * @param session_id Session identifier.
     * @param local_id Keyframe id within the session.
     * @param[out] T_w_b Filled world body pose on success.
     * @return false if session, trajectory, or local_id is missing.
     */
    bool WorldPose(const std::string& session_id, int local_id,
                   transform::Rigid3d* T_w_b) const;

    /**
     * @brief Pose-graph node id for a session-local keyframe.
     * @param session_id Session identifier.
     * @param local_id Local keyframe index.
     * @return Global node id, or -1 if session unknown.
     */
    int GlobalNodeId(const std::string& session_id, int local_id) const;

    /** @return All registered session ids (unordered). */
    std::vector<std::string> SessionIds() const;

    /**
     * @brief Text registry (version, metas, trajectories).
     * @param path Output file path.
     * @return false on I/O error.
     */
    bool Save(const std::string& path) const;
    /**
     * @brief Load registry from text file.
     * @param path Input file path.
     * @return false on parse or I/O error.
     */
    bool Load(const std::string& path);

    /** @return Logical map format version constant. */
    int map_format_version() const {
        return kMapFormatVersion;
    }

private:
    std::unordered_map<std::string, SessionMeta> sessions_;
    std::unordered_map<std::string, std::vector<SessionKeyFrame>> trajectories_;
    static const std::vector<SessionKeyFrame> kEmptyTraj;
};

}  // namespace map
}  // namespace argus

#endif  // ARGUS_MAP_MAP_SESSION_HPP_
