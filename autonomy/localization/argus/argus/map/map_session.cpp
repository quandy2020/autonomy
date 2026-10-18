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

#include "argus/map/map_session.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

#include <Eigen/Geometry>

namespace argus {
namespace map {

const std::vector<SessionKeyFrame> MapSession::kEmptyTraj{};

namespace {

void WriteRigid3(std::ostream& os, const transform::Rigid3d& T) {
    const Eigen::Matrix3d R = T.rotation().toRotationMatrix();
    const Eigen::Vector3d t = T.translation();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            os << R(r, c) << ' ';
        }
    }
    os << t.x() << ' ' << t.y() << ' ' << t.z();
}

bool ReadRigid3(std::istream& is, transform::Rigid3d* T) {
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            if (!(is >> R(r, c))) {
                return false;
            }
        }
    }
    if (!(is >> t.x() >> t.y() >> t.z())) {
        return false;
    }
    *T = transform::Rigid3d(t, Eigen::Quaterniond(R));
    return true;
}

}  // namespace

void MapSession::Clear() {
    sessions_.clear();
    trajectories_.clear();
}

bool MapSession::Register(const SessionMeta& meta) {
    if (meta.session_id.empty()) {
        return false;
    }
    sessions_[meta.session_id] = meta;
    if (trajectories_.find(meta.session_id) == trajectories_.end()) {
        trajectories_[meta.session_id] = {};
    }
    return true;
}

bool MapSession::Has(const std::string& session_id) const {
    return sessions_.find(session_id) != sessions_.end();
}

const SessionMeta* MapSession::Get(const std::string& session_id) const {
    const auto it = sessions_.find(session_id);
    return it == sessions_.end() ? nullptr : &it->second;
}

SessionMeta* MapSession::GetMutable(const std::string& session_id) {
    const auto it = sessions_.find(session_id);
    return it == sessions_.end() ? nullptr : &it->second;
}

void MapSession::SetTrajectory(const std::string& session_id,
                               std::vector<SessionKeyFrame> traj) {
    trajectories_[session_id] = std::move(traj);
}

const std::vector<SessionKeyFrame>& MapSession::Trajectory(
    const std::string& session_id) const {
    const auto it = trajectories_.find(session_id);
    return it == trajectories_.end() ? kEmptyTraj : it->second;
}

bool MapSession::WorldPose(const std::string& session_id, int local_id,
                           transform::Rigid3d* T_w_b) const {
    if (T_w_b == nullptr) {
        return false;
    }
    const SessionMeta* meta = Get(session_id);
    if (meta == nullptr) {
        return false;
    }
    const auto& traj = Trajectory(session_id);
    for (const SessionKeyFrame& kf : traj) {
        if (kf.local_id == local_id) {
            *T_w_b = meta->T_world_session * kf.T_session_b;
            return true;
        }
    }
    return false;
}

int MapSession::GlobalNodeId(const std::string& session_id,
                             int local_id) const {
    const SessionMeta* meta = Get(session_id);
    if (meta == nullptr) {
        return -1;
    }
    return meta->node_id_base + local_id;
}

std::vector<std::string> MapSession::SessionIds() const {
    std::vector<std::string> ids;
    ids.reserve(sessions_.size());
    for (const auto& kv : sessions_) {
        ids.push_back(kv.first);
    }
    return ids;
}

bool MapSession::Save(const std::string& path) const {
    std::ofstream os(path);
    if (!os) {
        return false;
    }
    os.setf(std::ios::fixed, std::ios::floatfield);
    os << std::setprecision(9);
    os << "argus_map_session " << kMapFormatVersion << '\n';
    os << "sessions " << sessions_.size() << '\n';
    for (const auto& kv : sessions_) {
        const SessionMeta& m = kv.second;
        os << "meta " << m.session_id << ' ' << m.t_start_sec << ' '
           << m.t_end_sec << ' ' << m.mean_confidence << ' ' << m.node_id_base
           << ' ';
        WriteRigid3(os, m.T_world_session);
        os << '\n';
        const auto& traj = Trajectory(m.session_id);
        os << "traj " << m.session_id << ' ' << traj.size() << '\n';
        for (const SessionKeyFrame& kf : traj) {
            os << kf.local_id << ' ' << kf.stamp_sec << ' ';
            WriteRigid3(os, kf.T_session_b);
            os << '\n';
        }
    }
    return static_cast<bool>(os);
}

bool MapSession::Load(const std::string& path) {
    std::ifstream is(path);
    if (!is) {
        return false;
    }
    Clear();
    std::string tag;
    int version = 0;
    if (!(is >> tag >> version) || tag != "argus_map_session") {
        return false;
    }
    if (version != kMapFormatVersion) {
        return false;
    }
    int n_sessions = 0;
    if (!(is >> tag >> n_sessions) || tag != "sessions") {
        return false;
    }
    for (int s = 0; s < n_sessions; ++s) {
        SessionMeta m;
        if (!(is >> tag) || tag != "meta") {
            return false;
        }
        if (!(is >> m.session_id >> m.t_start_sec >> m.t_end_sec >>
              m.mean_confidence >> m.node_id_base)) {
            return false;
        }
        if (!ReadRigid3(is, &m.T_world_session)) {
            return false;
        }
        Register(m);

        std::string sid;
        int n_traj = 0;
        if (!(is >> tag >> sid >> n_traj) || tag != "traj") {
            return false;
        }
        std::vector<SessionKeyFrame> traj;
        traj.reserve(static_cast<std::size_t>(n_traj));
        for (int i = 0; i < n_traj; ++i) {
            SessionKeyFrame kf;
            if (!(is >> kf.local_id >> kf.stamp_sec)) {
                return false;
            }
            if (!ReadRigid3(is, &kf.T_session_b)) {
                return false;
            }
            traj.push_back(kf);
        }
        SetTrajectory(sid, std::move(traj));
    }
    return true;
}

}  // namespace map
}  // namespace argus
