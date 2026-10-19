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

#include "argus/relocalization/loop_closure/cross_session.hpp"

#include <utility>
#include <vector>

namespace argus {
namespace loop_closure {

CrossSessionLinker::CrossSessionLinker(map::MapSession* sessions,
                                       CrossSessionConfig config)
    : sessions_(sessions), config_(std::move(config)) {}

int CrossSessionLinker::AddSessionToGraph(backend::PoseGraph& graph,
                                          const std::string& session_id) {
    if (sessions_ == nullptr || !config_.enable) {
        return 0;
    }
    const map::SessionMeta* meta = sessions_->Get(session_id);
    if (meta == nullptr) {
        return 0;
    }
    const auto& traj = sessions_->Trajectory(session_id);
    int added = 0;
    int prev_gid = -1;
    transform::Rigid3d prev_Twb = transform::Rigid3d::Identity();
    for (const map::SessionKeyFrame& kf : traj) {
        const int gid = sessions_->GlobalNodeId(session_id, kf.local_id);
        if (gid < 0) {
            continue;
        }
        if (graph.HasNode(gid)) {
            prev_gid = gid;
            prev_Twb = graph.Pose(gid);
            continue;
        }
        const transform::Rigid3d T_w_b = meta->T_world_session * kf.T_session_b;
        graph.AddNode(gid, T_w_b);
        if (prev_gid >= 0) {
            const transform::Rigid3d T_ij = prev_Twb.inverse() * T_w_b;
            const double information =
                config_.freeze_prior_nodes ? 1e4 : 50.0;
            graph.AddOdometryEdge(prev_gid, gid, T_ij, information);
        }
        prev_gid = gid;
        prev_Twb = T_w_b;
        ++added;
    }
    return added;
}

std::vector<backend::LoopFactor> CrossSessionLinker::BuildChainFactors(
    const CrossSessionLink& link) const {
    std::vector<backend::LoopFactor> out;
    if (sessions_ == nullptr || !config_.enable) {
        return out;
    }
    if (link.confidence < config_.min_confidence) {
        return out;
    }
    if (!sessions_->Has(link.session_prior) ||
        !sessions_->Has(link.session_current)) {
        return out;
    }
    const int from_id =
        sessions_->GlobalNodeId(link.session_current, link.local_id_current);
    const int to_id =
        sessions_->GlobalNodeId(link.session_prior, link.local_id_prior);
    if (from_id < 0 || to_id < 0) {
        return out;
    }
    backend::LoopFactor factor;
    factor.from_id = from_id;
    factor.to_id = to_id;
    factor.T_i_j = link.T_cur_prior;
    factor.information = config_.loop_information_scale * link.confidence;
    out.push_back(factor);
    return out;
}

bool CrossSessionLinker::LinkAndOptimize(backend::PoseGraph& graph,
                                         const CrossSessionLink& link,
                                         int max_iterations) {
    auto factors = BuildChainFactors(link);
    if (factors.empty()) {
        return false;
    }
    AddSessionToGraph(graph, link.session_prior);
    AddSessionToGraph(graph, link.session_current);

    for (const backend::LoopFactor& f : factors) {
        if (!graph.HasNode(f.from_id) || !graph.HasNode(f.to_id)) {
            return false;
        }
        graph.AddLoopFactor(f);
    }

    // Snapshot prior poses — PoseGraph fixes the lowest id, which is often the
    // live session; we restore prior as the absolute gauge after Optimize.
    const auto& prior_traj = sessions_->Trajectory(link.session_prior);
    std::vector<std::pair<int, transform::Rigid3d>> prior_snapshot;
    prior_snapshot.reserve(prior_traj.size());
    for (const map::SessionKeyFrame& kf : prior_traj) {
        const int gid =
            sessions_->GlobalNodeId(link.session_prior, kf.local_id);
        if (gid >= 0 && graph.HasNode(gid)) {
            prior_snapshot.emplace_back(gid, graph.Pose(gid));
        }
    }
    const int gid_prior =
        sessions_->GlobalNodeId(link.session_prior, link.local_id_prior);
    const int gid_cur =
        sessions_->GlobalNodeId(link.session_current, link.local_id_current);
    if (gid_prior < 0 || gid_cur < 0 || !graph.HasNode(gid_prior) ||
        !graph.HasNode(gid_cur)) {
        return false;
    }
    const transform::Rigid3d T_prior_fixed = graph.Pose(gid_prior);

    graph.Optimize(max_iterations);

    for (const auto& kv : prior_snapshot) {
        graph.SetPose(kv.first, kv.second);
    }

    // T_i_j ≈ T_i^{-1} T_j  ⇒  T_i_desired = T_j_fixed * T_i_j^{-1}
    const transform::Rigid3d T_cur_desired =
        T_prior_fixed * link.T_cur_prior.inverse();
    const transform::Rigid3d T_cur_opt = graph.Pose(gid_cur);
    const transform::Rigid3d delta = T_cur_desired * T_cur_opt.inverse();
    const auto& cur_traj = sessions_->Trajectory(link.session_current);
    for (const map::SessionKeyFrame& kf : cur_traj) {
        const int gid =
            sessions_->GlobalNodeId(link.session_current, kf.local_id);
        if (gid >= 0 && graph.HasNode(gid)) {
            graph.SetPose(gid, delta * graph.Pose(gid));
        }
    }

    map::SessionMeta* cur = sessions_->GetMutable(link.session_current);
    if (cur == nullptr || cur_traj.empty()) {
        return true;
    }
    const int gid0 = sessions_->GlobalNodeId(link.session_current,
                                             cur_traj.front().local_id);
    if (gid0 < 0 || !graph.HasNode(gid0)) {
        return true;
    }
    // T_w_s * T_s_b0 = T_w_b0  ⇒  T_w_s = T_w_b0 * T_s_b0^{-1}
    cur->T_world_session =
        graph.Pose(gid0) * cur_traj.front().T_session_b.inverse();
    return true;
}

}  // namespace loop_closure
}  // namespace argus
