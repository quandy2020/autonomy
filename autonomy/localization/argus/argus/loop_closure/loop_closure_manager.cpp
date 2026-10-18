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

#include "argus/loop_closure/loop_closure_manager.hpp"

#include <algorithm>
#include <utility>

namespace argus {
namespace loop_closure {

LoopClosureManager::LoopClosureManager(LoopClosureConfig config)
    : config_(std::move(config)),
      detector_(config_.scan_context),
      consensus_(config_.consensus) {
    if (config_.cross_session.enable) {
        cross_session_linker_ = std::make_unique<CrossSessionLinker>(
            &sessions_, config_.cross_session);
    }
    // Ensure live session exists for trajectory bookkeeping.
    if (!config_.current_session_id.empty() &&
        !sessions_.Has(config_.current_session_id)) {
        map::SessionMeta live;
        live.session_id = config_.current_session_id;
        live.node_id_base = 0;
        sessions_.Register(live);
    }
}

LoopClosureManager::~LoopClosureManager() {
    Stop();
}

void LoopClosureManager::Start() {
    if (!config_.enable || !config_.async) {
        return;
    }
    if (running_.exchange(true)) {
        return;
    }
    keyframe_queue_.ResetBreak();
    worker_ = std::thread([this] { WorkerLoop(); });
}

void LoopClosureManager::Stop() {
    if (!running_.exchange(false)) {
        return;
    }
    keyframe_queue_.BreakAllWait();
    if (worker_.joinable()) {
        worker_.join();
    }
}

void LoopClosureManager::SubmitKeyFrame(KeyFrame kf) {
    if (!config_.enable || !running_.load()) {
        return;
    }
    if (kf.session_id.empty()) {
        kf.session_id = config_.current_session_id;
    }
    if (kf.session_local_id < 0) {
        kf.session_local_id = kf.id;
    }
    keyframe_queue_.Enqueue(std::move(kf));
}

bool LoopClosureManager::RegisterPriorSession(
    const map::SessionMeta& meta, std::vector<map::SessionKeyFrame> traj,
    const std::vector<KeyFrame>& scan_context_keyframes) {
    if (meta.session_id.empty()) {
        return false;
    }
    if (!sessions_.Register(meta)) {
        return false;
    }
    sessions_.SetTrajectory(meta.session_id, std::move(traj));
    for (const KeyFrame& kf : scan_context_keyframes) {
        detector_.IndexKeyFrame(kf);
    }
    if (cross_session_linker_) {
        cross_session_linker_->AddSessionToGraph(pose_graph_, meta.session_id);
    }
    return true;
}

std::vector<LoopCandidate> LoopClosureManager::TakeAcceptedLoops() {
    std::lock_guard<std::mutex> lock(results_mutex_);
    std::vector<LoopCandidate> out;
    out.swap(accepted_loops_);
    return out;
}

std::vector<LoopCorrection> LoopClosureManager::TakeCorrections() {
    std::lock_guard<std::mutex> lock(results_mutex_);
    std::vector<LoopCorrection> out;
    out.swap(corrections_);
    return out;
}

void LoopClosureManager::WorkerLoop() {
    KeyFrame kf;
    while (running_.load()) {
        if (!keyframe_queue_.WaitDequeue(&kf)) {
            continue;
        }
        if (!running_.load()) {
            break;
        }
        ProcessKeyFrame(std::move(kf));
    }
}

void LoopClosureManager::AppendKeyFrameToPath(const KeyFrame& kf) {
    const Eigen::Vector3d xyz = kf.T_w_b.translation();
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (!path_positions_.empty()) {
        const transform::Rigid3d T_prev =
            pose_graph_.HasNode(path_keyframe_ids_.back())
                ? pose_graph_.Pose(path_keyframe_ids_.back())
                : transform::Rigid3d::Identity();
        const transform::Rigid3d T_ij = T_prev.inverse() * kf.T_w_b;
        pose_graph_.AddOdometryEdge(path_keyframe_ids_.back(), kf.id, T_ij);
    }
    pose_graph_.AddNode(kf.id, kf.T_w_b);
    path_positions_.push_back(xyz);
    path_keyframe_ids_.push_back(kf.id);
    path_session_ids_.push_back(kf.session_id);

    // Append to live session trajectory for anchor refresh.
    if (sessions_.Has(kf.session_id)) {
        auto traj = sessions_.Trajectory(kf.session_id);
        map::SessionKeyFrame sk;
        sk.local_id = kf.session_local_id >= 0 ? kf.session_local_id : kf.id;
        sk.stamp_sec = kf.stamp_sec;
        const map::SessionMeta* meta = sessions_.Get(kf.session_id);
        sk.T_session_b = meta->T_world_session.inverse() * kf.T_w_b;
        traj.push_back(sk);
        sessions_.SetTrajectory(kf.session_id, std::move(traj));
    }
}

double LoopClosureManager::PathLengthBetween(int query_id, int match_id) {
    std::lock_guard<std::mutex> lock(path_mutex_);
    int qi = -1;
    int mi = -1;
    for (std::size_t k = 0; k < path_keyframe_ids_.size(); ++k) {
        if (path_keyframe_ids_[k] == query_id) {
            qi = static_cast<int>(k);
        }
        if (path_keyframe_ids_[k] == match_id) {
            mi = static_cast<int>(k);
        }
    }
    if (qi >= 0 && mi >= 0) {
        double path_len = 0.0;
        const int a = std::min(qi, mi);
        const int b = std::max(qi, mi);
        for (int k = a + 1; k <= b; ++k) {
            path_len += (path_positions_[static_cast<std::size_t>(k)] -
                         path_positions_[static_cast<std::size_t>(k - 1)])
                            .norm();
        }
        return path_len;
    }
    if (mi < 0) {
        // Match from prior session database — treat as long separation.
        return 1e3;
    }
    return 0.0;
}

bool LoopClosureManager::TryCrossSessionLink(const LoopCandidate& cand,
                                             const KeyFrame& kf,
                                             double confidence) {
    // Resolve match session from detector database via id heuristic:
    // prior nodes use node_id_base + local_id; live uses raw id.
    if (!cross_session_linker_ || !config_.cross_session.enable) {
        return false;
    }
    for (const std::string& session_id : sessions_.SessionIds()) {
        if (session_id == config_.current_session_id) {
            continue;
        }
        const map::SessionMeta* prior = sessions_.Get(session_id);
        if (prior == nullptr) {
            continue;
        }
        const int local_id = cand.match_id - prior->node_id_base;
        if (local_id < 0) {
            continue;
        }
        transform::Rigid3d unused;
        if (!sessions_.WorldPose(session_id, local_id, &unused)) {
            continue;
        }
        CrossSessionLink link;
        link.session_prior = session_id;
        link.session_current =
            kf.session_id.empty() ? config_.current_session_id : kf.session_id;
        link.local_id_prior = local_id;
        link.local_id_current =
            kf.session_local_id >= 0 ? kf.session_local_id : kf.id;
        link.T_cur_prior = cand.T_query_match;
        link.confidence = confidence;
        if (cross_session_linker_->LinkAndOptimize(pose_graph_, link, 8)) {
            cross_session_link_count_.fetch_add(1);
            return true;
        }
        break;
    }
    return false;
}

void LoopClosureManager::RecordAccepted(const LoopCandidate& cand,
                                        double confidence,
                                        const transform::Rigid3d& T_pre) {
    {
        std::lock_guard<std::mutex> lock(results_mutex_);
        accepted_loops_.push_back(cand);
        if (config_.enable_pose_inject &&
            pose_graph_.HasNode(cand.query_id)) {
            const transform::Rigid3d T_post = pose_graph_.Pose(cand.query_id);
            LoopCorrection correction;
            correction.query_id = cand.query_id;
            correction.match_id = cand.match_id;
            correction.delta_w = T_post * T_pre.inverse();
            correction.confidence = confidence;
            corrections_.push_back(correction);
        }
    }
    accepted_loop_count_.fetch_add(1);
}

void LoopClosureManager::ProcessKeyFrame(KeyFrame kf) {
    AppendKeyFrameToPath(kf);

    const auto candidates = detector_.Detect(kf);
    for (const LoopCandidate& cand : candidates) {
        const double path_length_to_match =
            PathLengthBetween(cand.query_id, cand.match_id);

        double confidence = 0.0;
        if (!consensus_.AcceptCandidate(cand, path_length_to_match,
                                        &confidence)) {
            continue;
        }

        const bool has_query = pose_graph_.HasNode(cand.query_id);
        const transform::Rigid3d T_pre =
            has_query ? pose_graph_.Pose(cand.query_id) : kf.T_w_b;

        if (!TryCrossSessionLink(cand, kf, confidence)) {
            backend::LoopFactor factor;
            factor.from_id = cand.query_id;
            factor.to_id = cand.match_id;
            factor.T_i_j = cand.T_query_match;
            factor.information = 200.0 * confidence;
            pose_graph_.AddLoopFactor(factor);
            pose_graph_.Optimize(8);
        }

        RecordAccepted(cand, confidence, T_pre);
    }
}

}  // namespace loop_closure
}  // namespace argus
