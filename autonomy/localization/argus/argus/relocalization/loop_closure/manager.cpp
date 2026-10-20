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

#include "argus/relocalization/loop_closure/manager.hpp"

#include <algorithm>
#include <cstdlib>
#include <utility>

#include "argus/relocalization/loop_closure/geometric_verify.hpp"

namespace argus {
namespace loop_closure {
namespace {

sensor::PointCloudXYZI TransformCloudToWorld(
    const sensor::PointCloudXYZI& body, const transform::Rigid3d& T_w_b) {
    sensor::PointCloudXYZI world;
    world.reserve(body.size());
    for (const auto& p : body.points) {
        const Eigen::Vector3d pw = T_w_b * Eigen::Vector3d(p.x, p.y, p.z);
        sensor::PointType q = p;
        q.x = static_cast<float>(pw.x());
        q.y = static_cast<float>(pw.y());
        q.z = static_cast<float>(pw.z());
        world.push_back(q);
    }
    return world;
}

}  // namespace

LoopClosureManager::LoopClosureManager(LoopClosureConfig config)
    : config_(std::move(config)),
      detector_(config_.scan_context),
      consensus_(config_.consensus),
      map_reload_(config_.lta) {
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
    BufferPriorKeyFrames(meta.session_id, scan_context_keyframes);
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
        backend::PoseGraphOptimizeResult opt;
        if (cross_session_linker_->LinkAndOptimize(pose_graph_, link,
                                                   config_.fpr, 8, &opt)) {
            cross_session_link_count_.fetch_add(1);
            if (config_.lta_reload_prior_on_cross_session) {
                (void)EnqueuePriorLtaClouds(link.session_prior,
                                            link.local_id_prior);
            }
            return true;
        }
        if (opt.rejected_by_fpr) {
            fpr_reject_count_.fetch_add(1);
        }
        break;
    }
    return false;
}

void LoopClosureManager::NotifyPoseJump() {
    discard_next_keyframe_.store(true);
    std::lock_guard<std::mutex> lock(lta_mutex_);
    pending_second_loop_.clear();
    temporal_buffer_.clear();
}

LoopClosureStats LoopClosureManager::Stats() const {
    LoopClosureStats s;
    s.accepted = accepted_loop_count_.load();
    s.fpr_reject = fpr_reject_count_.load();
    s.inject_restore = inject_restore_count_.load();
    s.geometric_reject = geometric_reject_count_.load();
    s.geom_missing_cloud = geom_missing_cloud_.load();
    s.geom_empty_query = geom_empty_query_.load();
    s.geom_fitness = geom_fitness_.load();
    s.geom_corr = geom_corr_.load();
    s.geom_translation = geom_translation_.load();
    s.geom_yaw = geom_yaw_.load();
    s.geom_inlier = geom_inlier_.load();
    s.geom_other = geom_other_.load();
    s.temporal_reject = temporal_reject_count_.load();
    s.temporal_pending = temporal_pending_count_.load();
    s.signature_reject = signature_reject_count_.load();
    s.cross_session = cross_session_link_count_.load();
    return s;
}

void LoopClosureManager::NoteGeomFail(GeomFailReason reason) {
    geometric_reject_count_.fetch_add(1);
    switch (reason) {
        case GeomFailReason::kEmptyCloud:
            geom_empty_query_.fetch_add(1);
            break;
        case GeomFailReason::kTooFewCorrespondences:
        case GeomFailReason::kTooFewPoints:
            geom_corr_.fetch_add(1);
            break;
        case GeomFailReason::kFitness:
            geom_fitness_.fetch_add(1);
            break;
        case GeomFailReason::kTranslation:
            geom_translation_.fetch_add(1);
            break;
        case GeomFailReason::kYaw:
            geom_yaw_.fetch_add(1);
            break;
        case GeomFailReason::kInlierRatio:
            geom_inlier_.fetch_add(1);
            break;
        default:
            geom_other_.fetch_add(1);
            break;
    }
}

void LoopClosureManager::BufferLtaKeyFrame(const KeyFrame& kf) {
    std::lock_guard<std::mutex> lock(lta_mutex_);
    if (!kf.cloud.empty()) {
        if (keyframe_clouds_.find(kf.id) == keyframe_clouds_.end()) {
            keyframe_cloud_order_.push_back(kf.id);
        }
        keyframe_clouds_[kf.id] = kf.cloud;
    }
    if (config_.lta.enable) {
        lta_keyframes_.push_back(kf);
        const int cap = std::max(8, config_.lta_keyframe_buffer);
        while (static_cast<int>(lta_keyframes_.size()) > cap) {
            lta_keyframes_.pop_front();
        }
    }
    // Cap cloud cache (ICP) even when LTA is off — LRU by insertion order.
    const int cloud_cap = std::max(64, config_.lta_keyframe_buffer);
    while (static_cast<int>(keyframe_clouds_.size()) > cloud_cap &&
           !keyframe_cloud_order_.empty()) {
        const int old_id = keyframe_cloud_order_.front();
        keyframe_cloud_order_.pop_front();
        keyframe_clouds_.erase(old_id);
    }
}

bool LoopClosureManager::RefineCandidateGeometry(const KeyFrame& query_kf,
                                                 LoopCandidate* cand) {
    if (cand == nullptr) {
        return false;
    }
    if (!config_.geometric_verify.enable) {
        return true;
    }
    sensor::PointCloudXYZI match_cloud;
    {
        std::lock_guard<std::mutex> lock(lta_mutex_);
        auto it = keyframe_clouds_.find(cand->match_id);
        if (it == keyframe_clouds_.end() || it->second.empty()) {
            geometric_reject_count_.fetch_add(1);
            geom_missing_cloud_.fetch_add(1);
            return false;
        }
        match_cloud = it->second;
    }
    if (query_kf.cloud.empty()) {
        NoteGeomFail(GeomFailReason::kEmptyCloud);
        return false;
    }
    const auto verified = VerifyAndRefineLoop(
        query_kf.cloud, match_cloud, cand->T_query_match, config_.geometric_verify);
    if (!verified.ok) {
        NoteGeomFail(verified.fail_reason);
        return false;
    }
    cand->T_query_match = verified.T_query_match;
    cand->fitness_m = verified.fitness_m;
    cand->inlier_ratio = verified.inlier_ratio;
    return true;
}

bool LoopClosureManager::PassesRevisitSignature(
    const LoopCandidate& cand) const {
    if (!config_.revisit_signature.enable) {
        return true;
    }
    if (!pose_graph_.HasNode(cand.query_id) ||
        !pose_graph_.HasNode(cand.match_id)) {
        return false;
    }
    const transform::Rigid3d T_odom =
        pose_graph_.Pose(cand.query_id).inverse() *
        pose_graph_.Pose(cand.match_id);
    const double odom_gap = T_odom.translation().norm();
    const double icp_t = cand.T_query_match.translation().norm();
    const double fit_lim = config_.revisit_signature.max_fitness_m > 0.0
                               ? config_.revisit_signature.max_fitness_m
                               : config_.geometric_verify.max_fitness_m;
    if (cand.fitness_m > fit_lim) {
        return false;
    }
    if (icp_t > config_.revisit_signature.max_icp_trans_m) {
        return false;
    }
    if (odom_gap < config_.revisit_signature.min_odom_gap_m) {
        return false;
    }
    if ((odom_gap - icp_t) < config_.revisit_signature.min_gap_minus_icp_m) {
        return false;
    }
    return true;
}

bool LoopClosureManager::ConfirmTemporal(const LoopCandidate& cand,
                                         double confidence,
                                         const transform::Rigid3d& T_pre,
                                         LoopCandidate* out,
                                         double* out_confidence,
                                         transform::Rigid3d* out_T_pre) {
    if (out == nullptr || out_confidence == nullptr || out_T_pre == nullptr) {
        return false;
    }
    if (!config_.temporal.enable) {
        *out = cand;
        *out_confidence = confidence;
        *out_T_pre = T_pre;
        return true;
    }
    const int need = std::max(1, config_.temporal.min_confirmations);
    const int match_delta = std::max(0, config_.temporal.max_match_id_delta);
    const int query_span = std::max(1, config_.temporal.max_query_span);

    std::lock_guard<std::mutex> lock(lta_mutex_);
    // Drop stale entries outside the query window.
    while (!temporal_buffer_.empty() &&
           std::abs(temporal_buffer_.front().cand.query_id - cand.query_id) >
               query_span) {
        temporal_buffer_.pop_front();
    }

    int support = 1;  // current candidate
    for (const TemporalEntry& e : temporal_buffer_) {
        if (std::abs(e.cand.match_id - cand.match_id) <= match_delta &&
            std::abs(e.cand.query_id - cand.query_id) <= query_span &&
            e.cand.query_id != cand.query_id) {
            ++support;
        }
    }
    temporal_buffer_.push_back(TemporalEntry{cand, confidence, T_pre});
    while (static_cast<int>(temporal_buffer_.size()) > 32) {
        temporal_buffer_.pop_front();
    }

    if (support < need) {
        temporal_pending_count_.fetch_add(1);
        temporal_reject_count_.fetch_add(1);
        return false;
    }
    *out = cand;
    *out_confidence = confidence;
    *out_T_pre = T_pre;
    temporal_buffer_.clear();
    return true;
}

std::string LoopClosureManager::PriorCloudKey(const std::string& session_id,
                                              int local_id) {
    return session_id + "#" + std::to_string(local_id);
}

void LoopClosureManager::BufferPriorKeyFrames(
    const std::string& session_id, const std::vector<KeyFrame>& kfs) {
    if (session_id.empty() || kfs.empty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(lta_mutex_);
    for (const KeyFrame& kf : kfs) {
        if (kf.cloud.empty()) {
            continue;
        }
        const int local =
            kf.session_local_id >= 0 ? kf.session_local_id : kf.id;
        prior_clouds_[PriorCloudKey(session_id, local)] = kf.cloud;
    }
}

std::size_t LoopClosureManager::EnqueuePriorLtaClouds(
    const std::string& session_id, int local_id) {
    if (!config_.lta.enable || session_id.empty()) {
        return 0;
    }
    const int radius = std::max(0, config_.lta.prior_neighbor_keyframes);
    const int lo = local_id - radius;
    const int hi = local_id + radius;

    // Collect body clouds under lock, then transform unlocked.
    std::vector<std::pair<int, sensor::PointCloudXYZI>> batch;
    {
        std::lock_guard<std::mutex> lock(lta_mutex_);
        batch.reserve(static_cast<std::size_t>(2 * radius + 1));
        for (int lid = lo; lid <= hi; ++lid) {
            auto it = prior_clouds_.find(PriorCloudKey(session_id, lid));
            if (it == prior_clouds_.end() || it->second.empty()) {
                continue;
            }
            batch.emplace_back(lid, it->second);
        }
    }

    std::size_t enqueued = 0;
    for (const auto& item : batch) {
        transform::Rigid3d T_w_b;
        if (!sessions_.WorldPose(session_id, item.first, &T_w_b)) {
            continue;
        }
        map_reload_.EnqueueCorrectedCloud(
            TransformCloudToWorld(item.second, T_w_b));
        ++enqueued;
    }
    if (enqueued > 0) {
        map::MapReloadHint hint;
        hint.query_id = -1;
        hint.match_id = local_id;
        hint.confidence = 1.0;
        map_reload_.SubmitHint(hint);
    }
    return enqueued;
}

std::size_t LoopClosureManager::ReloadPriorNeighborhood(
    const std::string& session_id, int local_id) {
    return EnqueuePriorLtaClouds(session_id, local_id);
}

std::size_t LoopClosureManager::prior_cloud_count() const {
    std::lock_guard<std::mutex> lock(lta_mutex_);
    return prior_clouds_.size();
}

void LoopClosureManager::EnqueueLtaClouds(const transform::Rigid3d& delta_w,
                                          int match_id, int query_id) {
    if (!config_.lta.enable) {
        return;
    }
    const int lo = std::min(match_id, query_id);
    const int hi = std::max(match_id, query_id);
    std::lock_guard<std::mutex> lock(lta_mutex_);
    for (const KeyFrame& kf : lta_keyframes_) {
        if (kf.id < lo || kf.id > hi || kf.cloud.empty()) {
            continue;
        }
        // Body cloud → world at pre-correction pose, then apply Δw.
        sensor::PointCloudXYZI world =
            TransformCloudToWorld(kf.cloud, kf.T_w_b);
        for (auto& p : world.points) {
            const Eigen::Vector3d pc =
                delta_w * Eigen::Vector3d(p.x, p.y, p.z);
            p.x = static_cast<float>(pc.x());
            p.y = static_cast<float>(pc.y());
            p.z = static_cast<float>(pc.z());
        }
        map_reload_.EnqueueCorrectedCloud(std::move(world));
    }
    map::MapReloadHint hint;
    hint.query_id = query_id;
    hint.match_id = match_id;
    hint.delta_w = delta_w;
    map_reload_.SubmitHint(hint);
}

void LoopClosureManager::RecordAccepted(const LoopCandidate& cand,
                                        double confidence,
                                        const transform::Rigid3d& T_pre,
                                        bool apply_to_state,
                                        bool clear_eval_drift) {
    transform::Rigid3d delta_w = transform::Rigid3d::Identity();
    bool have_delta = false;
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
            correction.apply_to_state = apply_to_state;
            correction.clear_eval_drift = clear_eval_drift;
            if (!apply_to_state) {
                correction.delta_w = transform::Rigid3d::Identity();
            }
            delta_w = correction.delta_w;
            have_delta = true;
            corrections_.push_back(correction);
        }
    }
    accepted_loop_count_.fetch_add(1);
    if (have_delta && apply_to_state) {
        EnqueueLtaClouds(delta_w, cand.match_id, cand.query_id);
    }
}

bool LoopClosureManager::AcceptIntraSessionLoop(
    const LoopCandidate& cand, double confidence,
    const transform::Rigid3d& T_pre) {
    if (config_.max_loop_translation_m > 0.0 &&
        cand.T_query_match.translation().norm() >
            config_.max_loop_translation_m) {
        return false;
    }
    if (!PassesRevisitSignature(cand)) {
        signature_reject_count_.fetch_add(1);
        return false;
    }

    std::vector<LoopCandidate> batch;
    {
        std::lock_guard<std::mutex> lock(lta_mutex_);
        if (config_.wait_second_loop) {
            pending_second_loop_.push_back(cand);
            if (pending_second_loop_.size() < 2) {
                return false;
            }
            batch.swap(pending_second_loop_);
        } else {
            batch.push_back(cand);
        }
    }

    std::vector<backend::LoopFactor> trial;
    trial.reserve(batch.size());
    for (const LoopCandidate& c : batch) {
        backend::LoopFactor factor;
        factor.from_id = c.query_id;
        factor.to_id = c.match_id;
        factor.T_i_j = c.T_query_match;
        factor.information =
            config_.loop_information_scale * std::max(0.05, confidence);
        trial.push_back(factor);
    }

    // Signature already checked above (pre-Optimize). Do not re-check after
    // Optimize — closing the loop shrinks the odom gap and would false-reject.
    const bool pre_sig_ok = true;
    const auto backup = pose_graph_.Snapshot();
    const auto opt =
        pose_graph_.TryAcceptLoops(trial, config_.fpr, /*max_iterations=*/8);
    if (!opt.ok) {
        fpr_reject_count_.fetch_add(1);
        return false;
    }
    bool apply_to_state = true;
    bool clear_eval_drift = false;
    // Guard ESIKF inject magnitude. Default: Restore graph. With
    // `inject_clamp_only` after a passed revisit signature: keep edge, skip
    // yank, clear Td.
    if (config_.max_inject_translation_m > 0.0 &&
        pose_graph_.HasNode(cand.query_id)) {
        const transform::Rigid3d T_post = pose_graph_.Pose(cand.query_id);
        const transform::Rigid3d delta = T_post * T_pre.inverse();
        if (delta.translation().norm() > config_.max_inject_translation_m) {
            inject_restore_count_.fetch_add(1);
            if (config_.inject_clamp_only && pre_sig_ok) {
                apply_to_state = false;
                clear_eval_drift = true;  // Td recovery without state yank
            } else {
                pose_graph_.Restore(backup);
                return false;
            }
        } else {
            clear_eval_drift = true;  // safe inject also clears Td
        }
    } else {
        clear_eval_drift = true;
    }
    RecordAccepted(cand, confidence, T_pre, apply_to_state, clear_eval_drift);
    return true;
}

void LoopClosureManager::ProcessKeyFrame(KeyFrame kf) {
    if (discard_next_keyframe_.exchange(false)) {
        // Drop the first KF after a pose jump (half-built submap).
        return;
    }

    AppendKeyFrameToPath(kf);
    BufferLtaKeyFrame(kf);

    const auto candidates = detector_.Detect(kf);
    for (const LoopCandidate& cand : candidates) {
        const double path_length_to_match =
            PathLengthBetween(cand.query_id, cand.match_id);

        double confidence = 0.0;
        if (!consensus_.AcceptCandidate(cand, path_length_to_match,
                                        &confidence)) {
            continue;
        }

        LoopCandidate refined = cand;
        if (!RefineCandidateGeometry(kf, &refined)) {
            continue;
        }

        const bool has_query = pose_graph_.HasNode(refined.query_id);
        const transform::Rigid3d T_pre =
            has_query ? pose_graph_.Pose(refined.query_id) : kf.T_w_b;

        if (TryCrossSessionLink(refined, kf, confidence)) {
            RecordAccepted(refined, confidence, T_pre);
            continue;
        }

        LoopCandidate confirmed = refined;
        double conf_out = confidence;
        transform::Rigid3d T_pre_out = T_pre;
        if (!ConfirmTemporal(refined, confidence, T_pre, &confirmed, &conf_out,
                             &T_pre_out)) {
            continue;
        }
        AcceptIntraSessionLoop(confirmed, conf_out, T_pre_out);
    }
}

}  // namespace loop_closure
}  // namespace argus
