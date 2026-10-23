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
 * @file manager.hpp
 * @brief Async loop-closure worker (never runs on ESIKF thread).
 */

#ifndef ARGUS_RELOCALIZATION_LOOP_MANAGER_HPP_
#define ARGUS_RELOCALIZATION_LOOP_MANAGER_HPP_

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "argus/backend/pose_graph/pose_graph.hpp"
#include "argus/common/macros.hpp"
#include "argus/common/thread_safe_queue.hpp"
#include "argus/map/map_reload_worker.hpp"
#include "argus/map/map_session.hpp"
#include "argus/relocalization/loop_closure/consensus.hpp"
#include "argus/relocalization/loop_closure/cross_session.hpp"
#include "argus/relocalization/loop_closure/geometric_verify.hpp"
#include "argus/relocalization/loop_closure/scan_context/detector.hpp"
#include "argus/relocalization/loop_closure/types.hpp"

namespace argus {
namespace loop_closure {

/**
 * @brief Require N geom-ok candidates with nearby match ids before Optimize.
 *        Stronger than blind `wait_second_loop` on corridor aliases.
 */
struct TemporalConsistencyConfig {
    bool enable = false;
    /** @brief Min geom-ok candidates with consistent match ids. */
    int min_confirmations = 2;
    /** @brief Max |match_id_i − match_id_j| among confirmations. */
    int max_match_id_delta = 10;
    /** @brief Max |query_id_i − query_id_j| window (keyframes). */
    int max_query_span = 20;
};

/**
 * @brief True-revisit signature: ICP says co-located, pose-graph odometry gap
 *        is large (drift / \(T_d\)). Rejects corridor aliases with large ICP |t|.
 */
struct RevisitSignatureConfig {
    bool enable = false;
    /** @brief Min |t| of \(T_{q\leftarrow m}\) from pose-graph poses (m). */
    double min_odom_gap_m = 2.0;
    /** @brief Max |t| of ICP-refined relative (m). */
    double max_icp_trans_m = 1.5;
    /** @brief Min (odom_gap − icp_t) (m). */
    double min_gap_minus_icp_m = 1.0;
    /**
     * @brief Extra fitness gate (m); 0 uses `geometric_verify.max_fitness_m`.
     */
    double max_fitness_m = 0.0;
};

/** @brief Snapshot of loop-closure accept / reject counters. */
struct LoopClosureStats {
    int accepted = 0;
    int fpr_reject = 0;
    int inject_restore = 0;
    int geometric_reject = 0;
    int geom_missing_cloud = 0;
    int geom_empty_query = 0;
    int geom_fitness = 0;
    int geom_corr = 0;
    int geom_translation = 0;
    int geom_yaw = 0;
    int geom_inlier = 0;
    int geom_other = 0;
    int temporal_reject = 0;
    int temporal_pending = 0;
    int signature_reject = 0;
    int cross_session = 0;
};

/** @brief Async loop-closure worker and pose-graph inject settings. */
struct LoopClosureConfig {
    bool enable = false;
    bool async = true;  // must stay true
    ScanContextConfig scan_context;
    MultiModalConsensusConfig consensus;
    CrossSessionConfig cross_session;
    double keyframe_distance_m = 2.0;
    double keyframe_yaw_rad = 0.5;
    /** @brief Live session id stamped on submitted keyframes. */
    std::string current_session_id = "live";
    /**
     * @brief After Optimize, enqueue ΔT for estimate-thread inject.
     *        Default on when loop closure is enabled.
     */
    bool enable_pose_inject = true;
    /** @brief Hard FPR (optimize-and-recover) on pose graph. */
    backend::PoseGraphFprConfig fpr;
    /** @brief ICP refine + fitness gate after Scan Context. */
    GeometricVerifyConfig geometric_verify;
    /** @brief Match-id consistency across nearby queries. */
    TemporalConsistencyConfig temporal;
    /** @brief Odometry-gap vs ICP co-location check. */
    RevisitSignatureConfig revisit_signature;
    /**
     * @brief Wait for a second consensus-passed LC before optimizing (LTA-OM).
     *        Prefer `temporal` when both are considered.
     */
    bool wait_second_loop = false;
    /** @brief Max |t| of relative loop (m); 0 disables. */
    double max_loop_translation_m = 0.0;
    /**
     * @brief Max |t| of ESIKF inject \(\Delta_w\) after Optimize (m).
     *        0 disables the check. Default: Restore graph when exceeded.
     */
    double max_inject_translation_m = 5.0;
    /**
     * @brief If true, large \(\Delta\) keeps the pose-graph loop but skips
     *        ESIKF yank (`LoopCorrection::apply_to_state=false`). Used for
     *        synthetic-drift recovery where clearing \(T_d\) is enough.
     */
    bool inject_clamp_only = false;
    /**
     * @brief Loop factor information scale (\(info = scale \times conf\)).
     *        Keep soft on corridor scenes to limit Optimize yank.
     */
    double loop_information_scale = 50.0;
    /** @brief LTA history-map reload (default off). */
    map::MapReloadConfig lta;
    /** @brief Keyframes retained for LTA cloud transform. */
    int lta_keyframe_buffer = 64;
    /**
     * @brief After cross-session link, enqueue prior KF clouds into live map
     *        via MapReloadWorker (requires `lta.enable`).
     */
    bool lta_reload_prior_on_cross_session = true;
};

/** @brief Background loop detection, pose-graph optimization, and correction
 * export. */
class LoopClosureManager
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(LoopClosureManager);

    /**
     * @brief Construct manager; call Start() before submitting keyframes.
     * @param config Detector, consensus, and inject options.
     */
    explicit LoopClosureManager(LoopClosureConfig config = {});
    ~LoopClosureManager();

    LoopClosureManager(const LoopClosureManager&) = delete;
    LoopClosureManager& operator=(const LoopClosureManager&) = delete;

    /** @brief Start the async worker thread. */
    void Start();
    /** @brief Stop worker and drain pending queue. */
    void Stop();

    /**
     * @brief Non-blocking; safe from ESIKF / LIO thread.
     * @param kf Keyframe snapshot copied into the worker queue.
     */
    void SubmitKeyFrame(KeyFrame kf);

    /**
     * @brief Register a prior map session (metadata + traj + SC database).
     *        Safe before Start() or from any thread (locks internally).
     * @param meta Session metadata and world anchor.
     * @param traj Keyframe trajectory in session frame.
     * @param scan_context_keyframes Optional Scan Context keyframes for
     *        cross-session match.
     * @return false if registration fails (duplicate id, invalid meta).
     */
    bool RegisterPriorSession(
        const map::SessionMeta& meta, std::vector<map::SessionKeyFrame> traj,
        const std::vector<KeyFrame>& scan_context_keyframes = {});

    /** @return Mutable multi-session registry. */
    map::MapSession& sessions() {
        return sessions_;
    }
    /** @return Const session registry. */
    const map::MapSession& sessions() const {
        return sessions_;
    }

    /**
     * @brief Drain accepted loops (logging / diagnostics).
     * @return Accepted candidates since last call; queue cleared.
     */
    std::vector<LoopCandidate> TakeAcceptedLoops();

    /**
     * @brief Drain Optimize ΔT corrections for ESIKF atomic inject.
     * @return World-frame corrections since last call; queue cleared.
     */
    std::vector<LoopCorrection> TakeCorrections();

    /** @return Mutable pose graph used by the worker. */
    backend::PoseGraph& pose_graph() {
        return pose_graph_;
    }
    /** @return Const pose graph. */
    const backend::PoseGraph& pose_graph() const {
        return pose_graph_;
    }

    /** @return Total accepted intra-session loops (monotonic). */
    int accepted_loop_count() const {
        return accepted_loop_count_.load();
    }
    /** @return Total cross-session links applied (monotonic). */
    int cross_session_link_count() const {
        return cross_session_link_count_.load();
    }
    /** @return Loops rejected by pose-graph FPR (monotonic). */
    int fpr_reject_count() const {
        return fpr_reject_count_.load();
    }
    /** @return Graph restores due to inject |\Delta t| cap (monotonic). */
    int inject_restore_count() const {
        return inject_restore_count_.load();
    }
    /** @return Candidates rejected by geometric ICP gate (monotonic). */
    int geometric_reject_count() const {
        return geometric_reject_count_.load();
    }
    /** @return Candidates held/rejected by temporal match consistency. */
    int temporal_reject_count() const {
        return temporal_reject_count_.load();
    }
    /** @return Candidates rejected by revisit signature. */
    int signature_reject_count() const {
        return signature_reject_count_.load();
    }
    /** @return Aggregated accept/reject histogram. */
    LoopClosureStats Stats() const;

    /** @brief LTA reload queue (estimate thread drains). */
    map::MapReloadWorker& map_reload() {
        return map_reload_;
    }
    const map::MapReloadWorker& map_reload() const {
        return map_reload_;
    }

    /**
     * @brief Signal a pose jump (LTA/reloc): discard in-flight KF accumulation.
     *        Safe from estimate thread.
     */
    void NotifyPoseJump();

    /**
     * @brief Enqueue ±`lta.prior_neighbor_keyframes` prior clouds into reload
     *        queue (world frame via current session anchors). Worker / tests.
     * @return Number of keyframe clouds enqueued.
     */
    std::size_t ReloadPriorNeighborhood(const std::string& session_id,
                                        int local_id);

    /** @return Buffered prior KF clouds (for LTA / diagnostics). */
    std::size_t prior_cloud_count() const;

private:
    void WorkerLoop();
    void ProcessKeyFrame(KeyFrame kf);
    void AppendKeyFrameToPath(const KeyFrame& kf);
    double PathLengthBetween(int query_id, int match_id);
    bool TryCrossSessionLink(const LoopCandidate& cand, const KeyFrame& kf,
                             double confidence);
    void RecordAccepted(const LoopCandidate& cand, double confidence,
                        const transform::Rigid3d& T_pre,
                        bool apply_to_state = true,
                        bool clear_eval_drift = false);
    bool AcceptIntraSessionLoop(const LoopCandidate& cand, double confidence,
                                const transform::Rigid3d& T_pre);
    bool RefineCandidateGeometry(const KeyFrame& query_kf, LoopCandidate* cand);
    void NoteGeomFail(GeomFailReason reason);
    /**
     * @brief Buffer geom-ok candidate; return true when match-id cluster is
     *        confirmed and `*out` is the candidate to optimize.
     */
    bool ConfirmTemporal(const LoopCandidate& cand, double confidence,
                         const transform::Rigid3d& T_pre, LoopCandidate* out,
                         double* out_confidence, transform::Rigid3d* out_T_pre);
    /** @brief Odom-gap vs ICP co-location signature (pose-graph poses). */
    bool PassesRevisitSignature(const LoopCandidate& cand) const;
    void BufferLtaKeyFrame(const KeyFrame& kf);
    void BufferPriorKeyFrames(const std::string& session_id,
                              const std::vector<KeyFrame>& kfs);
    void EnqueueLtaClouds(const transform::Rigid3d& delta_w, int match_id,
                          int query_id);
    /** @brief Prior-session neighborhood → world → MapReloadWorker. */
    std::size_t EnqueuePriorLtaClouds(const std::string& session_id,
                                      int local_id);

    static std::string PriorCloudKey(const std::string& session_id,
                                     int local_id);

    LoopClosureConfig config_;
    ScanContextDetector detector_;
    MultiModalConsensus consensus_;
    backend::PoseGraph pose_graph_;
    map::MapSession sessions_;
    std::unique_ptr<CrossSessionLinker> cross_session_linker_;
    map::MapReloadWorker map_reload_;

    common::ThreadSafeQueue<KeyFrame> keyframe_queue_;
    std::thread worker_;
    std::atomic<bool> running_{false};
    std::atomic<bool> discard_next_keyframe_{false};

    std::mutex results_mutex_;
    std::vector<LoopCandidate> accepted_loops_;
    std::vector<LoopCorrection> corrections_;
    std::atomic<int> accepted_loop_count_{0};
    std::atomic<int> cross_session_link_count_{0};
    std::atomic<int> fpr_reject_count_{0};
    std::atomic<int> inject_restore_count_{0};
    std::atomic<int> geometric_reject_count_{0};
    std::atomic<int> geom_missing_cloud_{0};
    std::atomic<int> geom_empty_query_{0};
    std::atomic<int> geom_fitness_{0};
    std::atomic<int> geom_corr_{0};
    std::atomic<int> geom_translation_{0};
    std::atomic<int> geom_yaw_{0};
    std::atomic<int> geom_inlier_{0};
    std::atomic<int> geom_other_{0};
    std::atomic<int> temporal_reject_count_{0};
    std::atomic<int> temporal_pending_count_{0};
    std::atomic<int> signature_reject_count_{0};

    std::mutex path_mutex_;
    /** @brief Body clouds for ICP (id → cloud); capped with LTA buffer. */
    std::unordered_map<int, sensor::PointCloudXYZI> keyframe_clouds_;
    /** @brief Insertion order for LRU eviction of `keyframe_clouds_`. */
    std::deque<int> keyframe_cloud_order_;
    std::vector<Eigen::Vector3d> path_positions_;
    std::vector<int> path_keyframe_ids_;
    std::vector<std::string> path_session_ids_;

    mutable std::mutex lta_mutex_;
    std::deque<KeyFrame> lta_keyframes_;
    std::vector<LoopCandidate> pending_second_loop_;
    struct TemporalEntry {
        LoopCandidate cand;
        double confidence = 0.0;
        transform::Rigid3d T_pre = transform::Rigid3d::Identity();
    };
    std::deque<TemporalEntry> temporal_buffer_;
    /** @brief Body-frame clouds from prior sessions (for LTA reload). */
    std::unordered_map<std::string, sensor::PointCloudXYZI> prior_clouds_;
};

}  // namespace loop_closure
}  // namespace argus

#endif  // ARGUS_RELOCALIZATION_LOOP_MANAGER_HPP_
