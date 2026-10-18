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
 * @file loop_closure_manager.hpp
 * @brief Async loop-closure worker (never runs on ESIKF thread).
 */

#ifndef ARGUS_LOOP_CLOSURE_LOOP_CLOSURE_MANAGER_HPP_
#define ARGUS_LOOP_CLOSURE_LOOP_CLOSURE_MANAGER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "argus/backend/pose_graph/pose_graph.hpp"
#include "argus/common/macros.hpp"
#include "argus/common/thread_safe_queue.hpp"
#include "argus/loop_closure/cross_session_linker.hpp"
#include "argus/loop_closure/loop_detector_base.hpp"
#include "argus/loop_closure/multi_modal_consensus.hpp"
#include "argus/loop_closure/scan_context_detector.hpp"
#include "argus/map/map_session.hpp"

namespace argus {
namespace loop_closure {

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

private:
    void WorkerLoop();
    void ProcessKeyFrame(KeyFrame kf);
    void AppendKeyFrameToPath(const KeyFrame& kf);
    double PathLengthBetween(int query_id, int match_id);
    bool TryCrossSessionLink(const LoopCandidate& cand, const KeyFrame& kf,
                             double confidence);
    void RecordAccepted(const LoopCandidate& cand, double confidence,
                        const transform::Rigid3d& T_pre);

    LoopClosureConfig config_;
    ScanContextDetector detector_;
    MultiModalConsensus consensus_;
    backend::PoseGraph pose_graph_;
    map::MapSession sessions_;
    std::unique_ptr<CrossSessionLinker> cross_session_linker_;

    common::ThreadSafeQueue<KeyFrame> keyframe_queue_;
    std::thread worker_;
    std::atomic<bool> running_{false};

    std::mutex results_mutex_;
    std::vector<LoopCandidate> accepted_loops_;
    std::vector<LoopCorrection> corrections_;
    std::atomic<int> accepted_loop_count_{0};
    std::atomic<int> cross_session_link_count_{0};

    std::mutex path_mutex_;
    std::vector<Eigen::Vector3d> path_positions_;
    std::vector<int> path_keyframe_ids_;
    std::vector<std::string> path_session_ids_;
};

}  // namespace loop_closure
}  // namespace argus

#endif  // ARGUS_LOOP_CLOSURE_LOOP_CLOSURE_MANAGER_HPP_
