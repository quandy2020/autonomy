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
 * @file loop_closing.hpp
 * @brief Loop detection/correction and map merge (ORB-SLAM3 LoopClosing;
 *        queue + thread-pool worker).
 *
 * Flow: `InsertKeyFrame` → worker `Run` → `NewDetectCommonRegions`
 * (BoW + coincidence count) → `CorrectLoop` / `MergeLocal` → optional async GBA.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_BACKEND_LOOP_CLOSING_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_BACKEND_LOOP_CLOSING_HPP_

#include <atomic>
#include <future>
#include <memory>
#include <vector>

#include "autolink/base/thread_safe_queue.hpp"

#include "autonomy/localization/atlas/backend/sim3.hpp"
#include "autonomy/localization/atlas/map/keyframe.hpp"
#include "autonomy/localization/atlas/map/keyframe_database.hpp"
#include "autonomy/localization/atlas/map/map.hpp"
#include "autonomy/localization/atlas/map/map_point.hpp"
#include "autonomy/localization/atlas/map/multi_map.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

class SlamScheduler;
class LocalMapping;

namespace backend {

/**
 * @class autonomy::localization::atlas::backend::LoopClosing
 * @brief Place recognition + Sim3 loop correction / cross-map merge.
 *
 * **Threading**: `Start(scheduler)` runs `Run` on the ThreadPool, blocking on
 * `keyframe_queue_` for keyframes; exits after `RequestFinish`.
 *
 * **With LocalMapping**: `PauseLocalMapping` before correction,
 * `ResumeLocalMapping` after; optional async `LaunchGlobalBA`.
 */
class LoopClosing {
public:
    /**
     * @brief Construct the loop-closing module.
     * @param map Current active map (non-owning).
     * @param keyframe_database BoW database (non-owning).
     * @param multi_map Multi-map container; needed for merge path, may be null.
     */
    LoopClosing(Map* map, KeyFrameDatabase* keyframe_database,
                 MultiMap* multi_map = nullptr);

    /**
     * @brief Enqueue a new keyframe (called by LocalMapping).
     * @param keyframe New keyframe.
     */
    void InsertKeyFrame(const std::shared_ptr<KeyFrame>& keyframe);

    /**
     * @brief Detect loop candidates (public / test entry).
     * @param[out] candidates Candidate keyframe list.
     * @return true if candidates were found.
     */
    bool DetectLoop(std::vector<std::shared_ptr<KeyFrame>>* candidates);

    /**
     * @brief Correct against a given loop keyframe (Sim3 + Essential Graph + fuse).
     * @param loop_keyframe Loop-matched keyframe.
     * @return true if correction succeeded.
     */
    bool CorrectLoop(const std::shared_ptr<KeyFrame>& loop_keyframe);

    /**
     * @brief Merge the map of `merge_keyframe` into the current map via Sim3.
     * @param merge_keyframe Merge-side keyframe.
     * @param sim3_current_from_merge Similarity current ← merge side.
     * @return true if merge succeeded.
     */
    bool MergeLocal(const std::shared_ptr<KeyFrame>& merge_keyframe,
                    const Sim3& sim3_current_from_merge);

    /**
     * @brief Inertial map merge (ORB `MergeLocal2`).
     *
     * Scales the current map into the merge frame, rewires the spanning tree,
     * then runs welding inertial BA.
     * @param merge_keyframe Merge-side keyframe.
     * @param sim3_current_from_merge Similarity current-camera ← merge world.
     * @return false when the scale check rejects the merge.
     */
    bool MergeLocal2(const std::shared_ptr<KeyFrame>& merge_keyframe,
                     const Sim3& sim3_current_from_merge);

    /**
     * @brief Write a finished global BA through the spanning tree.
     * @param map Active map.
     * @param gba_id Epoch passed into `GlobalBundleAdjustment` / `FullInertialBA`.
     */
    void PropagateGlobalBA(Map* map, uint64_t gba_id);

    /**
     * @brief Start the worker on a scheduler.
     * @param scheduler Non-null ThreadPool scheduler.
     */
    void Start(SlamScheduler* scheduler);

    /** @brief Request the worker loop to finish. */
    void RequestFinish();

    /**
     * @brief Whether the worker has fully exited.
     * @return true if finished.
     */
    bool isFinished() const { return finished_.load(); }

    /**
     * @brief Whether running as an async worker.
     * @return true if Started and not yet finished.
     */
    bool isAsync() const { return async_.load(); }

    /**
     * @brief Set whether Sim3 scale is fixed.
     * @param fix_scale Should be true for stereo/RGB-D.
     */
    void set_fix_scale(bool fix_scale) { fix_scale_ = fix_scale; }

    /**
     * @brief Bind MultiMap (for merge).
     * @param multi_map Multi-map pointer.
     */
    void SetMultiMap(MultiMap* multi_map) { multi_map_ = multi_map; }

    /**
     * @brief Bind LocalMapping (pause/resume).
     * @param local_mapping Local mapping pointer.
     */
    void SetLocalMapper(LocalMapping* local_mapping) {
        local_mapping_ = local_mapping;
    }

    /**
     * @brief Most recently successfully estimated Sim3.
     * @return Copied Sim3.
     */
    Sim3 last_sim3() const { return last_sim3_; }

private:
    /** @brief Worker main loop: dequeue → ProcessKeyFrame. */
    void Run();
    /**
     * @brief Process one keyframe: detect common region → loop or merge.
     * @param keyframe Current keyframe.
     * @return true if this frame triggered correction/merge.
     */
    bool ProcessKeyFrame(const std::shared_ptr<KeyFrame>& keyframe);

    /** @brief Pause LocalMapping, drain its queue, wait until stopped (ORB CorrectLoop prelude). */
    void PauseLocalMapping();
    /** @brief Resume LocalMapping. */
    void ResumeLocalMapping();

    /**
     * @brief Async global BA after loop/merge (ORB RunGlobalBundleAdjustment).
     * @param active_map Active map.
     * @param iterations GBA iteration count.
     */
    void LaunchGlobalBA(Map* active_map, int iterations = 10);

    /**
     * @brief ORB NewDetectCommonRegions: coincidence count + BoW.
     * @param[out] matched_out Matched keyframe.
     * @param[out] sim3_out Estimated Sim3 (\(S_{cw}\) semantics in the implementation).
     * @param[out] is_merge true for a cross-map merge candidate.
     * @param[out] matched_mps_out Optional matched map points.
     * @return true if a common region was confirmed.
     */
    bool NewDetectCommonRegions(std::shared_ptr<KeyFrame>* matched_out,
                                Sim3* sim3_out, bool* is_merge,
                                std::vector<std::shared_ptr<MapPoint>>*
                                    matched_mps_out = nullptr);

    /**
     * @brief Detect and refine covisibility / Sim3 from BoW candidates.
     * @param bow_candidates BoW retrieval candidates.
     * @param[out] matched_out Best matched keyframe.
     * @param[out] Scw_out World←matched-camera Sim3 or implementation form.
     * @param[out] num_coincidences Coincidence count.
     * @param[out] matched_mps_out Matched points.
     * @return true on success.
     */
    bool DetectCommonRegionsFromBoW(
        const std::vector<std::shared_ptr<KeyFrame>>& bow_candidates,
        std::shared_ptr<KeyFrame>* matched_out, Sim3* Scw_out,
        int* num_coincidences,
        std::vector<std::shared_ptr<MapPoint>>* matched_mps_out = nullptr);

    /**
     * @brief Project relative to the last matched keyframe and refine Sim3.
     * @param current Current keyframe.
     * @param matched Last matched keyframe.
     * @param[in,out] Scw Sim3.
     * @param[out] num_proj_matches Projection match count.
     * @param[out] matched_mps Matched map points.
     * @return true if refinement succeeded.
     */
    bool DetectAndRefineSim3FromLastKF(
        const std::shared_ptr<KeyFrame>& current,
        const std::shared_ptr<KeyFrame>& matched, Sim3* Scw,
        int* num_proj_matches,
        std::vector<std::shared_ptr<MapPoint>>* matched_mps);

    /**
     * @brief Search matches by projection under a given Sim3.
     * @param current / matched Keyframe pair.
     * @param Scw Similarity transform.
     * @param[in,out] map_points Map points participating in the search.
     * @param[out] matched_mps Match results.
     * @return Match count.
     */
    int FindMatchesByProjection(
        const std::shared_ptr<KeyFrame>& current,
        const std::shared_ptr<KeyFrame>& matched, const Sim3& Scw,
        std::vector<std::shared_ptr<MapPoint>>* map_points,
        std::vector<std::shared_ptr<MapPoint>>* matched_mps);

    /**
     * @brief Estimate relative Sim3 via RANSAC + OptimizeSim3.
     * @param matched_keyframe Matched keyframe.
     * @param[out] sim3_out Result.
     * @return true on success.
     */
    bool EstimateSim3(const std::shared_ptr<KeyFrame>& matched_keyframe,
                      Sim3* sim3_out);

    /**
     * @brief Apply loop: pose propagation, point fuse, Essential Graph.
     * @param loop_keyframe Loop keyframe.
     * @param sim3 Current ← loop Sim3.
     * @param matched_mps Matched map points (for fuse).
     * @return true on success.
     */
    bool ApplyLoopCorrection(
        const std::shared_ptr<KeyFrame>& loop_keyframe, const Sim3& sim3,
        const std::vector<std::shared_ptr<MapPoint>>& matched_mps = {});

    /**
     * @brief Project-fuse map points on connected keyframes (ORB SearchAndFuse).
     * @param connected Connected keyframes.
     * @param map_points Points to fuse.
     */
    void SearchAndFuse(
        const std::vector<std::shared_ptr<KeyFrame>>& connected,
        const std::vector<std::shared_ptr<MapPoint>>& map_points);

    /** @brief Reset loop coincidence state machine. */
    void ResetLoopState();
    /** @brief Reset merge coincidence state machine. */
    void ResetMergeState();

    /**
     * @brief From \(S_{cw}\) and matched KF pose, get "current ← matched" Sim3.
     * @param Scw World-related Sim3.
     * @param matched Matched keyframe.
     * @return Relative Sim3.
     */
    Sim3 Sim3CurrentFromMatched(const Sim3& Scw,
                                              const std::shared_ptr<KeyFrame>& matched)
        const;

    Map* map_ = nullptr;                          ///< Active map
    MultiMap* multi_map_ = nullptr;               ///< Multi-map
    KeyFrameDatabase* keyframe_database_ = nullptr;  ///< BoW database
    LocalMapping* local_mapping_ = nullptr;       ///< Local mapping
    SlamScheduler* scheduler_ = nullptr;          ///< Scheduler
    std::shared_ptr<KeyFrame> current_keyframe_;  ///< Keyframe being processed
    Sim3 last_sim3_;                              ///< Latest Sim3
    float min_score_ = 0.05f;                     ///< BoW minimum score
    bool fix_scale_ = true;                       ///< Fix Sim3 scale

    std::atomic<bool> running_gba_{false};        ///< Whether GBA is running
    std::atomic<uint64_t> gba_epoch_{0};          ///< GBA generation id
    std::shared_ptr<bool> stop_gba_flag_;         ///< GBA stop flag
    std::future<void> gba_future_;                ///< GBA future

    // ORB-SLAM3 loop coincidence state
    int loop_num_coincidences_ = 0;               ///< Consecutive coincidence count
    int loop_num_not_found_ = 0;                  ///< Consecutive not-found count
    std::shared_ptr<KeyFrame> loop_matched_kf_;   ///< Loop matched KF
    std::shared_ptr<KeyFrame> loop_last_current_kf_;  ///< Previous current KF
    Sim3 loop_sim3_slw_;                          ///< Loop Sim3 state
    std::vector<std::shared_ptr<MapPoint>> loop_map_points_;  ///< Candidate points
    std::vector<std::shared_ptr<MapPoint>> loop_matched_mps_;  ///< Matched points

    // ORB-SLAM3 merge coincidence state
    int merge_num_coincidences_ = 0;
    int merge_num_not_found_ = 0;
    std::shared_ptr<KeyFrame> merge_matched_kf_;
    std::shared_ptr<KeyFrame> merge_last_current_kf_;
    Sim3 merge_sim3_slw_;

    ::autolink::base::ThreadSafeQueue<std::shared_ptr<KeyFrame>> keyframe_queue_;  ///< Enqueued keyframes
    std::atomic<bool> finish_requested_{false};  ///< Finish request
    std::atomic<bool> finished_{false};          ///< Finished
    std::atomic<bool> async_{false};             ///< In async worker
    std::future<void> worker_future_;            ///< Worker future
};

}  // namespace backend
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_BACKEND_LOOP_CLOSING_HPP_
