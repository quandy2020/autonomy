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
 * LocalMapping: ThreadSafeQueue + optional ThreadPool worker (ORB-SLAM3).
 */

/**
 * @file local_mapping.hpp
 * @brief Local mapping thread: keyframe processing, triangulation, fuse, cull.
 *
 * Corresponds to ORB-SLAM3 `LocalMapping`. Keyframes enter a
 * `ThreadSafeQueue` via `InsertKeyFrame`; after `Start` on `SlamScheduler`,
 * a pool thread blocks and consumes.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MAP_LOCAL_MAPPING_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MAP_LOCAL_MAPPING_HPP_

#include <atomic>
#include <future>
#include <list>
#include <memory>

#include "Eigen/Core"

#include "autolink/base/thread_safe_queue.hpp"

#include "autonomy/localization/atlas/map/keyframe.hpp"
#include "autonomy/localization/atlas/map/map.hpp"
#include "autonomy/localization/atlas/map/map_point.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

namespace backend {
class LoopClosing;
}  // namespace backend

namespace tracking {
class Tracker;
}  // namespace tracking

class SlamScheduler;

/**
 * @class autonomy::localization::atlas::LocalMapping
 * @brief Process new keyframes: covisibility, cull/create points, neighbor fuse, local BA.
 *
 * Corresponds to ORB-SLAM3 `LocalMapping`. Keyframes enqueue via
 * `InsertKeyFrame`; after `Start(scheduler)`, `Run` blocks on `WaitDequeue`.
 * Sync tests can use `ProcessNextKeyFrame`.
 *
 * @note Async start
 * @code{.cpp}
 * LocalMapping mapping(map, false);
 * mapping.SetLoopCloser(loop);
 * mapping.SetTracker(tracker);
 * mapping.Start(scheduler);
 * mapping.InsertKeyFrame(kf);
 * @endcode
 */
class LocalMapping {
public:
    /**
     * @brief Construct bound to an active map.
     * @param map Active map.
     * @param monocular Whether monocular (affects cull thresholds, etc.).
     */
    explicit LocalMapping(Map* map, bool monocular = false);

    /**
     * @brief Inject loop-closing module (forward after new KF processing).
     * @param loop_closing Loop-closing pointer.
     */
    void SetLoopCloser(backend::LoopClosing* loop_closing);
    /**
     * @brief Inject Tracking (IMU init and related callbacks).
     * @param tracker Tracker.
     */
    void SetTracker(tracking::Tracker* tracker) { tracker_ = tracker; }
    /**
     * @brief Switch the active map pointer.
     * @param map New map.
     */
    void SetMap(Map* map) { map_ = map; }

    /**
     * @brief Enqueue a new keyframe.
     * @param keyframe Keyframe.
     */
    void InsertKeyFrame(const std::shared_ptr<KeyFrame>& keyframe);
    /**
     * @brief Non-blocking: process at most one queued frame (test / sync mode).
     * @return `true` if a frame was processed.
     */
    bool ProcessNextKeyFrame();
    /**
     * @brief Drain queue with only `ProcessNewKeyFrame` (ORB `EmptyQueue`; often before loop).
     */
    void EmptyQueue();
    /**
     * @brief Number of keyframes waiting in the queue.
     * @return Count.
     */
    int KeyframesInQueue() const;
    /**
     * @brief Whether the queue is empty enough for a new KF (ORB `AcceptKeyFrames`).
     * @return `true` when queue length < 3.
     */
    bool AcceptKeyFrames() const {
        return KeyframesInQueue() < 3;
    }

    /**
     * @brief Request abort of local BA (when Tracking inserts a KF; ORB `InterruptBA`).
     */
    void InterruptBA() { abort_ba_.store(true); }
    /**
     * @brief Whether BA abort was requested.
     * @return Atomic flag.
     */
    bool AbortRequested() const { return abort_ba_.load(); }

    /**
     * @brief Start long-lived `Run()` on @p scheduler (uses ≥1 pool slot).
     * @param scheduler SLAM scheduler.
     */
    void Start(SlamScheduler* scheduler);
    /**
     * @brief Soft pause (ORB `RequestStop`): idle until `Release`; do not kill the worker.
     */
    void RequestPause();
    /** @brief Clear soft pause and resume processing. */
    void Release();
    /**
     * @brief Refuse pause while Tracking creates a KF (ORB `SetNotStop`).
     * @param flag `true` to refuse pause.
     * @return Whether the setting was accepted (ORB return semantics).
     */
    bool SetNotStop(bool flag);
    /**
     * @brief Hard stop: break queue wait and join the worker (reset / shutdown).
     */
    void RequestStop();
    /**
     * @brief Whether the worker has stopped.
     * @return Flag.
     */
    bool isStopped() const { return stopped_.load(); }
    /**
     * @brief Whether running in async mode.
     * @return Flag.
     */
    bool isAsync() const { return async_.load(); }
    /**
     * @brief Whether a stop/pause was requested.
     * @return `stop_requested_ || pause_requested_`.
     */
    bool stopRequested() const {
        return stop_requested_.load() || pause_requested_.load();
    }
    /**
     * @brief Whether IMU initialization is in progress.
     * @return Flag.
     */
    bool IsInitializing() const { return initializing_.load(); }

    /**
     * @brief Switch monocular mode flag.
     * @param monocular Whether monocular.
     */
    void SetMonocular(bool monocular) { monocular_ = monocular; }

    /**
     * @brief Set far-point threshold (ORB `mbFarPoints` / `mThFarPoints`).
     * @param th_meters Distance threshold (m); ≤0 disables far-point discard.
     */
    void SetFarPoints(float th_meters) {
        th_far_points_ = th_meters;
        far_points_ = th_meters > 0.f;
    }
    /**
     * @brief Whether far-point discard is enabled.
     * @return Flag.
     */
    bool far_points() const { return far_points_; }
    /**
     * @brief Far-point distance threshold (m).
     * @return Threshold.
     */
    float th_far_points() const { return th_far_points_; }

    /**
     * @brief IMU motion was too small to initialize (ORB `mbBadImu`).
     * @return true until tracking resets the active map.
     */
    bool bad_imu() const { return bad_imu_.load(); }

    /**
     * @brief IMU initialization (gravity / scale / bias); optional FullInertialBA.
     * @param prior_g Gravity prior weight.
     * @param prior_a Accelerometer bias prior.
     * @param run_full_inertial_ba Whether to run full inertial BA (ORB `b_fiba`).
     *
     * Corresponds to ORB-SLAM3 `InitializeIMU`.
     */
    void InitializeImu(float prior_g = 1e2f, float prior_a = 1e6f,
                       bool run_full_inertial_ba = true);
    /**
     * @brief Periodic monocular scale refinement (ORB `ScaleRefinement`).
     */
    void ScaleRefinement();

    /** @brief Gravity direction from the latest inertial initialization. */
    const Mat33& gravity_rotation() const { return gravity_rotation_; }
    /** @brief Scale from the latest inertial initialization. */
    double imu_scale() const { return imu_scale_; }
    /** @brief Gyro bias from the latest inertial initialization. */
    const Vec3& gyro_bias() const { return gyro_bias_; }
    /** @brief Accelerometer bias from the latest inertial initialization. */
    const Vec3& acc_bias() const { return acc_bias_; }
    /** @brief Covariance of the inertial initialization blocks. */
    const Eigen::MatrixXd& inertial_covariance() const {
        return inertial_covariance_;
    }
    /** @brief Initialization section counter (ORB `mInitSect`). */
    int init_section() const { return init_section_; }
    /** @brief Seconds from the first IMU keyframe to initialization. */
    double init_time_sec() const { return init_time_sec_; }
    /** @brief Wall time of the last inertial initialization, in seconds. */
    double init_cost_sec() const { return init_cost_sec_; }
    /** @brief Timestamp of the keyframe currently being processed. */
    double current_keyframe_timestamp() const {
        return current_keyframe_ ? current_keyframe_->timestamp : -1.0;
    }
    /** @brief Timestamp of the first keyframe in the IMU chain. */
    double first_imu_keyframe_timestamp() const {
        return first_imu_keyframe_ts_;
    }

private:
    /** @brief Worker-thread main loop. */
    void Run();
    /**
     * @brief Full per-frame pipeline (process → cull → triangulate → fuse → cull KF).
     * @param keyframe Current keyframe.
     */
    void ProcessPipeline(const std::shared_ptr<KeyFrame>& keyframe);
    /**
     * @brief Ingest new keyframe: BoW, covisibility, observation links.
     * @param keyframe Keyframe.
     */
    void ProcessNewKeyFrame(const std::shared_ptr<KeyFrame>& keyframe);
    /** @brief Cull recently created map points. */
    void MapPointCulling();
    /** @brief Triangulate new map points with neighbors. */
    void CreateNewMapPoints();
    /** @brief Neighbor projection fuse (ORB `SearchInNeighbors`). */
    void SearchInNeighbors();
    /** @brief Cull redundant keyframes. */
    void KeyFrameCulling();

    Map* map_ = nullptr;                              ///< Active map
    backend::LoopClosing* loop_closing_ = nullptr;    ///< Loop closing
    tracking::Tracker* tracker_ = nullptr;              ///< Tracking
    bool monocular_ = false;                            ///< Monocular mode
    bool far_points_ = false;                           ///< Far-point discard toggle
    float th_far_points_ = 0.f;                         ///< Far-point threshold (m)

    ::autolink::base::ThreadSafeQueue<std::shared_ptr<KeyFrame>>
        keyframe_queue_;                                ///< KF queue
    std::list<std::shared_ptr<MapPoint>> recent_map_points_;  ///< Recent new points
    std::shared_ptr<KeyFrame> current_keyframe_;        ///< KF currently processed

    double imu_init_timestamp_ = -1.0;      ///< IMU init time marker
    double first_imu_keyframe_ts_ = -1.0;   ///< First IMU KF timestamp
    int last_scale_refine_slot_ = -1;       ///< Last scale-refinement slot
    Mat33 gravity_rotation_ = Mat33::Identity();  ///< Latest \(R_{wg}\)
    double imu_scale_ = 1.0;                      ///< Latest scale
    Vec3 gyro_bias_ = Vec3::Zero();               ///< Latest gyro bias
    Vec3 acc_bias_ = Vec3::Zero();                ///< Latest accelerometer bias
    Eigen::MatrixXd inertial_covariance_;         ///< Init covariance
    int init_section_ = 0;                        ///< Init section id
    double init_time_sec_ = 0.0;                  ///< Init span [s]
    double init_cost_sec_ = 0.0;                  ///< Init wall time [s]
    double init_motion_time_ = 0.0;               ///< Accumulated init motion [s]
    std::atomic<bool> bad_imu_{false};            ///< Reset active map from tracking

    std::atomic<bool> stop_requested_{false};   ///< Hard-stop request
    std::atomic<bool> pause_requested_{false};  ///< Soft-pause request
    std::atomic<bool> not_stop_{false};         ///< Refuse pause
    std::atomic<bool> stopped_{false};          ///< Stopped
    std::atomic<bool> async_{false};            ///< Async running
    std::atomic<bool> initializing_{false};     ///< IMU initializing
    std::atomic<bool> abort_ba_{false};         ///< Abort local BA
    std::future<void> worker_future_;           ///< Worker future
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MAP_LOCAL_MAPPING_HPP_
