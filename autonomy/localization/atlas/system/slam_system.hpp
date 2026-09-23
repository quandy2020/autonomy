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
 * @file slam_system.hpp
 * @brief Atlas SLAM top-level facade (ORB-SLAM3 System aligned, autolink scheduling).
 *
 * Threading: Tracking on the caller thread; LocalMapping / LoopClosing run as
 * long-lived tasks on `SlamScheduler`; keyframes and IMU hand off via
 * ThreadSafeQueue.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_SYSTEM_SLAM_SYSTEM_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_SYSTEM_SLAM_SYSTEM_HPP_

#include <memory>
#include <string>

#include <opencv2/core/mat.hpp>

#include "autonomy/localization/atlas/backend/loop_closing.hpp"
#include "autonomy/localization/atlas/common/config.hpp"
#include "autonomy/localization/atlas/common/types.hpp"
#include "autonomy/localization/atlas/frontend/tracking/tracker.hpp"
#include "autonomy/localization/atlas/map/local_mapping.hpp"
#include "autonomy/localization/atlas/map/map.hpp"
#include "autonomy/localization/atlas/map/map_manager.hpp"
#include "autonomy/localization/atlas/map/multi_map.hpp"
#include "autonomy/localization/atlas/sensor/imu/types.hpp"
#include "autonomy/localization/atlas/system/slam_scheduler.hpp"
#include "autonomy/localization/atlas/system/slam_visualizer.hpp"

#include "autolink/node/node.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

/**
 * @class autonomy::localization::atlas::SlamSystem
 * @brief SLAM system entry: config, tracking, backend, visualization, Atlas I/O.
 *
 * **Typical call order**
 * 1. `Init` → `StartVisualization` (optional) → loop `Track*` / `GrabImuData`
 * 2. `FlushPendingKeyframes` / `SaveAtlas` when needed
 * 3. `Shutdown` to stop backend and release resources
 *
 * **Scheduling**
 * - Tracking: caller thread
 * - LocalMapping / LoopClosing: long tasks on `SlamScheduler` ThreadPool
 * - Keyframe handoff: `ThreadSafeQueue`
 */
class SlamSystem {
public:
    /**
     * @enum Sensor
     * @brief Sensor modality (drives Tracker init and whether IMU is enabled).
     */
    enum class Sensor {
        kMonocular = 0,  ///< Monocular
        kStereo,         ///< Stereo
        kRgbd,           ///< RGB-D
        kImuMonocular,   ///< Monocular + IMU
        kImuStereo,      ///< Stereo + IMU
        kImuRgbd,        ///< RGB-D + IMU
    };

    /**
     * @brief Initialize Tracker / MultiMap / LoopClosing / Scheduler from config.
     * @param config Atlas YAML config.
     * @param sensor Sensor modality (default RGB-D).
     * @return true on success.
     */
    bool Init(const AtlasConfig& config, Sensor sensor = Sensor::kRgbd);

    /**
     * @brief Request backend stop and wait until finished.
     */
    void Shutdown();

    /**
     * @brief One stereo tracking step.
     * @param left / right Left and right images.
     * @param timestamp Image timestamp (seconds).
     * @return Estimated body/camera pose \(T_{wb}\) or \(T_{cw}\) (Tracker convention).
     */
    SE3 TrackStereo(const cv::Mat& left, const cv::Mat& right, double timestamp);

    /**
     * @brief One RGB-D tracking step.
     * @param rgb / depth Color and depth images.
     * @param timestamp Timestamp (seconds).
     * @return Current pose.
     */
    SE3 TrackRgbd(const cv::Mat& rgb, const cv::Mat& depth, double timestamp);

    /**
     * @brief One monocular tracking step.
     * @param image Gray or color image.
     * @param timestamp Timestamp (seconds).
     * @return Current pose (scale may be unfixed).
     */
    SE3 TrackMonocular(const cv::Mat& image, double timestamp);

    /**
     * @brief Inject one IMU measurement (forwards to Tracker buffer).
     * @param measurement Accel + gyro + timestamp.
     */
    void GrabImuData(const sensor::imu::Measurement& measurement);

    /**
     * @brief Set IMU↔camera extrinsics and noise (forwards to Tracker).
     * @param calib IMU calibration.
     */
    void SetImuCalib(const sensor::imu::Calib& calib);

    /**
     * @brief Serialize the current MultiMap / Atlas subset to JSON.
     * @param path Output path.
     * @return true if write succeeded.
     */
    bool SaveAtlas(const std::string& path) const;

    /**
     * @brief Load Atlas from JSON (ORB File.loadAtlasFrom subset).
     * @param path Input path.
     * @return true if read succeeded.
     */
    bool LoadAtlas(const std::string& path);

    /**
     * @brief Fetch the latest odometry result (pose + local points, etc.).
     * @param[out] out Output struct; contents undefined on failure.
     * @return Whether a valid result is available.
     */
    bool GetOdometry(OdometryResult* out) const;

    /** @brief Current active Map (mutable). */
    Map* mutable_map() { return tracker_.mutable_map(); }
    /** @brief MultiMap container (mutable). */
    MultiMap* mutable_multi_map() { return multi_map_.get(); }
    /** @brief Create and switch to a new Map (multi-map session). */
    void CreateNewMap();

    /**
     * @brief Switch to localization-only tracking (ORB ActivateLocalizationMode).
     *
     * Applied on the next `Track*`: LocalMapping pauses and no new keyframes
     * are inserted.
     */
    void ActivateLocalizationMode();
    /**
     * @brief Resume mapping (ORB DeactivateLocalizationMode).
     */
    void DeactivateLocalizationMode();
    /**
     * @brief Start a new sequence (ORB ChangeDataset).
     *
     * Fewer than 12 keyframes resets the active map. Otherwise a new map is
     * created in the atlas and backend workers are restarted.
     */
    void ChangeDataset();
    /**
     * @brief Write IMU-init debug files (ORB SaveDebugData).
     *
     * Writes scale, gravity, biases, covariance, timing, and a keyframe
     * trajectory next to the process working directory.
     * @param init_idx Suffix used in the trajectory and covariance filenames.
     */
    void SaveDebugData(int init_idx);

    /**
     * @brief Write the frame trajectory in TUM format.
     *
     * Pure monocular is skipped. Lost frames are omitted. Poses are expressed
     * relative to the first keyframe after loop correction.
     * @param filename Output path.
     */
    void SaveTrajectoryTUM(const std::string& filename);
    /**
     * @brief Write keyframe poses in TUM format.
     * @param filename Output path.
     */
    void SaveKeyFrameTrajectoryTUM(const std::string& filename);
    /**
     * @brief Write the frame trajectory of the largest map in EuRoC format.
     * @param filename Output path. Timestamps are nanoseconds.
     */
    void SaveTrajectoryEuRoC(const std::string& filename);
    /**
     * @brief Write one map's frame trajectory in EuRoC format.
     * @param filename Output path.
     * @param map Map to export. Null selects the largest map.
     */
    void SaveTrajectoryEuRoC(const std::string& filename, Map* map);
    /**
     * @brief Write keyframe poses of the largest map in EuRoC format.
     * @param filename Output path.
     */
    void SaveKeyFrameTrajectoryEuRoC(const std::string& filename);
    /**
     * @brief Write one map's keyframe poses in EuRoC format.
     * @param filename Output path.
     * @param map Map to export. Null selects the largest map.
     */
    void SaveKeyFrameTrajectoryEuRoC(const std::string& filename, Map* map);
    /**
     * @brief Write the frame trajectory as a KITTI 3×4 pose file.
     *
     * Pure monocular is skipped.
     * @param filename Output path.
     */
    void SaveTrajectoryKITTI(const std::string& filename);

    /**
     * @brief Whether a loop, merge, or bundle adjustment changed the map.
     * @return true once per change index.
     */
    bool MapChanged();
    /**
     * @brief Tracking state as an integer (ORB `eTrackingState`).
     * @return `Tracker::State` value.
     */
    int GetTrackingState() const;
    /**
     * @brief Lost after IMU initialization.
     * @return true when inertial tracking is in the lost state.
     */
    bool isLost();
    /**
     * @brief Whether IMU initialization has been running for more than 0.1 s.
     * @return ORB `isFinished` predicate.
     */
    bool isFinished();
    /**
     * @brief Seconds since the first IMU keyframe, after initialization.
     * @return 0 before IMU initialization.
     */
    double GetTimeFromIMUInit();
    /** @brief Clear the atlas on the next `Track*` (ORB `Reset`). */
    void Reset();
    /** @brief Clear the active map on the next `Track*` (ORB `ResetActiveMap`). */
    void ResetActiveMap();
    /**
     * @brief Reload camera intrinsics from a settings file.
     * @param settings_path Atlas YAML.
     */
    void ChangeCalibration(const std::string& settings_path);

    /** @brief LocalMapping pointer. */
    LocalMapping* mutable_local_mapping() {
        return tracker_.mutable_local_mapping();
    }
    /** @brief LoopClosing pointer. */
    backend::LoopClosing* mutable_loop_closing() {
        return loop_closing_.get();
    }
    /** @brief Scheduler pointer. */
    SlamScheduler* mutable_scheduler() { return scheduler_.get(); }
    /** @brief Visualizer reference. */
    SlamVisualizer* mutable_visualizer() { return &visualizer_; }

    /**
     * @brief Bind an Autolink Node and start visualization publishers.
     * @param node Autolink node.
     * @param options Topic toggles and rate-limit parameters.
     */
    void StartVisualization(const std::shared_ptr<autolink::Node>& node,
                            const SlamVisualizer::Options& options =
                                SlamVisualizer::Options{});

    /**
     * @brief Flush frontend-pending system keyframes into MapManager (not LocalMapping queue).
     */
    void FlushPendingKeyframes();

private:
    /** @brief Start LocalMapping / LoopClosing workers. */
    void StartBackendWorkers();
    /** @brief Rebuild LoopClosing and restart workers on the active map. */
    void RebuildBackendForActiveMap();
    /**
     * @brief Publish visualization from the latest tracking result.
     * @param timestamp_sec Image timestamp.
     */
    void PublishVisualization(double timestamp_sec);
    /** @brief Apply a pending localization-mode request before tracking. */
    void ApplyModeChange();

    tracking::Tracker tracker_;                          ///< Frontend tracking
    MapManager map_manager_;                             ///< System-side KF/landmark cache
    std::shared_ptr<MultiMap> multi_map_;                ///< Multi-map
    std::unique_ptr<SlamScheduler> scheduler_;           ///< Backend thread pool
    std::unique_ptr<backend::LoopClosing> loop_closing_; ///< Loop / merge
    SlamVisualizer visualizer_;                          ///< RViz channels
    Sensor sensor_ = Sensor::kRgbd;                      ///< Current modality
    AtlasConfig config_;                                 ///< Runtime config
    bool activate_localization_ = false;                 ///< Pending localization-only
    bool deactivate_localization_ = false;               ///< Pending mapping resume
    bool reset_ = false;                                 ///< Pending full reset
    bool reset_active_map_ = false;                      ///< Pending active-map reset
    int last_map_change_idx_ = 0;                        ///< Last `MapChanged` index
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_SYSTEM_SLAM_SYSTEM_HPP_
