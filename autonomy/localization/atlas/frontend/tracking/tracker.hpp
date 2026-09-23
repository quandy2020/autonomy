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
 * Tracking flow adapted from ORB-SLAM3 Tracking (GrabImage* → Track).
 */

/**
 * @file tracker.hpp
 * @brief ORB-SLAM3-style tracking main class: image grab, motion/local-map tracking, keyframes, IMU preintegration.
 *
 * Pipeline aligned with ORB-SLAM3 Tracking (GrabImage* → Track). Wrapped as FrontendBase by
 * VisualOdometry / VisualInertial; GrabImage* / GrabImuData may also be called directly.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_TRACKING_TRACKER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_TRACKING_TRACKER_HPP_

#include <deque>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "autolink/base/thread_safe_queue.hpp"

#include "autonomy/localization/atlas/common/config.hpp"
#include "autonomy/localization/atlas/common/types.hpp"
#include "autonomy/localization/atlas/frontend/feature/orb/orb_extractor.hpp"
#include "autonomy/localization/atlas/frontend/feature/orb/orb_params.hpp"
#include "autonomy/localization/atlas/frontend/feature/orb/orb_vocabulary.hpp"
#include "autonomy/localization/atlas/frontend/tracking/frame.hpp"
#include "autonomy/localization/atlas/map/keyframe.hpp"
#include "autonomy/localization/atlas/map/keyframe_database.hpp"
#include "autonomy/localization/atlas/map/local_mapping.hpp"
#include "autonomy/localization/atlas/map/map.hpp"
#include "autonomy/localization/atlas/map/map_point.hpp"
#include "autonomy/localization/atlas/map/multi_map.hpp"
#include "autonomy/localization/atlas/sensor/geometric_camera.hpp"
#include "autonomy/localization/atlas/sensor/imu/types.hpp"
#include "autonomy/localization/atlas/sensor/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace tracking {

/**
 * @class autonomy::localization::atlas::tracking::Tracker
 * @brief ORB-SLAM3-style frontend tracker (RGB-D / stereo init + motion model / local map).
 *
 * Responsibilities:
 * - Build current Frame, feature matching, and pose estimation;
 * - Maintain local keyframe/map-point window; decide and create keyframes;
 * - In IMU mode, enqueue measurements and run inter-frame preintegration + pose prediction.
 *
 * @note Main tracking path (Track / GrabImage*) must run single-threaded on the **Tracking thread**;
 *       GrabImuData may safely enqueue from other threads via ThreadSafeQueue.
 */
class Tracker {
public:
    /**
     * @enum autonomy::localization::atlas::tracking::Tracker::State
     * @brief Tracking state machine (aligned with ORB-SLAM3 Tracking::eTrackingState).
     */
    enum class State {
        kSystemNotReady = -1,  ///< System not ready yet
        kNoImagesYet = 0,      ///< No images received yet
        kNotInitialized = 1,   ///< Images received but map not initialized
        kOk = 2,               ///< Tracking OK
        kRecentlyLost = 3,     ///< Briefly lost (IMU may keep preintegration window)
        kLost = 4,             ///< Tracking lost; needs relocalization or a new map
    };

    /**
     * @enum autonomy::localization::atlas::tracking::Tracker::Sensor
     * @brief Sensor configuration (mono/stereo/RGB-D and IMU combinations).
     */
    enum class Sensor {
        kMonocular = 0,    ///< Monocular
        kStereo = 1,       ///< Stereo
        kRgbd = 2,         ///< RGB-D
        kImuMonocular = 3, ///< Mono + IMU
        kImuStereo = 4,    ///< Stereo + IMU
        kImuRgbd = 5,      ///< RGB-D + IMU
    };

    /**
     * @struct autonomy::localization::atlas::tracking::Tracker::Options
     * @brief Tracker runtime options (sensor type, ORB params, depth scale, …).
     */
    struct Options {
        Sensor sensor = Sensor::kRgbd;  ///< Sensor mode
        feature::OrbParams orb;         ///< ORB extraction parameters
        float depth_map_factor = 1.f;    ///< Depth-map value → meters scale
        float depth_threshold = 40.f;   ///< Near/far depth threshold (meters; ORB-consistent)
        bool rgb = true;                ///< Color input (else treat as gray)
    };

    Tracker() = default;

    /**
     * @brief Map `AtlasConfig::camera_sensor` to a Tracker sensor, optionally with IMU.
     * @param config Runtime config (`camera.sensor`).
     * @param with_imu If true, return the inertial variant of the rig.
     * @return Tracker::Sensor used by VisualOdometry / VisualInertial.
     */
    static Sensor SensorFromConfig(const AtlasConfig& config, bool with_imu) {
        switch (config.camera_sensor) {
            case CameraSensor::kMonocular:
                return with_imu ? Sensor::kImuMonocular : Sensor::kMonocular;
            case CameraSensor::kStereo:
                return with_imu ? Sensor::kImuStereo : Sensor::kStereo;
            case CameraSensor::kRgbd:
            default:
                return with_imu ? Sensor::kImuRgbd : Sensor::kRgbd;
        }
    }

    /**
     * @brief Initialize camera, ORB, vocabulary, and map pipeline from Atlas config + options.
     * @param config Global config (calibration, vocabulary path, …).
     * @param options Runtime options (sensor, ORB, …).
     * @return true on success.
     * @note If using MultiMap, call SetMultiMap before Init.
     */
    bool Init(const AtlasConfig& config, Options options);

    /**
     * @brief Reset tracking and map-related state.
     */
    void Reset();

    /**
     * @brief Share MultiMap ownership (ORB-SLAM3 Atlas).
     * @param multi_map Multi-map container; must be set before Init.
     */
    void SetMultiMap(const std::shared_ptr<MultiMap>& multi_map);

    /**
     * @brief Create a new empty map on lost tracking / map switch.
     */
    void CreateNewMap();

    /**
     * @brief RGB-D grab entry (ORB GrabImageRGBD → Track).
     * @param rgb Color or gray image.
     * @param depth Aligned depth map.
     * @param timestamp Timestamp (seconds).
     * @return Current estimated pose (impl: world←body or T_cw; matches PublishResult).
     * @note Called on the Tracking thread.
     */
    SE3 GrabImageRgbd(const cv::Mat& rgb, const cv::Mat& depth,
                      double timestamp);

    /**
     * @brief Stereo grab entry.
     * @param left Left image.
     * @param right Right image (rectified or fisheye pipeline).
     * @param timestamp Timestamp (seconds).
     * @return Current estimated pose.
     */
    SE3 GrabImageStereo(const cv::Mat& left, const cv::Mat& right,
                        double timestamp);

    /**
     * @brief Monocular grab entry.
     * @param image Gray/color image.
     * @param timestamp Timestamp (seconds).
     * @return Current estimated pose.
     */
    SE3 GrabImageMonocular(const cv::Mat& image, double timestamp);

    /**
     * @brief Enqueue an IMU measurement (ORB GrabImuData).
     * @param measurement Single IMU sample.
     * @note May run concurrent with Tracking; buffered in a thread-safe queue, consumed in PreintegrateImu.
     */
    void GrabImuData(const sensor::imu::Measurement& measurement);

    /**
     * @brief Inject IMU↔camera extrinsics and noise (Tbc = calib.T_body_camera).
     * @param calib IMU calibration struct.
     */
    void SetImuCalib(const sensor::imu::Calib& calib);

    /**
     * @brief FrontendBase-style entry: dispatch image/IMU from SensorData and track.
     * @param data Multi-sensor packet of automsgs `sensor_msgs` messages.
     * @return Whether this step advanced successfully.
     */
    bool Process(const SensorData& data);

    /**
     * @brief Read the latest published odometry result.
     * @param[out] out Output; must not be nullptr.
     * @return true if a valid result exists.
     */
    bool GetResult(OdometryResult* out) const;

    /**
     * @brief Pop one pending system Keyframe for map insertion.
     * @param[out] out Output keyframe.
     * @return true if a pending keyframe was written successfully.
     */
    bool ConsumePendingKeyframe(Keyframe* out);

    /**
     * @brief Enable async LocalMapping (Track no longer drains the mapping queue on this thread).
     * @param enabled If true, an async worker owns LocalMapping.
     */
    void set_async_mapping(bool enabled) { async_mapping_ = enabled; }

    /**
     * @brief Whether async mapping is enabled.
     */
    bool async_mapping() const { return async_mapping_; }

    /**
     * @brief After prolonged loss, open a new map in Atlas (ORB CreateMapInAtlas).
     */
    void CreateMapInAtlas();

    /**
     * @brief Clear active map contents and re-init tracking (ORB ResetActiveMap).
     */
    void ResetActiveMap();

    /** @brief Current tracking state. */
    State state() const { return state_; }
    /** @brief Mutable active-map pointer. */
    Map* mutable_map() { return map_.get(); }
    /** @brief Read-only active map. */
    const Map* map() const { return map_.get(); }
    /** @brief Mutable MultiMap. */
    MultiMap* mutable_multi_map() { return multi_map_.get(); }
    /** @brief Mutable local-mapping module. */
    LocalMapping* mutable_local_mapping() { return local_mapping_.get(); }
    /** @brief Mutable keyframe database (relocalization / loop retrieval). */
    KeyFrameDatabase* mutable_keyframe_database() {
        return keyframe_database_.get();
    }
    /** @brief Most recently created internal KeyFrame. */
    std::shared_ptr<KeyFrame> last_keyframe() const { return last_keyframe_; }

    /**
     * @brief Localization-only mode (ORB mbOnlyTracking): do not insert new keyframes.
     * @param enabled true to enable.
     */
    void set_only_tracking(bool enabled) { only_tracking_ = enabled; }
    /** @brief Whether localization-only mode is on. */
    bool only_tracking() const { return only_tracking_; }
    /** @brief Inliers of the latest successful tracking step. */
    int matches_inliers() const { return matches_inliers_; }
    /** @brief Current tracking state. */
    State state() const { return state_; }

    /**
     * @brief One tracked frame stored relative to its reference keyframe.
     *
     * Used by TUM / EuRoC / KITTI trajectory export after later BA.
     */
    struct FramePoseRecord {
        SE3 relative_cw = SE3Identity();              ///< \(T_{cr}\)
        std::shared_ptr<KeyFrame> reference;         ///< Reference keyframe
        double timestamp = 0.0;                      ///< Frame timestamp [s]
        bool lost = false;                           ///< Tracking failed
    };
    /** @brief Frames recorded while tracking was OK or recently lost. */
    const std::vector<FramePoseRecord>& frame_poses() const {
        return frame_poses_;
    }

    /**
     * @brief Reload camera intrinsics, distortion and stereo baseline.
     * @param settings_path Atlas YAML (ORB `ChangeCalibration` settings file).
     */
    void ChangeCalibration(const std::string& settings_path);

    /**
     * @brief After inertial optimization, propagate scale and bias to frames (ORB UpdateFrameIMU).
     * @param scale Scale factor.
     * @param bias Updated IMU bias.
     * @param current_keyframe Reference keyframe.
     */
    void UpdateFrameIMU(float scale, const sensor::imu::Bias& bias,
                        const std::shared_ptr<KeyFrame>& current_keyframe);

private:
    /** @brief Main tracking loop (init / motion model / local map / relocalization). */
    void Track();
    /** @brief RGB-D / stereo map initialization. */
    void StereoInitialization();
    /** @brief Monocular two-view initialization. */
    void MonocularInitialization();
    /** @brief Track vs reference keyframe via BoW / brute-force matching. */
    bool TrackReferenceKeyFrame();
    /** @brief Constant-velocity motion-model projection matching. */
    bool TrackWithMotionModel();
    /** @brief Local map-point search and pose re-optimization. */
    bool TrackLocalMap();
    /** @brief Relocalization after loss. */
    bool Relocalization();
    /** @brief Refresh local keyframe and map-point sets. */
    void UpdateLocalMap();
    void UpdateLocalKeyFrames();
    void UpdateLocalPoints();
    /** @brief Projection search for matches in the local window. */
    void SearchLocalPoints();
    /** @brief Whether a new keyframe should be inserted. */
    bool NeedNewKeyFrame() const;
    /** @brief Create and register a new keyframe. */
    void CreateNewKeyFrame();
    /** @brief Update last-frame pose and temporary map points. */
    void UpdateLastFrame();
    void PrepareFrame();
    void CheckReplacedInLastFrame();
    void RegisterKeyFrame(const std::shared_ptr<KeyFrame>& keyframe);
    /**
     * @brief Predict current pose from IMU preintegration (ORB PredictStateIMU).
     * @return true if prediction succeeded.
     */
    bool PredictStateImu();
    /**
     * @brief Visual or visual-inertial pose optimization (TrackLocalMap branch).
     * @return Inlier count.
     */
    int OptimizeCurrentPose();
    void EnsureImuCalib();
    void ResetFrameImuPreintegrator();
    cv::Mat ToGray(const cv::Mat& image) const;
    void PublishResult(bool valid);
    /** @brief Append the current frame to the trajectory log. */
    void RecordFramePose();
    bool IsImuSensor() const;
    bool IsMonocularSensor() const;
    bool LoadVocabulary(const AtlasConfig& config);
    Keyframe MakeSystemKeyframe(const std::shared_ptr<KeyFrame>& keyframe) const;
    /** @brief Consume IMU queue and update frame/keyframe preintegration. */
    void PreintegrateImu();
    cv::Mat PreprocessImage(const cv::Mat& image) const;
    void BuildDistCoefFromConfig();
    void BuildCameraFromConfig();

    Options options_;
    AtlasConfig config_;
    cv::Mat dist_coef_;
    bool need_resize_ = false;
    int resize_width_ = 0;
    int resize_height_ = 0;
    std::shared_ptr<sensor::GeometricCamera> camera_;
    std::shared_ptr<sensor::GeometricCamera> camera2_;
    SE3 T_c1_c2_ = SE3Identity();
    bool has_T_c1_c2_ = false;
    std::unique_ptr<feature::OrbExtractor> orb_extractor_left_;
    std::unique_ptr<feature::OrbExtractor> orb_extractor_right_;
    std::unique_ptr<feature::OrbExtractor> orb_extractor_ini_;
    std::shared_ptr<feature::OrbVocabulary> vocabulary_;
    std::unique_ptr<KeyFrameDatabase> keyframe_database_;
    std::shared_ptr<MultiMap> multi_map_;
    std::shared_ptr<Map> map_;
    std::unique_ptr<LocalMapping> local_mapping_;

    State state_ = State::kNoImagesYet;
    std::vector<FramePoseRecord> frame_poses_;  ///< Relative poses for export
    Frame current_frame_;
    Frame last_frame_;
    Frame initial_frame_;
    bool has_initial_frame_ = false;
    std::vector<int> ini_matches_;
    std::vector<cv::Point2f> ini_prev_matched_;
    std::shared_ptr<KeyFrame> last_keyframe_;
    std::shared_ptr<KeyFrame> reference_keyframe_;
    std::vector<std::shared_ptr<KeyFrame>> local_keyframes_;
    std::vector<std::shared_ptr<MapPoint>> local_map_points_;
    std::list<std::shared_ptr<MapPoint>> temporal_points_;

    SE3 velocity_ = SE3Identity();
    bool velocity_valid_ = false;
    int matches_inliers_ = 0;

    // IMU (producer: GrabImuData; consumer: Tracking-thread PreintegrateImu).
    ::autolink::base::ThreadSafeQueue<sensor::imu::Measurement> imu_queue_;
    std::deque<sensor::imu::Measurement> imu_pending_;
    std::shared_ptr<sensor::imu::Preintegrator> imu_preintegrated_from_last_kf_;
    std::shared_ptr<sensor::imu::Preintegrator> imu_preintegrated_from_last_frame_;
    sensor::imu::Calib imu_calib_;
    sensor::imu::Bias last_bias_;
    bool async_mapping_ = false;
    bool map_updated_for_inertial_ = true;
    bool only_tracking_ = false;
    //! Localization-only VO mode: few map matches, mostly temporary points (ORB mbVO).
    bool vo_mode_ = false;
    int max_frames_ = 30;
    int min_frames_ = 0;
    long unsigned int last_reloc_frame_id_ = 0;
    double time_stamp_lost_ = 0.0;
    double time_recently_lost_ = 5.0;  ///< ORB IMU recently-lost window (seconds)
    bool insert_kfs_when_lost_ = true;

    OdometryResult last_result_;
    std::optional<Keyframe> pending_system_keyframe_;
    long unsigned int last_keyframe_frame_id_ = 0;
};

}  // namespace tracking
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_TRACKING_TRACKER_HPP_
