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
 * @file keyframe.hpp
 * @brief Map keyframe (ORB-SLAM3 `KeyFrame`: covisibility, spanning tree, BoW, stereo/IMU).
 *
 * Promoted from `tracking::Frame`; becomes a map topology node after entering
 * LocalMapping / LoopClosing. Pose is \(T_{cw}\) (camera ← world). Stereo
 * fisheye uses `num_left` and `grid_right_` for left/right feature indexing.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MAP_KEYFRAME_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MAP_KEYFRAME_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include "autonomy/localization/atlas/common/types.hpp"
#include "autonomy/localization/atlas/frontend/feature/orb/orb_vocabulary.hpp"
#include "autonomy/localization/atlas/frontend/tracking/frame.hpp"
#include "autonomy/localization/atlas/sensor/geometric_camera.hpp"
#include "autonomy/localization/atlas/sensor/imu/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

class MapPoint;
class Map;

/**
 * @class autonomy::localization::atlas::KeyFrame
 * @brief Keyframe: features, map-point slots, covisibility, spanning tree, loop/merge edges.
 *
 * @code{.cpp}
 * auto kf = std::make_shared<KeyFrame>(frame, map.get());
 * map->AddKeyFrame(kf);
 * kf->ComputeBoW();
 * kf->UpdateConnections();
 * @endcode
 *
 * @note Must be held as `shared_ptr`; covisibility keys are `weak_ptr`.
 */
class KeyFrame : public std::enable_shared_from_this<KeyFrame> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr int kGridRows = tracking::kFrameGridRows;  ///< Feature grid rows
    static constexpr int kGridCols = tracking::kFrameGridCols;  ///< Feature grid columns

    /**
     * @brief Construct from a tracking frame (copy features, depth, pose, camera, IMU).
     * @param[in] frame Source frame (pose and features should be set).
     * @param[in] map Owning map pointer.
     */
    KeyFrame(const tracking::Frame& frame, Map* map);

    /** @brief Set \(T_{cw}\). @param[in] pose_camera_world camera ← world. */
    void SetPose(const SE3& pose_camera_world);
    /** @brief Get \(T_{cw}\). */
    SE3 GetPose() const;
    /** @brief Get \(T_{wc}=T_{cw}^{-1}\). */
    SE3 GetPoseInverse() const;
    /** @brief Left camera center (world frame). */
    Vec3 GetCameraCenter() const;

    /**
     * @brief Associate a map point at a feature index.
     * @param[in] map_point Map point.
     * @param[in] index Global feature index.
     */
    void AddMapPoint(const std::shared_ptr<MapPoint>& map_point, int index);
    /** @brief Clear map-point slot by index. */
    void EraseMapPointMatch(int index);
    /** @brief Clear match slot by map-point pointer. */
    void EraseMapPointMatch(const std::shared_ptr<MapPoint>& map_point);
    /**
     * @brief Replace the map-point pointer at an index.
     * @param[in] index Feature index.
     * @param[in] map_point New map point.
     */
    void ReplaceMapPointMatch(int index,
                              const std::shared_ptr<MapPoint>& map_point);
    /** @brief Map point at index; nullptr if empty. */
    std::shared_ptr<MapPoint> GetMapPoint(int index) const;
    /** @brief All map-point slots (including nulls). */
    std::vector<std::shared_ptr<MapPoint>> GetMapPoints() const;
    /** @brief Same as GetMapPoints (ORB alias). */
    std::vector<std::shared_ptr<MapPoint>> GetMapPointMatches()
        const {
        return GetMapPoints();
    }

    /** @brief Left (or mono) undistorted keypoints. */
    const std::vector<cv::KeyPoint>& GetKeyPoints() const {
        return keypoints_;
    }
    /** @brief ORB descriptor matrix (row = feature). */
    cv::Mat GetDescriptors() const { return descriptors_; }
    /** @brief Per-feature depth; invalid ≤0. */
    const std::vector<float>& GetDepths() const { return depths_; }
    /** @brief Rectified stereo right u; fisheye may be placeholder. */
    const std::vector<float>& GetRightCoordinates() const {
        return right_coordinate_;
    }
    /** @brief Depth of one feature. @param[in] index Feature index. */
    float GetDepth(int index) const;

    /** @brief Set vocabulary. @param[in] vocabulary FBoW vocabulary. */
    void SetVocabulary(const std::shared_ptr<feature::OrbVocabulary>& vocabulary);
    /** @brief Compute BoW / FeatVector from descriptors. */
    void ComputeBoW();
    /**
     * @brief Restore BoW from archive (skip recomputation).
     * @param[in] bow BoW histogram.
     * @param[in] feat Feature-to-node inverted index.
     */
    void SetBoW(const fbow::BoWVector& bow, const fbow::BoWFeatVector& feat);
    /** @brief Whether BoW is ready. */
    bool HasBoW() const { return bow_ready_; }
    /** @brief BoW vector. */
    const fbow::BoWVector& bow_vector() const { return bow_vector_; }
    /** @brief FeatVector (SearchByBoW). */
    const fbow::BoWFeatVector& feat_vector() const {
        return feat_vector_;
    }

    /**
     * @brief Add a covisibility edge.
     * @param[in] keyframe Other keyframe.
     * @param[in] weight Number of shared map points.
     */
    void AddConnection(const std::shared_ptr<KeyFrame>& keyframe, int weight);
    /** @brief Erase a covisibility edge. */
    void EraseConnection(const std::shared_ptr<KeyFrame>& keyframe);
    /**
     * @brief Recompute covisibility from observations; optionally update spanning-tree parent.
     * @param[in] update_parent Whether to maintain parent.
     */
    void UpdateConnections(bool update_parent = true);
    /** @brief Sort covisibility list by weight. */
    void UpdateBestCovisibles();
    /**
     * @brief Top-n covisibility keyframes by weight.
     * @param[in] n Maximum count.
     */
    std::vector<std::shared_ptr<KeyFrame>>
    GetBestCovisibilityKeyFrames(int n) const;
    /** @brief Covisibility keyframes with weight ≥ weight. */
    std::vector<std::shared_ptr<KeyFrame>>
    GetCovisiblesByWeight(int weight) const;
    /** @brief All covisibility edges. */
    std::map<std::weak_ptr<KeyFrame>, int,
                           std::owner_less<std::weak_ptr<KeyFrame>>>
    GetConnectedKeyFrames() const;

    /** @brief Add a child in the spanning tree. */
    void AddChild(const std::shared_ptr<KeyFrame>& child);
    /** @brief Change parent and AddChild on the new parent. */
    void ChangeParent(const std::shared_ptr<KeyFrame>& parent);
    /** @brief Remove a spanning-tree child. */
    void EraseChild(const std::shared_ptr<KeyFrame>& child);
    /** @brief Spanning-tree parent keyframe. */
    std::shared_ptr<KeyFrame> GetParent() const;
    /** @brief Spanning-tree child keyframes. */
    std::set<std::shared_ptr<KeyFrame>> GetChildren() const;

    /** @brief Mark as a bad keyframe. */
    void SetBadFlag();
    /** @brief Whether this keyframe is bad. */
    bool isBad() const;

    /** @brief Add a loop edge. */
    void AddLoopEdge(const std::shared_ptr<KeyFrame>& keyframe);
    /** @brief Add a merge edge. */
    void AddMergeEdge(const std::shared_ptr<KeyFrame>& keyframe);
    /** @brief Loop-edge set. */
    std::set<std::shared_ptr<KeyFrame>> GetLoopEdges() const;
    /** @brief Merge-edge set. */
    std::set<std::shared_ptr<KeyFrame>> GetMergeEdges() const;

    /** @brief Change owning map. */
    void UpdateMap(Map* map);
    /** @brief Owning map. */
    Map* GetMap() const { return map_; }

    /** @brief Whether a pixel is inside the image. */
    bool IsInImage(float x, float y) const;
    /**
     * @brief Query neighboring feature indices via the grid.
     * @param[in] right true to query the right grid.
     */
    std::vector<size_t> GetFeaturesInArea(
        float x, float y, float radius, bool right = false) const;
    /**
     * @brief Scene-depth quantile.
     * @param[in] q Quantile parameter (often 2).
     */
    float ComputeSceneMedianDepth(int q) const;

    /** @brief Right-camera \(T_{c2w}\). */
    SE3 GetRightPose() const;
    /** @brief Right-camera \(T_{wc2}\). */
    SE3 GetRightPoseInverse() const;
    /** @brief Right camera center (world frame). */
    Vec3 GetRightCameraCenter() const;

    /**
     * @brief Count of map points with observations ≥ min_observations.
     * @param[in] min_observations Minimum observation threshold.
     */
    int TrackedMapPoints(int min_observations) const;

    long unsigned int id = 0;          ///< Keyframe ID
    long unsigned int frame_id = 0;    ///< Source Frame::id
    double timestamp = 0.0;            ///< Timestamp [s]
    static long unsigned int next_id;  ///< ID seed

    float fx = 0.f;  ///< Intrinsics fx
    float fy = 0.f;  ///< Intrinsics fy
    float cx = 0.f;  ///< Principal point cx
    float cy = 0.f;  ///< Principal point cy
    float inv_fx = 0.f;
    float inv_fy = 0.f;
    float baseline_times_fx = 0.f;  ///< bf
    float baseline_meters = 0.f;    ///< Baseline [m]
    float depth_threshold = 40.f;   ///< Near-point depth threshold [m]

    float scale_factor = 1.f;
    float log_scale_factor = 0.f;
    int scale_levels = 0;
    std::vector<float> scale_factors;
    std::vector<float> inverse_scale_factors;
    std::vector<float> level_sigma2;
    std::vector<float> inverse_level_sigma2;

    long unsigned int fuse_target_for_kf = 0;  ///< Fuse target marker

    Vec3 velocity_world = Vec3::Zero();  ///< World-frame velocity
    bool has_velocity = false;
    sensor::imu::Bias imu_bias;
    sensor::imu::Calib imu_calib;
    std::shared_ptr<sensor::imu::Preintegrator> imu_preintegrated;
    std::weak_ptr<KeyFrame> previous_keyframe;
    std::weak_ptr<KeyFrame> next_keyframe;
    bool imu_ready = false;

    SE3 pose_to_parent = SE3Identity();    ///< \(T_{cp}\) saved when this keyframe is culled
    SE3 pose_before_gba = SE3Identity();  ///< Live \(T_{cw}\) captured when GBA is applied
    SE3 pose_gba = SE3Identity();          ///< Optimized \(T_{cw}\) not yet written
    Vec3 velocity_gba = Vec3::Zero();      ///< Optimized velocity applied with GBA
    sensor::imu::Bias bias_gba;            ///< Optimized bias applied with GBA
    uint64_t gba_for_kf = 0;               ///< GBA epoch that produced `pose_gba`

    /** @brief IMU pose \(T_{wb}\). */
    SE3 GetImuPose() const;
    /** @brief Write back \(T_{cw}\) from \(T_{wb}\). */
    void SetImuPose(const SE3& T_wb);

    std::shared_ptr<sensor::GeometricCamera> camera;   ///< Left / primary camera
    std::shared_ptr<sensor::GeometricCamera> camera2;  ///< Right camera
    SE3 T_c1_c2 = SE3Identity();                       ///< Left ← right
    std::vector<int> left_to_right_match;
    int num_left = -1;  ///< Left feature count; -1 = single list
    int num_right = 0;

    /** @brief Whether stereo split indexing is used. */
    bool HasDualCameraIndex() const { return num_left >= 0; }
    /** @brief Total feature count. */
    int TotalFeatures() const {
        return HasDualCameraIndex()
                   ? (num_left + num_right)
                   : static_cast<int>(keypoints_.size());
    }
    /** @brief Keypoint by global index. */
    cv::KeyPoint GetKeyPoint(int index) const;
    /** @brief Right keypoints. */
    const std::vector<cv::KeyPoint>& GetKeyPointsRight() const {
        return keypoints_right_;
    }

private:
    void AssignFeaturesToGrid();

    mutable std::mutex mutex_pose_;
    mutable std::mutex mutex_features_;
    mutable std::mutex mutex_connections_;

    SE3 pose_camera_world_ = SE3Identity();
    std::vector<cv::KeyPoint> keypoints_;
    std::vector<cv::KeyPoint> keypoints_right_;
    cv::Mat descriptors_;
    std::vector<float> depths_;
    std::vector<float> right_coordinate_;
    std::vector<std::shared_ptr<MapPoint>> map_points_;
    Map* map_ = nullptr;

    std::shared_ptr<feature::OrbVocabulary> vocabulary_;
    fbow::BoWVector bow_vector_;
    fbow::BoWFeatVector feat_vector_;
    bool bow_ready_ = false;
    bool bad_ = false;
    bool first_connection_ = true;

    std::map<std::weak_ptr<KeyFrame>, int,
             std::owner_less<std::weak_ptr<KeyFrame>>>
        connected_keyframe_weights_;
    std::vector<std::shared_ptr<KeyFrame>> ordered_connected_keyframes_;
    std::vector<int> ordered_weights_;
    std::weak_ptr<KeyFrame> parent_;
    std::set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame>>>
        children_;
    std::set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame>>>
        loop_edges_;
    std::set<std::weak_ptr<KeyFrame>, std::owner_less<std::weak_ptr<KeyFrame>>>
        merge_edges_;

    float min_x_ = 0.f;
    float max_x_ = 0.f;
    float min_y_ = 0.f;
    float max_y_ = 0.f;
    float grid_element_width_inv_ = 0.f;
    float grid_element_height_inv_ = 0.f;
    std::vector<std::size_t> grid_[kGridCols][kGridRows];
    std::vector<std::size_t> grid_right_[kGridCols][kGridRows];
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MAP_KEYFRAME_HPP_
