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
 * @file map_point.hpp
 * @brief Sparse map point (ORB-SLAM3 `MapPoint`).
 *
 * Each `MapPoint` holds a world-frame 3D position, multi-keyframe observations,
 * a representative ORB descriptor, and scale-invariant distance bounds. The
 * tracking thread caches projection matches via public `track_*` fields;
 * fuse / loop maintain the replace chain through `Replace` / `GetReplaced`.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MAP_MAP_POINT_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MAP_MAP_POINT_HPP_

#include <map>
#include <memory>
#include <mutex>

#include <opencv2/core/mat.hpp>

#include "autonomy/localization/atlas/common/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

class KeyFrame;
class Map;

/**
 * @class autonomy::localization::atlas::MapPoint
 * @brief A 3D feature in the map and its multi-view observations.
 *
 * Typical lifecycle:
 * 1. Create via triangulation / stereo unprojection → `AddObservation` binds
 *    keyframe feature indices;
 * 2. `ComputeDistinctiveDescriptors` / `UpdateNormalAndDepth` maintain
 *    descriptor and normal;
 * 3. LocalMapping cull or Fuse uses `Replace` / `SetBadFlag`;
 * 4. Tracking `CheckReplacedInLastFrame` follows the `GetReplaced` chain.
 *
 * @note Must be managed with `std::shared_ptr`; observation keys are
 * `weak_ptr<KeyFrame>`.
 */
class MapPoint : public std::enable_shared_from_this<MapPoint> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct a map point.
     * @param[in] position_world World-frame coordinates.
     * @param[in] ref Reference keyframe (initial normal / scale).
     * @param[in] map Owning map (may be null; then `UpdateMap`).
     */
    MapPoint(const Vec3& position_world, const std::shared_ptr<KeyFrame>& ref,
             Map* map);

    /**
     * @brief Set world-frame position.
     * @param[in] position_world New coordinates.
     */
    void SetWorldPos(const Vec3& position_world);

    /**
     * @brief Read world-frame position.
     * @return Current 3D coordinates.
     */
    Vec3 GetWorldPos() const;

    /**
     * @brief Read mean observation normal (unit vector).
     * @return Normal maintained by `UpdateNormalAndDepth`.
     */
    Vec3 GetNormal() const;

    /**
     * @brief Add a keyframe observation.
     * @param[in] keyframe Observing keyframe.
     * @param[in] index Global feature index (right stereo: `num_left + local`).
     */
    void AddObservation(const std::shared_ptr<KeyFrame>& keyframe, int index);

    /**
     * @brief Erase the observation on a keyframe.
     * @param[in] keyframe Target keyframe.
     */
    void EraseObservation(const std::shared_ptr<KeyFrame>& keyframe);

    /**
     * @brief Copy the current observation table (weak_ptr → feature index).
     * @return Observation map.
     */
    std::map<std::weak_ptr<KeyFrame>, int,
                           std::owner_less<std::weak_ptr<KeyFrame>>>
    GetObservations() const;

    /**
     * @brief Number of valid observations.
     * @return Observation count.
     */
    int Observations() const;

    /**
     * @brief Whether this point is observed by the keyframe.
     * @param[in] keyframe Query keyframe.
     * @return true if the KF is in the observation table.
     */
    bool IsInKeyFrame(const std::shared_ptr<KeyFrame>& keyframe)
        const;

    /**
     * @brief Replace this point with another (fuse): migrate observations and mark bad.
     * @param[in] map_point Surviving map point (usually more observations).
     */
    void Replace(const std::shared_ptr<MapPoint>& map_point);

    /**
     * @brief Follow the fuse replace chain (ORB `GetReplaced`).
     * @return Final surviving map point; empty if never replaced.
     */
    std::shared_ptr<MapPoint> GetReplaced() const;

    /**
     * @brief Mark as bad and clear observations.
     */
    void SetBadFlag();

    /**
     * @brief Whether already marked bad.
     * @return true if unusable for matching / optimization.
     */
    bool isBad() const;

    /**
     * @brief Pick the descriptor with smallest median Hamming distance to others.
     * @note Call after observations stabilize; no-op if empty.
     */
    void ComputeDistinctiveDescriptors();

    /**
     * @brief Update mean normal and scale-invariant distance bounds from all observations.
     */
    void UpdateNormalAndDepth();

    /**
     * @brief Copy of the representative descriptor.
     * @return Single-row ORB descriptor; may be empty.
     */
    cv::Mat GetDescriptor() const;

    /**
     * @brief Minimum scale-invariant observation distance (about \(0.8 d_{\min}\)).
     */
    float GetMinDistanceInvariance() const;

    /**
     * @brief Maximum scale-invariant observation distance (about \(1.2 d_{\max}\)).
     */
    float GetMaxDistanceInvariance() const;

    /**
     * @brief Predict pyramid level for the current distance (ORB `PredictScale`).
     * @param[in] current_dist Camera-to-point distance.
     * @param[in] scale_levels Number of pyramid levels.
     * @param[in] log_scale_factor \(\log(\mathrm{scale\_factor})\).
     * @return Suggested octave, clamped to \([0, scale_levels-1]\).
     */
    int PredictScale(float current_dist, int scale_levels,
                                   float log_scale_factor) const;

    /**
     * @brief Found ratio \(\mathrm{found}/\mathrm{visible}\).
     * @return Ratio used by MapPointCulling.
     */
    float GetFoundRatio() const;

    /**
     * @brief Increase "entered frustum" count.
     * @param[in] n Increment, default 1.
     */
    void IncreaseVisible(int n = 1);

    /**
     * @brief Increase "successfully matched" count.
     * @param[in] n Increment, default 1.
     */
    void IncreaseFound(int n = 1);

    /**
     * @brief Migrate owning map pointer (map merge).
     * @param[in] map New map.
     */
    void UpdateMap(Map* map);

    /**
     * @brief Owning map.
     * @return Map pointer; may be null.
     */
    Map* GetMap() const { return map_; }

    long unsigned int id = 0;                 ///< Globally unique ID
    long unsigned int first_keyframe_id = 0;  ///< First observing keyframe ID
    static long unsigned int next_id;         ///< ID allocator

    // --- Tracking cache (ORB public fields; written by IsInFrustum / matching) ---
    bool track_in_view = false;    ///< Left in current-frame frustum
    bool track_in_view_r = false;  ///< Right in frustum
    float track_proj_x = 0.f;      ///< Left projection u
    float track_proj_y = 0.f;      ///< Left projection v
    float track_proj_xr = 0.f;  ///< Right u (rectified stereo) or fisheye right x
    float track_proj_yr = 0.f;  ///< Fisheye right y (ORB mTrackProjYR)
    int track_scale_level = 0;     ///< Left predicted octave
    int track_scale_level_r = -1;  ///< Right predicted octave; -1 = unused
    float track_view_cos = 1.f;    ///< Left view-angle cosine
    float track_view_cos_r = 1.f;  ///< Right view-angle cosine
    float track_depth = 0.f;       ///< Left depth
    float track_depth_r = 0.f;     ///< Right depth
    Vec3 position_gba = Vec3::Zero();  ///< Optimized position applied after GBA
    uint64_t gba_for_kf = 0;           ///< GBA epoch that produced `position_gba`

    /** @brief Reference keyframe used to back-project this point. */
    std::shared_ptr<KeyFrame> GetReferenceKeyFrame() const;
    long unsigned int last_frame_seen = 0;       ///< Last visible frame ID
    long unsigned int fuse_candidate_for_kf = 0; ///< Fuse target KF marker

private:
    mutable std::mutex mutex_position_;
    mutable std::mutex mutex_features_;

    Vec3 position_world_ = Vec3::Zero();
    Vec3 normal_ = Vec3::Zero();
    cv::Mat descriptor_;
    float min_distance_ = 0.f;
    float max_distance_ = 0.f;

    std::map<std::weak_ptr<KeyFrame>, int,
             std::owner_less<std::weak_ptr<KeyFrame>>>
        observations_;
    int observation_count_ = 0;
    int visible_count_ = 1;
    int found_count_ = 1;
    bool bad_ = false;
    std::weak_ptr<MapPoint> replaced_;

    std::weak_ptr<KeyFrame> reference_keyframe_;
    Map* map_ = nullptr;
};

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MAP_MAP_POINT_HPP_
