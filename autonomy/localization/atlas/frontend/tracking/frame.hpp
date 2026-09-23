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
 * Structure adapted from ORB-SLAM3 Frame (RGB-D / stereo / mono subset).
 */

/**
 * @file frame.hpp
 * @brief Single-frame image container: ORB features, depth/stereo matches, pose, BoW, IMU state.
 *
 * Structure aligned with ORB-SLAM3 Frame (RGB-D / stereo / mono subset). Pose convention:
 * `pose_camera_world` is T_cw (world → camera); public odometry still uses world←body.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_TRACKING_FRAME_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_TRACKING_FRAME_HPP_

#include <memory>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

#include "autonomy/localization/atlas/common/types.hpp"
#include "autonomy/localization/atlas/frontend/feature/orb/orb_extractor.hpp"
#include "autonomy/localization/atlas/frontend/feature/orb/orb_vocabulary.hpp"
#include "autonomy/localization/atlas/sensor/geometric_camera.hpp"
#include "autonomy/localization/atlas/sensor/imu/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

class MapPoint;
class KeyFrame;

namespace tracking {

constexpr int kFrameGridRows = 48;  ///< Feature grid rows (ORB style)
constexpr int kFrameGridCols = 64;  ///< Feature grid columns

/**
 * @class autonomy::localization::atlas::tracking::Frame
 * @brief Camera frame with ORB features (ORB-SLAM3 Frame subset).
 *
 * Construction extracts features and optionally fills depth/stereo; tracking sets pose,
 * associates MapPoints, and computes BoW. Supports pinhole and dual fisheye (camera / camera2 + T_c1_c2).
 *
 * @note Created and mutated by Tracker on the Tracking thread; not thread-safe.
 */
class Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Frame();

    /**
     * @brief RGB-D ctor: extract ORB; fill depth / right-u from the depth map.
     * @param gray Gray image.
     * @param depth Aligned depth map.
     * @param timestamp Timestamp.
     * @param extractor Left ORB extractor (non-owning).
     * @param fx,fy,cx,cy Pinhole intrinsics.
     * @param baseline_times_fx Baseline×fx (for disparity conversion).
     * @param depth_threshold Near/far depth threshold.
     */
    Frame(const cv::Mat& gray, const cv::Mat& depth, double timestamp,
          feature::OrbExtractor* extractor, float fx, float fy, float cx,
          float cy, float baseline_times_fx, float depth_threshold);

    /**
     * @brief Stereo ctor (rectified L/R). Optional lapping for fisheye overlap packing.
     * @param left_gray Left gray image.
     * @param right_gray Right gray image.
     * @param timestamp Timestamp.
     * @param left_extractor Left extractor.
     * @param right_extractor Right extractor.
     * @param fx,fy,cx,cy Intrinsics.
     * @param baseline_times_fx Baseline×fx.
     * @param depth_threshold Depth threshold.
     * @param lapping_left Left overlap column range [begin, end).
     * @param lapping_right Right overlap column range.
     */
    Frame(const cv::Mat& left_gray, const cv::Mat& right_gray, double timestamp,
          feature::OrbExtractor* left_extractor,
          feature::OrbExtractor* right_extractor, float fx, float fy, float cx,
          float cy, float baseline_times_fx, float depth_threshold,
          const std::vector<int>& lapping_left = {0, 1000},
          const std::vector<int>& lapping_right = {0, 1000});

    /**
     * @brief Monocular ctor: extract ORB only; no depth.
     * @param gray Gray image.
     * @param timestamp Timestamp.
     * @param extractor ORB extractor.
     * @param fx,fy,cx,cy Intrinsics.
     * @param baseline_times_fx Placeholder (usually 0 for mono).
     * @param depth_threshold Placeholder threshold.
     */
    Frame(const cv::Mat& gray, double timestamp, feature::OrbExtractor* extractor,
          float fx, float fy, float cx, float cy, float baseline_times_fx,
          float depth_threshold);

    /**
     * @brief Extract ORB for the given left/right flag.
     * @param flag 0=left, 1=right (matches ORB ExtractORB).
     * @param image Input gray image.
     */
    void ExtractOrb(int flag, const cv::Mat& image);

    /**
     * @brief Fill depths / right_coordinate from an RGB-D depth map.
     * @param depth Depth map.
     */
    void ComputeStereoFromRgbd(const cv::Mat& depth);

    /**
     * @brief Rectified stereo matching; fill depths.
     */
    void ComputeStereoMatches();

    /**
     * @brief Fisheye stereo matching (ORB ComputeStereoFishEyeMatches: BF + triangulate).
     */
    void ComputeStereoFishEyeMatches();

    /**
     * @brief Unproject the @p index-th stereo point to the world frame.
     * @param index Feature index.
     * @param[out] point_world World coordinates.
     * @return true if depth is valid and pose has been set.
     */
    bool UnprojectStereo(int index, Vec3* point_world) const;

    /**
     * @brief Set camera pose T_cw.
     * @param pose_camera_world World → camera.
     */
    void SetPose(const SE3& pose_camera_world);

    /** @brief Get T_cw. */
    SE3 GetPose() const { return pose_camera_world_; }
    /** @brief Whether pose has been set. */
    bool has_pose() const { return has_pose_; }
    /** @brief Camera optical center in world. */
    Vec3 GetCameraCenter() const;
    /**
     * @brief Export world←body pose (Atlas odometry convention).
     */
    SE3 PoseWorldBody() const;

    /**
     * @brief Bind ORB vocabulary for BoW computation.
     * @param vocabulary Vocabulary shared pointer.
     */
    void SetVocabulary(const std::shared_ptr<feature::OrbVocabulary>& vocabulary);

    /**
     * @brief Compute BoW / Feat vectors from descriptors.
     */
    void ComputeBoW();

    /** @brief Whether BoW has been computed. */
    bool HasBoW() const { return bow_ready_; }
    const fbow::BoWVector& bow_vector() const { return bow_vector_; }
    const fbow::BoWFeatVector& feat_vector() const {
        return feat_vector_;
    }

    /**
     * @brief Assign features to an image grid for fast region queries.
     */
    void AssignFeaturesToGrid();

    /**
     * @brief Query feature indices inside a circular region.
     * @param x,y Circle center (pixels).
     * @param radius Radius.
     * @param min_level,max_level Pyramid level range; -1 means unrestricted.
     * @param right If true, query the right-camera grid.
     * @return List of feature indices.
     */
    std::vector<size_t> GetFeaturesInArea(
        float x, float y, float radius, int min_level = -1,
        int max_level = -1, bool right = false) const;

    float min_x() const { return min_x_; }
    float max_x() const { return max_x_; }
    float min_y() const { return min_y_; }
    float max_y() const { return max_y_; }

    long unsigned int id = 0;  ///< Frame ID (auto-increment)
    double timestamp = 0.0;    ///< Timestamp
    int num_keypoints = 0;     ///< Keypoint count (left or pre-merge)

    //! Optional GeometricCamera (ORB mpCamera); preferred for projection.
    std::shared_ptr<sensor::GeometricCamera> camera;
    //! Right fisheye camera + T_left_right (ORB mpCamera2 / mTlr).
    std::shared_ptr<sensor::GeometricCamera> camera2;
    SE3 T_c1_c2 = SE3Identity();  ///< Left→right (or c1→c2) extrinsics
    //! Fisheye ORB packing overlap columns (ORB mvLappingArea).
    std::vector<int> lapping_left = {0, 1000};
    std::vector<int> lapping_right = {0, 1000};
    //! Non-overlap keypoint count at the front of the vector (ORB monoLeft).
    int mono_left = 0;
    int mono_right = 0;

    //! Fisheye dual-camera indices (ORB Nleft/Nright; -1 if not dual-camera).
    int num_left = -1;
    int num_right = 0;
    bool HasDualCameraIndex() const { return num_left >= 0; }
    int TotalFeatures() const {
        return HasDualCameraIndex() ? (num_left + num_right) : num_keypoints;
    }
    /**
     * @brief Keypoint by unified index (left+right concat for dual-camera).
     * @param index Feature index.
     */
    cv::KeyPoint GetKeyPoint(int index) const;
    /**
     * @brief After fisheye matching: concat descriptors and grow map_points to N.
     */
    void FinalizeFisheyeStereo();

    float fx = 0.f;
    float fy = 0.f;
    float cx = 0.f;
    float cy = 0.f;
    float inv_fx = 0.f;
    float inv_fy = 0.f;
    float baseline_times_fx = 0.f;
    float baseline_meters = 0.f;
    float depth_threshold = 40.f;

    //! OpenCV distortion coeffs (ORB mDistCoef); empty/all-zero skips undistort.
    cv::Mat dist_coef;

    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::KeyPoint> keypoints_right;
    std::vector<cv::KeyPoint> keypoints_undistorted;
    cv::Mat descriptors;
    cv::Mat descriptors_right;

    //! Stereo right-u; negative for monocular.
    std::vector<float> right_coordinate;
    std::vector<float> depths;
    //! Fisheye left↔right index map (ORB mvLeftToRightMatch).
    std::vector<int> left_to_right_match;
    std::vector<int> right_to_left_match;
    //! Fisheye-triangulated 3D in camera-1 (ORB mvStereo3Dpoints).
    std::vector<Vec3> stereo_3d_points;
    std::vector<std::shared_ptr<MapPoint>> map_points;  ///< One-to-one with features
    std::vector<bool> outliers;                         ///< Outlier mask

    feature::OrbExtractor* orb_extractor_left = nullptr;
    feature::OrbExtractor* orb_extractor_right = nullptr;

    std::shared_ptr<KeyFrame> reference_keyframe;

    /**
     * @brief Soft pose-velocity-bias prior (ORB `ConstraintPoseImu` / `mpcpi`).
     *
     * Written after pose-inertial optimization and consumed as
     * `EdgePriorPoseImu` on the previous frame.
     */
    struct PoseImuPrior {
        SE3 Twb = SE3Identity();             ///< Prior body pose
        Vec3 velocity = Vec3::Zero();        ///< Prior world velocity
        sensor::imu::Bias bias;              ///< Prior gyro / accelerometer bias
        bool valid = false;                  ///< False until the first inertial pose opt
        /// Symmetric square root of the 15×15 information (ORB `ConstraintPoseImu::H`).
        Eigen::Matrix<double, 15, 15> sqrt_information =
            Eigen::Matrix<double, 15, 15>::Identity();
    };

    // IMU state (ORB-SLAM3 Frame inertial fields).
    Vec3 velocity_world = Vec3::Zero();
    bool has_velocity = false;
    sensor::imu::Bias imu_bias;
    sensor::imu::Calib imu_calib;
    std::shared_ptr<sensor::imu::Preintegrator> imu_preintegrated;
    std::shared_ptr<sensor::imu::Preintegrator> imu_preintegrated_from_last_frame;
    PoseImuPrior pose_imu_prior;  ///< Prior for the next frame's inertial pose opt

    /**
     * @brief IMU-frame pose Twb (via Tcb, ORB GetImuPose).
     */
    SE3 GetImuPose() const;
    /**
     * @brief Set camera pose from Twb.
     * @param T_wb World←IMU/body.
     */
    void SetImuPose(const SE3& T_wb);

    int scale_levels = 0;
    float scale_factor = 1.f;
    float log_scale_factor = 0.f;
    std::vector<float> scale_factors;
    std::vector<float> inverse_scale_factors;
    std::vector<float> level_sigma2;
    std::vector<float> inverse_level_sigma2;

    /**
     * @brief Whether a map point is in the frustum (ORB isInFrustum).
     * @param map_point Map point.
     * @param viewing_cos_limit Lower bound on viewing-angle cosine.
     * @param right If true, use camera2.
     * @return true if in frustum and projectable; also updates observation prediction cache.
     */
    bool IsInFrustum(const std::shared_ptr<MapPoint>& map_point,
                     float viewing_cos_limit, bool right = false);

    /**
     * @brief Apply distortion and rebuild undistorted keypoints + grid (ORB UndistortKeyPoints).
     * @param dist Distortion coefficients.
     */
    void ApplyDistortion(const cv::Mat& dist);

    /**
     * @brief Set grid bounds from image size / undistortion (ORB ComputeImageBounds).
     * @param image_width Width.
     * @param image_height Height.
     */
    void ComputeImageBounds(int image_width, int image_height);

    static long unsigned int next_id;  ///< Next frame ID

private:
    void UndistortKeyPoints();
    bool PositionInGrid(const cv::KeyPoint& keypoint, int* pos_x,
                        int* pos_y) const;

    SE3 pose_camera_world_ = SE3Identity();  ///< T_cw
    bool has_pose_ = false;

    std::shared_ptr<feature::OrbVocabulary> vocabulary_;
    fbow::BoWVector bow_vector_;
    fbow::BoWFeatVector feat_vector_;
    bool bow_ready_ = false;

    float grid_element_width_inv_ = 0.f;
    float grid_element_height_inv_ = 0.f;
    float min_x_ = 0.f;
    float max_x_ = 0.f;
    float min_y_ = 0.f;
    float max_y_ = 0.f;
    std::vector<std::size_t> grid_[kFrameGridCols][kFrameGridRows];
    std::vector<std::size_t> grid_right_[kFrameGridCols][kFrameGridRows];
};

}  // namespace tracking
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_TRACKING_FRAME_HPP_
