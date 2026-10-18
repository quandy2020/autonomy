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
 * @file vio_manager.hpp
 * @brief Direct visual ESIKF update on sparse visual map (FAST-LIVO2 VIO).
 */

#ifndef ARGUS_ODOMETRY_VIO_MANAGER_HPP_
#define ARGUS_ODOMETRY_VIO_MANAGER_HPP_

#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include "argus/common/camera/abstract_camera.hpp"
#include "argus/common/camera/pinhole_camera.hpp"
#include "argus/common/macros.hpp"
#include "argus/common/math/se3.hpp"
#include "argus/frontend/types.hpp"
#include "argus/map/visual_map.hpp"
#include "argus/map/visual_point.hpp"
#include "argus/map/voxel_map.hpp"
#include "argus/odometry/frame.hpp"

namespace argus {
namespace odometry {

/**
 * @brief Affine warp from reference patch coordinates to current frame.
 */
class Warp
{
public:
    /** @brief 2x2 warp from reference to current patch space. */
    Eigen::Matrix2d A_cur_ref = Eigen::Matrix2d::Identity();
    /** @brief Coarsest pyramid level used for this warp. */
    int search_level = 0;

    /**
     * @brief Construct warp metadata for patch alignment.
     * @param level Search pyramid level.
     * @param warp_matrix Affine warp matrix.
     */
    Warp(int level, const Eigen::Matrix2d& warp_matrix)
        : A_cur_ref(warp_matrix), search_level(level) {}
};

/**
 * @brief Direct visual-inertial update: sparse map, patch photometric ESIKF.
 *
 * Wired by `LivoOdometry` when vision is enabled. Call
 * `SetImuToLidarExtrinsic`, `SetLidarToCameraExtrinsic`, then `InitializeVIO()`
 * before `ProcessFrame`.
 */
class VIOManager
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(VIOManager);

    using SE3 = math::SE3;
    using StatesGroup = frontend::StatesGroup;
    using PointWithVar = frontend::PointWithVar;
    using VisualPoint = map::VisualPoint;
    using Feature = map::Feature;
    using SubSparseMap = map::SubSparseMap;
    using VisualMap = map::VisualMap;
    using VoxelLocation = map::VoxelLocation;
    using VoxelPoints = map::VoxelPoints;
    using VoxelOctoTree = map::VoxelOctoTree;

    /** @brief Image grid cell size (pixels) for feature distribution. */
    int grid_size = 0;
    /** @brief Active camera model (not owned). */
    camera::AbstractCamera* cam = nullptr;
    /** @brief Pinhole view when model is pinhole (not owned). */
    camera::PinholeCamera* pinhole_cam = nullptr;
    /** @brief ESIKF state updated by visual residuals (not owned). */
    StatesGroup* state = nullptr;
    /** @brief IMU-propagated state prior for this image (not owned). */
    StatesGroup* state_propagated = nullptr;

    /** @brief IMU to LiDAR rotation. */
    Eigen::Matrix3d Rli = Eigen::Matrix3d::Identity();
    /** @brief Camera to IMU rotation. */
    Eigen::Matrix3d Rci = Eigen::Matrix3d::Identity();
    /** @brief Camera to LiDAR rotation. */
    Eigen::Matrix3d Rcl = Eigen::Matrix3d::Identity();
    /** @brief World to camera rotation (cached). */
    Eigen::Matrix3d Rcw = Eigen::Matrix3d::Identity();
    /** @brief Jacobian of rotation w.r.t. SO(3) increment. */
    Eigen::Matrix3d Jdphi_dR = Eigen::Matrix3d::Identity();
    /** @brief Jacobian of position w.r.t. translation increment. */
    Eigen::Matrix3d Jdp_dt = Eigen::Matrix3d::Identity();
    /** @brief Jacobian of position w.r.t. rotation increment. */
    Eigen::Matrix3d Jdp_dR = Eigen::Matrix3d::Identity();
    /** @brief IMU to LiDAR translation. */
    Eigen::Vector3d Pli = Eigen::Vector3d::Zero();
    /** @brief Camera to IMU translation. */
    Eigen::Vector3d Pci = Eigen::Vector3d::Zero();
    /** @brief Camera to LiDAR translation. */
    Eigen::Vector3d Pcl = Eigen::Vector3d::Zero();
    /** @brief World to camera translation (cached). */
    Eigen::Vector3d Pcw = Eigen::Vector3d::Zero();

    std::vector<int> grid_num;     /**< @brief Per-cell feature counts. */
    std::vector<int> map_index;    /**< @brief Grid cell to map slot index. */
    std::vector<int> border_flag;  /**< @brief Border cell markers. */
    std::vector<int> update_flag;  /**< @brief Cells selected for update. */
    std::vector<float> map_dist;   /**< @brief Distance scores per grid cell. */
    std::vector<float> scan_value; /**< @brief Scan-line sampling buffer. */
    std::vector<float> patch_buffer; /**< @brief Stacked patch intensities. */

    /** @brief Use point surface normals in warp / residual. */
    bool enable_normal = true;
    /** @brief Inverse-composition Gauss-Newton on patches. */
    bool enable_inverse_composition = false;
    /** @brief Estimate exposure scale in the update. */
    bool enable_exposure_estimate = false;
    /** @brief Raycast LiDAR points for depth prior. */
    bool enable_raycast = false;
    /** @brief Reference patches precomputed for current frame. */
    bool has_ref_patch_cache = false;
    /** @brief Normalized cross-correlation outlier check. */
    bool enable_ncc = false;
    /** @brief Write Colmap-compatible debug dumps. */
    bool enable_colmap_output = false;

    int width = 0;         /**< @brief Image width after resize. */
    int height = 0;        /**< @brief Image height after resize. */
    int grid_n_width = 0;  /**< @brief Grid count along width. */
    int grid_n_height = 0; /**< @brief Grid count along height. */
    int length = 0;        /**< @brief Linearized grid index stride. */
    double image_resize_factor = 1.0; /**< @brief Scale applied to raw image. */
    double fx = 0;                    /**< @brief Focal length x (pixels). */
    double fy = 0;                    /**< @brief Focal length y (pixels). */
    double cx = 0;                    /**< @brief Principal point x (pixels). */
    double cy = 0;                    /**< @brief Principal point y (pixels). */
    int patch_pyramid_level = 0;      /**< @brief Patch pyramid depth. */
    int patch_size = 0;               /**< @brief Patch side length (pixels). */
    int patch_size_total = 0;         /**< @brief Total patch pixel count. */
    int patch_size_half = 0;          /**< @brief Half patch radius. */
    int border = 0;         /**< @brief Image border margin (pixels). */
    int warp_len = 0;       /**< @brief Warp parameter vector length. */
    int max_iterations = 0; /**< @brief Max GN iterations per level. */
    int total_points = 0;   /**< @brief Active map points in update. */

    double image_point_cov = 0;     /**< @brief Pixel noise variance. */
    double outlier_threshold = 0; /**< @brief Photometric outlier gate. */
    double ncc_threshold = 0;          /**< @brief Min NCC for inliers. */

    SubSparseMap* visual_submap =
        nullptr;          /**< @brief Active submap window (not owned). */
    VisualMap visual_map; /**< @brief Global sparse visual map. */
    std::vector<std::vector<Eigen::Vector3d>>
        rays_with_sample_points; /**< @brief Raycast samples. */

    double compute_jacobian_time =
        0;                      /**< @brief Last Jacobian build time (s). */
    double update_ekf_time = 0; /**< @brief Last EKF update time (s). */
    double ave_total = 0;       /**< @brief Running average VIO time (s). */

    int frame_count = 0;    /**< @brief Processed frame counter. */
    bool enable_plot = false; /**< @brief Enable debug visualization. */

    Eigen::Matrix<double, frontend::kDimState, frontend::kDimState>
        G; /**< @brief Kalman gain helper. */
    Eigen::Matrix<double, frontend::kDimState, frontend::kDimState>
        H_T_H;         /**< @brief Normal equations block. */
    Eigen::MatrixXd K; /**< @brief Kalman gain matrix. */
    Eigen::MatrixXd
        H_sub_inv; /**< @brief Inverse-composition Hessian factor. */

    std::ofstream fout_camera; /**< @brief Camera trajectory log stream. */
    std::ofstream colmap_log_; /**< @brief Colmap export stream. */
    /** @brief Optional log root (Colmap dumps); empty disables file output. */
    std::string log_dir;

    std::unordered_map<VoxelLocation, VoxelPoints*>&
        feat_map; /**< @brief Voxel feature index (owned elsewhere). */
    std::unordered_map<VoxelLocation, int>
        sub_feat_map; /**< @brief Submap voxel occupancy. */
    std::unordered_map<int, Warp*>
        warp_map; /**< @brief Per-point warp cache. */
    std::vector<VisualPoint*>
        retrieve_voxel_points; /**< @brief Map points matched this frame. */
    std::vector<PointWithVar>
        append_voxel_points; /**< @brief New points from LiDAR overlay. */
    FramePtr new_frame_;     /**< @brief Current processing frame. */
    cv::Mat img_cp;          /**< @brief Working grayscale image. */
    cv::Mat img_rgb;         /**< @brief RGB copy for visualization. */
    cv::Mat img_test;        /**< @brief Debug overlay image. */

    /** @brief Grid cell classification for map vs LiDAR seeding. */
    enum CellType { TYPE_MAP = 1, TYPE_POINTCLOUD, TYPE_UNKNOWN };

    /** @brief Construct manager; binds internal `feat_map` storage. */
    VIOManager();

    /** @brief Close log streams and release warp cache. */
    ~VIOManager();

    /**
     * @brief One inverse-composition GN step at a pyramid level.
     * @param img Current image pyramid level.
     * @param level Pyramid level index.
     */
    void UpdateStateInverse(cv::Mat img, int level);

    /**
     * @brief One forward-composition GN step at a pyramid level.
     * @param img Current image pyramid level.
     * @param level Pyramid level index.
     */
    void UpdateState(cv::Mat img, int level);

    /**
     * @brief Main VIO entry: retrieve map, update state, extend map.
     * @param img Input image (may be resized in place).
     * @param pg LiDAR points with variance for seeding / raycast.
     * @param plane_map Voxel plane map from LIO.
     * @param img_time Image timestamp (s).
     */
    void ProcessFrame(
        cv::Mat& img, std::vector<PointWithVar>& pg,
        const std::unordered_map<VoxelLocation, VoxelOctoTree*>& plane_map,
        double img_time);

    /**
     * @brief Match visual map points visible in the current image.
     * @param img Current grayscale image.
     * @param pg LiDAR points for depth / normal hints.
     * @param plane_map Voxel plane map from LIO.
     */
    void RetrieveFromVisualSparseMap(
        cv::Mat img, std::vector<PointWithVar>& pg,
        const std::unordered_map<VoxelLocation, VoxelOctoTree*>& plane_map);

    /**
     * @brief Seed new visual map points from image and LiDAR overlay.
     * @param img Current grayscale image.
     * @param pg Candidate 3D points with variance.
     */
    void GenerateVisualMapPoints(cv::Mat img, std::vector<PointWithVar>& pg);

    /**
     * @brief Set IMU-to-LiDAR extrinsic used to chain camera pose.
     * @param transl Translation IMU to LiDAR (m).
     * @param rot Rotation IMU to LiDAR.
     */
    void SetImuToLidarExtrinsic(const Eigen::Vector3d& transl,
                                const Eigen::Matrix3d& rot);

    /**
     * @brief Set LiDAR-to-camera extrinsic from calibration vectors.
     * @param R Rotation as 9-element row-major matrix.
     * @param P Translation as 3-element vector.
     */
    void SetLidarToCameraExtrinsic(std::vector<double>& R,
                                   std::vector<double>& P);

    /**
     * @brief Allocate grids, load camera intrinsics, open logs if configured.
     *
     * Requires `cam`, `state`, and extrinsics set; sets `width` / `height` when
     * ready.
     */
    void InitializeVIO();

    /**
     * @brief Sample a patch around a pixel at a pyramid level.
     * @param img Source image or pyramid level.
     * @param pc Center pixel (level coordinates).
     * @param[out] patch_tmp Output buffer (`patch_size_total` floats).
     * @param level Pyramid level.
     */
    void GetImagePatch(cv::Mat img, Eigen::Vector2d pc, float* patch_tmp,
                       int level);

    /**
     * @brief Pinhole projection Jacobian d(u,v)/d(X,Y,Z).
     * @param p Point in camera frame.
     * @param[out] J 2x3 Jacobian.
     */
    void ComputeProjectionJacobian(Eigen::Vector3d p,
                                   Eigen::Matrix<double, 2, 3>& J);

    /**
     * @brief Build photometric residuals and apply ESIKF update.
     * @param img Current image for patch sampling.
     */
    void ComputeJacobianAndUpdateEKF(cv::Mat img);

    /** @brief Clear per-frame grid counters and flags. */
    void ResetGrid();

    /**
     * @brief Refine map point positions / normals from new observations.
     * @param img Current grayscale image.
     */
    void UpdateVisualMapPoints(cv::Mat img);

    /**
     * @brief Affine warp from reference to current for a 3D point.
     * @param cam Camera model.
     * @param px_ref Reference pixel.
     * @param f_ref Reference bearing.
     * @param depth_ref Depth along bearing.
     * @param T_cur_ref Pose from reference to current frame.
     * @param level_ref Reference pyramid level.
     * @param pyramid_level Current pyramid level.
     * @param halfpatch_size Patch half size.
     * @param[out] A_cur_ref Output 2x2 warp.
     */
    void GetWarpMatrixAffine(const camera::AbstractCamera& cam,
                             const Eigen::Vector2d& px_ref,
                             const Eigen::Vector3d& f_ref, double depth_ref,
                             const SE3& T_cur_ref, int level_ref,
                             int pyramid_level, int halfpatch_size,
                             Eigen::Matrix2d& A_cur_ref);

    /**
     * @brief Plane-induced homography warp for patch alignment.
     * @param cam Camera model.
     * @param px_ref Reference pixel.
     * @param xyz_ref 3D point in reference frame.
     * @param normal_ref Surface normal in reference frame.
     * @param T_cur_ref Pose from reference to current frame.
     * @param level_ref Reference pyramid level.
     * @param[out] A_cur_ref Output 2x2 affine approximation.
     */
    void GetWarpMatrixAffineHomography(const camera::AbstractCamera& cam,
                                       const Eigen::Vector2d& px_ref,
                                       const Eigen::Vector3d& xyz_ref,
                                       const Eigen::Vector3d& normal_ref,
                                       const SE3& T_cur_ref, int level_ref,
                                       Eigen::Matrix2d& A_cur_ref);

    /**
     * @brief Warp reference patch into current coordinates.
     * @param A_cur_ref Affine warp matrix.
     * @param img_ref Reference image pyramid level.
     * @param px_ref Reference pixel center.
     * @param level_ref Reference level.
     * @param search_level Coarsest search level.
     * @param pyramid_level Current pyramid level.
     * @param halfpatch_size Patch half size.
     * @param[out] patch Output intensity patch.
     */
    void WarpAffine(const Eigen::Matrix2d& A_cur_ref, const cv::Mat& img_ref,
                    const Eigen::Vector2d& px_ref, int level_ref,
                    int search_level, int pyramid_level, int halfpatch_size,
                    float* patch);

    /**
     * @brief Insert a new visual point into the voxel feature map.
     * @param pt_new New map point (ownership transferred to map).
     */
    void InsertPointIntoVoxelMap(VisualPoint* pt_new);

    /** @brief Draw tracked points on `img_test` when `enable_plot` is set. */
    void PlotTrackedPoints();

    /**
     * @brief Sync `new_frame_` pose with filter state.
     * @param state Current `StatesGroup` after update.
     */
    void UpdateFrameState(StatesGroup state);

    /**
     * @brief Warp reference patches into the current frame using plane map.
     * @param plane_map Voxel plane map from LIO.
     */
    void ProjectPatchFromRefToCur(
        const std::unordered_map<VoxelLocation, VoxelOctoTree*>& plane_map);

    /**
     * @brief Select best reference observation per map point.
     * @param plane_map Voxel plane map from LIO.
     */
    void UpdateReferencePatch(
        const std::unordered_map<VoxelLocation, VoxelOctoTree*>& plane_map);

    /**
     * @brief Cache reference patches at a pyramid level.
     * @param level Pyramid level to precompute.
     */
    void PrecomputeReferencePatches(int level);

    /** @brief Write Colmap debug artifacts when `enable_colmap_output`. */
    void DumpDataForColmap();

    /**
     * @brief Normalized cross-correlation between two patches.
     * @param ref_patch Reference intensities.
     * @param cur_patch Current intensities.
     * @param patch_size Side length (pixel count is `patch_size * patch_size`).
     * @return NCC score in [-1, 1].
     */
    double CalculateNCC(float* ref_patch, float* cur_patch, int patch_size);

    /**
     * @brief Pick coarsest pyramid level that keeps warp area bounded.
     * @param A_cur_ref Affine warp matrix.
     * @param max_level Highest allowed pyramid level.
     * @return Recommended search level.
     */
    int GetBestSearchLevel(const Eigen::Matrix2d& A_cur_ref, int max_level);

    /**
     * @brief Bilinear sample of grayscale intensity.
     * @param img Source image.
     * @param pc Sub-pixel coordinates.
     * @return Interpolated intensity.
     */
    Eigen::Vector3f GetInterpolatedPixel(cv::Mat img, Eigen::Vector2d pc);

private:
    /** @brief Load intrinsics and chain extrinsic Jacobians. */
    void InitIntrinsicsAndPoseJacobians();
    /** @brief Compute image grid dimensions from `grid_size`. */
    void InitImageGrid();
    /** @brief Precompute per-cell ray sample points when `enable_raycast`. */
    void InitRaycastSamples();
    /** @brief Open Colmap dump streams when `enable_colmap_output`. */
    void InitColmapLogs();
    /** @brief Allocate grid / patch working buffers. */
    void InitPatchBuffers();

    /** @brief Resize / grayscale prep and bind `new_frame_`. */
    void PrepareImage(cv::Mat& img);
    /** @brief Optional timing table when `enable_plot`. */
    void PrintTimingStats(double t1, double t2, double t3, double t4,
                             double t5, double t6, double t7,
                             const std::unordered_map<VoxelLocation,
                                                      VoxelOctoTree*>& feat_map);

    /** @brief Project LiDAR overlay into depth image and seed `sub_feat_map`. */
    void BuildDepthAndSubmap(const std::vector<PointWithVar>& pg,
                                    cv::Mat& depth_img, float voxel_size);
    /** @brief Mark FOV map points into grid; collect out-of-FOV voxel keys. */
    void CollectVisibleSubmapPoints(
        std::vector<VoxelLocation>& delete_key_list);
    /** @brief Process one ray sample; true = stop this ray. */
    bool TryRaycastHit(
        const Eigen::Vector3d& sample_point_w, float voxel_size,
        const std::unordered_map<VoxelLocation, VoxelOctoTree*>&
            plane_map);
    /** @brief Raycast empty cells to retrieve map / plane seeds. */
    void RaycastEmptyCells(
        float voxel_size,
        const std::unordered_map<VoxelLocation, VoxelOctoTree*>& plane_map);
    /** @brief Depth discontinuity check around projected pixel. */
    bool HasDepthJump(const float* depth_it, const Eigen::Vector2d& pc,
                              const Eigen::Vector3d& pt_cam) const;
    /** @brief Pick / cache reference feature for a map point. */
    bool SelectRefFeature(VisualPoint* pt, const Eigen::Vector2d& pc,
                                Feature*& ref_ftr);
    /** @brief Homography or affine warp + search level for a point. */
    void ComputePointWarp(VisualPoint* pt, Feature* ref_ftr,
                             int& search_level, Eigen::Matrix2d& A_cur_ref_zero);
    /** @brief Photometric / NCC gate and push into `visual_submap`. */
    void TryAddRetrievedPoint(cv::Mat img, int grid_index,
                                     const float* depth_it);
    /** @brief Walk TYPE_MAP cells and append inliers to submap. */
    void AppendRetrievedToSubmap(cv::Mat img, const float* depth_it);

    /** @brief Score LiDAR points into empty grid cells. */
    void ScoreCloudCandidates(cv::Mat img, std::vector<PointWithVar>& pg);
    /** @brief Score raycast-seeded voxel centers into empty cells. */
    void ScoreVoxelCandidates(cv::Mat img);
    /** @brief Create VisualPoint / Feature for TYPE_POINTCLOUD cells. */
    int InsertCandidatePoints(cv::Mat img);

    /** @brief Refine map-point normal from LIO plane if consistent. */
    void UpdateNormalFromPlane(
        VisualPoint* pt,
        const std::unordered_map<VoxelLocation, VoxelOctoTree*>& plane_map);
    /** @brief NCC + view-angle score to pick best reference patch. */
    void SelectBestRefPatch(VisualPoint* pt);

    /** @brief Debug: warp ref patches via plane normal homography. */
    void WarpPatchesByNormal(uchar* it_normal, int& num,
                                      const std::string& dir);
    /** @brief Debug overlay image for one tracked point. */
    void DrawRefCurOverlay(int submap_index, Feature* ref_ftr,
                                     const Eigen::Vector2d& pc,
                                     const Eigen::Vector2d& pc_prior, int num,
                                     const std::string& dir);
    /** @brief Rasterize homography-warped patch into normal debug image. */
    void RasterizeWarpedPatch(Feature* ref_ftr, const Eigen::Vector2d& pc,
                                  const Eigen::Matrix2d& A_cur_ref,
                                  int search_level, uchar* it_normal);
    /** @brief Debug: warp ref patches via affine model. */
    void WarpPatchesByAffine(uchar* it);
    /** @brief Write concatenated ref/cur debug images. */
    void SavePatchDebugMosaics(const cv::Mat& result,
                                         const cv::Mat& result_normal,
                                         const cv::Mat& img_photometric_error,
                                         const std::string& dir);

    /** @brief Fill one map point's rows in `H_sub_inv` at pyramid level. */
    void FillRefPatchJacobians(int level, int point_index);

    /** @brief Build residual / Jacobian for inverse-composition iteration. */
    float AccumulateInverseResiduals(
        cv::Mat img, int level, Eigen::VectorXd& z, Eigen::MatrixXd& H_sub,
        int& n_meas);
    /** @brief ESIKF step for inverse-composition update. */
    bool ApplyInverseUpdate(StatesGroup& old_state, float error,
                               float& last_error, const Eigen::VectorXd& z,
                               const Eigen::MatrixXd& H_sub);

    /** @brief One map-point residual/Jacobian block (forward composition). */
    float FillPointResidualsForward(cv::Mat img, int level, int point_index,
                                    Eigen::VectorXd& z, Eigen::MatrixXd& H_sub,
                                    int& n_meas);
    /** @brief Build residual / Jacobian for forward-composition iteration. */
    float AccumulateForwardResiduals(
        cv::Mat img, int level, Eigen::VectorXd& z, Eigen::MatrixXd& H_sub,
        int& n_meas);
    /** @brief ESIKF step for forward-composition update. */
    bool ApplyForwardUpdate(StatesGroup& old_state, float error,
                               float& last_error, const Eigen::VectorXd& z,
                               const Eigen::MatrixXd& H_sub);
};

}  // namespace odometry
}  // namespace argus

#endif  // ARGUS_ODOMETRY_VIO_MANAGER_HPP_
