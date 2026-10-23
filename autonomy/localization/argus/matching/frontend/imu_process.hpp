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
 * @file imu_process.hpp
 * @brief IMU initialization and LiDAR motion undistortion (FAST-LIVO2
 * ImuProcess).
 */

#ifndef ARGUS_FRONTEND_IMU_PROCESS_HPP_
#define ARGUS_FRONTEND_IMU_PROCESS_HPP_

#include <deque>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "argus/common/macros.hpp"
#include "argus/frontend/types.hpp"
#include "argus/sensor/imu_data.hpp"
#include "argus/sensor/point_types.hpp"

namespace argus {
namespace frontend {

/**
 * @brief Compare points by relative time stored in `curvature` (ms).
 * @param x Left point.
 * @param y Right point.
 * @return True if `x` is earlier within the scan than `y`.
 */
inline bool TimeListLess(const sensor::PointType& x,
                         const sensor::PointType& y) {
    return x.curvature < y.curvature;
}

/**
 * @brief Propagate IMU, initialize gravity/bias, and undistort LiDAR sweeps.
 */
class ImuProcess
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(ImuProcess);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess();

    /** @brief Clear buffers and re-enter IMU initialization. */
    void Reset();

    /**
     * @brief Set LiDAR←IMU extrinsic as a homogeneous transform.
     * @param T 4x4 matrix with rotation R and translation t (LiDAR in IMU).
     */
    void SetExtrinsic(const Eigen::Matrix4d& T);

    /**
     * @brief Set translation only (identity rotation).
     * @param transl LiDAR origin in the IMU frame (m).
     */
    void SetExtrinsic(const Eigen::Vector3d& transl);

    /**
     * @brief Set full extrinsic.
     * @param transl LiDAR origin in the IMU frame (m).
     * @param rot Rotation of LiDAR w.r.t. IMU.
     */
    void SetExtrinsic(const Eigen::Vector3d& transl,
                      const Eigen::Matrix3d& rot);

    /**
     * @brief Scale process noise on gyro.
     * @param scaler Diagonal scale factors.
     */
    void SetGyrCovScale(const Eigen::Vector3d& scaler);

    /**
     * @brief Scale process noise on accelerometer.
     * @param scaler Diagonal scale factors.
     */
    void SetAccCovScale(const Eigen::Vector3d& scaler);

    /**
     * @brief Gyro bias random-walk covariance diagonal.
     * @param b_g Bias covariance (rad/s)^2.
     */
    void SetGyrBiasCov(const Eigen::Vector3d& b_g);

    /**
     * @brief Accel bias random-walk covariance diagonal.
     * @param b_a Bias covariance (m/s^2)^2.
     */
    void SetAccBiasCov(const Eigen::Vector3d& b_a);

    /**
     * @brief Inverse-exposure process noise (VIO photometric).
     * @param inv_expo Scalar variance.
     */
    void SetInvExpoCov(double inv_expo);

    /**
     * @brief Number of IMU frames used during static init.
     * @param num Frame count (>=1).
     */
    void SetImuInitFrameNum(int num);

    /**
     * @brief Directory for optional IMU debug dumps.
     * @param dir Absolute or relative path; empty disables.
     */
    void SetLogDir(const std::string& dir);

    /**
     * @brief Enable/disable point-wise motion compensation.
     * @param enable True to undistort using IMU poses.
     */
    void SetMotionCompensate(bool enable);

    /** @brief Disable IMU; fall back to constant-velocity / no-IMU path. */
    void DisableImu();
    /** @brief Freeze gravity estimation after init. */
    void DisableGravityEstimate();
    /** @brief Freeze bias estimation. */
    void DisableBiasEstimate();
    /** @brief Freeze inverse-exposure estimation. */
    void DisableExposureEstimate();

    /**
     * @brief Main entry: init / propagate and produce undistorted cloud.
     * @param[in,out] lidar_meas Synced package (IMU buffer consumed).
     * @param[in,out] state ESIKF state (pose/vel/bias updated).
     * @param[out] undistorted_cloud Undistorted body-frame cloud.
     */
    void Process(LidarMeasureGroup& lidar_meas, StatesGroup& state,
                 sensor::PointCloudXYZI::Ptr undistorted_cloud);

    /**
     * @brief Undistort points of one sweep into the end-of-scan pose.
     * @param[in,out] lidar_meas Package with IMU samples for the sweep.
     * @param[in,out] state State used for forward/backward prop.
     * @param[out] cloud_out Undistorted cloud.
     */
    void UndistortPcl(LidarMeasureGroup& lidar_meas, StatesGroup& state,
                      sensor::PointCloudXYZI& cloud_out);

    std::ofstream imu_log_;       /**< @brief Optional IMU log stream. */
    double imu_mean_acc_norm = 0; /**< @brief Mean |acc| during init. */
    Eigen::Vector3d unbiased_gyr =
        Eigen::Vector3d::Zero(); /**< @brief Last debiased gyro. */

    Eigen::Vector3d cov_acc =
        Eigen::Vector3d(0.1, 0.1, 0.1); /**< @brief Acc noise diag. */
    Eigen::Vector3d cov_gyr =
        Eigen::Vector3d(0.1, 0.1, 0.1); /**< @brief Gyro noise diag. */
    Eigen::Vector3d cov_bias_gyr =
        Eigen::Vector3d(0.1, 0.1, 0.1); /**< @brief Gyro bias RW. */
    Eigen::Vector3d cov_bias_acc =
        Eigen::Vector3d(0.1, 0.1, 0.1); /**< @brief Acc bias RW. */
    double cov_inv_expo = 0.2;          /**< @brief Inverse-exposure noise. */
    double first_lidar_time = 0;        /**< @brief Stamp of first LiDAR (s). */
    bool imu_time_init = false; /**< @brief True after first IMU stamp. */
    bool need_imu_init = true;  /**< @brief Still collecting init samples. */

private:
    void ImuInit(const MeasureGroup& meas, StatesGroup& state, int& N);
    void ForwardWithoutImu(LidarMeasureGroup& meas, StatesGroup& state,
                           sensor::PointCloudXYZI& cloud_out);
    void CompensateCloudConstantVelocity(sensor::PointCloudXYZI& cloud_out,
                                         const StatesGroup& state,
                                         double cloud_end_offset_time);
    void PrepareLioPendingCloud(LidarMeasureGroup& lidar_meas,
                                StatesGroup& state);
    void ForwardPropagateImu(LidarMeasureGroup& lidar_meas,
                             StatesGroup& state_inout,
                             const std::deque<sensor::ImuData>& v_imu,
                             double prop_beg_time, double prop_end_time,
                             Eigen::Vector3d& acc_imu,
                             Eigen::Vector3d& angvel_avr,
                             Eigen::Vector3d& vel_imu,
                             Eigen::Vector3d& pos_imu, Eigen::Matrix3d& R_imu);
    void PropagateImuInterval(StatesGroup& state_inout, double dt, double offs_t,
                              const Eigen::Vector3d& angvel_avr,
                              const Eigen::Vector3d& acc_avr,
                              Eigen::Vector3d& acc_imu,
                              Eigen::Vector3d& vel_imu,
                              Eigen::Vector3d& pos_imu, Eigen::Matrix3d& R_imu);
    void BackwardUndistortLioPoints(StatesGroup& state_inout,
                                    sensor::PointCloudXYZI& pcl_out);

    sensor::PointCloudXYZI pending_cloud_;
    sensor::ImuData last_imu_;
    std::vector<Pose6D> imu_pose_;
    Eigen::Matrix3d lidar_rot_to_imu_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d lidar_offset_to_imu_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_acc_ = Eigen::Vector3d(0, 0, -1.0);
    Eigen::Vector3d mean_gyr_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d angvel_last_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_s_last_ = Eigen::Vector3d::Zero();
    double last_prop_end_time_ = 0;
    double time_last_scan_ = 0;
    int init_iter_num_ = 1;
    int max_init_count_ = 20;
    bool is_first_frame_ = true;
    bool enable_imu_ = true;
    bool enable_gravity_estimate_ = true;
    bool enable_bias_estimate_ = true;
    bool enable_exposure_estimate_ = true;
    bool enable_motion_compensate_ = true;
    std::string log_dir_;
};

}  // namespace frontend
}  // namespace argus

#endif  // ARGUS_FRONTEND_IMU_PROCESS_HPP_
