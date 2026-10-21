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

#include "argus/frontend/imu_process.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>

#include "argus/common/math/so3.hpp"

namespace argus {
namespace frontend {
namespace {

using math::Exp;
using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;

}  // namespace

ImuProcess::ImuProcess()
    : is_first_frame_(true), need_imu_init(true) {
    init_iter_num_ = 1;
    cov_acc = V3D(0.1, 0.1, 0.1);
    cov_gyr = V3D(0.1, 0.1, 0.1);
    cov_bias_gyr = V3D(0.1, 0.1, 0.1);
    cov_bias_acc = V3D(0.1, 0.1, 0.1);
    cov_inv_expo = 0.2;
    mean_acc_ = V3D(0, 0, -1.0);
    mean_gyr_ = V3D(0, 0, 0);
    angvel_last_ = V3D::Zero();
    acc_s_last_ = V3D::Zero();
    lidar_offset_to_imu_ = V3D::Zero();
    lidar_rot_to_imu_ = M3D::Identity();
    last_imu_ = sensor::ImuData();
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() {
    std::cerr << "Reset ImuProcess" << std::endl;
    mean_acc_ = V3D(0, 0, -1.0);
    mean_gyr_ = V3D(0, 0, 0);
    angvel_last_ = V3D::Zero();
    acc_s_last_ = V3D::Zero();
    need_imu_init = true;
    imu_time_init = false;
    is_first_frame_ = true;
    init_iter_num_ = 1;
    imu_pose_.clear();
    pending_cloud_.clear();
    last_imu_ = sensor::ImuData();
    last_prop_end_time_ = 0.0;
    time_last_scan_ = 0.0;
    first_lidar_time = 0.0;
}

void ImuProcess::DisableImu() {
    enable_imu_ = false;
    need_imu_init = false;
}

void ImuProcess::DisableGravityEstimate() {
    enable_gravity_estimate_ = false;
}

void ImuProcess::DisableBiasEstimate() {
    enable_bias_estimate_ = false;
}

void ImuProcess::DisableExposureEstimate() {
    enable_exposure_estimate_ = false;
}

void ImuProcess::SetExtrinsic(const Eigen::Matrix4d& T) {
    lidar_offset_to_imu_ = T.block<3, 1>(0, 3);
    lidar_rot_to_imu_ = T.block<3, 3>(0, 0);
}

void ImuProcess::SetExtrinsic(const Eigen::Vector3d& transl) {
    lidar_offset_to_imu_ = transl;
    lidar_rot_to_imu_.setIdentity();
}

void ImuProcess::SetExtrinsic(const Eigen::Vector3d& transl,
                              const Eigen::Matrix3d& rot) {
    lidar_offset_to_imu_ = transl;
    lidar_rot_to_imu_ = rot;
}

void ImuProcess::SetGyrCovScale(const Eigen::Vector3d& scaler) {
    cov_gyr = scaler;
}

void ImuProcess::SetAccCovScale(const Eigen::Vector3d& scaler) {
    cov_acc = scaler;
}

void ImuProcess::SetGyrBiasCov(const Eigen::Vector3d& b_g) {
    cov_bias_gyr = b_g;
}

void ImuProcess::SetInvExpoCov(double inv_expo) {
    cov_inv_expo = inv_expo;
}

void ImuProcess::SetAccBiasCov(const Eigen::Vector3d& b_a) {
    cov_bias_acc = b_a;
}

void ImuProcess::SetImuInitFrameNum(int num) {
    max_init_count_ = num;
}

void ImuProcess::SetLogDir(const std::string& dir) {
    log_dir_ = dir;
}

void ImuProcess::SetMotionCompensate(bool enable) {
    enable_motion_compensate_ = enable;
}

void ImuProcess::ImuInit(const MeasureGroup& meas, StatesGroup& state_inout,
                         int& N) {
    /** 1. initializing the gravity, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurenments to unit gravity **/
    // Progress is noisy on the hot path; keep silent in production.
    V3D cur_acc, cur_gyr;

    if (is_first_frame_) {
        Reset();
        N = 1;
        is_first_frame_ = false;
        mean_acc_ = meas.imu.front().linear_acceleration;
        mean_gyr_ = meas.imu.front().angular_velocity;
        // first_lidar_time = meas.lidar_frame_beg_time;
        // cout<<"init acc norm: "<<mean_acc_.norm()<<std::endl;
    }

    for (const auto& imu : meas.imu) {
        cur_acc = imu.linear_acceleration;
        cur_gyr = imu.angular_velocity;

        mean_acc_ += (cur_acc - mean_acc_) / N;
        mean_gyr_ += (cur_gyr - mean_gyr_) / N;

        // cov_acc = cov_acc * (N - 1.0) / N + (cur_acc -
        // mean_acc_).cwiseProduct(cur_acc - mean_acc_) * (N - 1.0) / (N * N);
        // cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr -
        // mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) * (N - 1.0) / (N * N);

        // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc_.norm()<<std::endl;

        N++;
    }
    imu_mean_acc_norm = mean_acc_.norm();
    state_inout.gravity = -mean_acc_ / mean_acc_.norm() * kGravity;
    state_inout.rot_end =
        M3D::Identity();  // Exp(mean_acc_.cross(V3D(0, 0, -1 / scale_gravity)));
    state_inout.bias_g = V3D::Zero();  // mean_gyr_;

    last_imu_ = meas.imu.back();
}

void ImuProcess::CompensateCloudConstantVelocity(
    sensor::PointCloudXYZI& pcl_out, const StatesGroup& state_inout,
    double pcl_end_offset_time) {
    auto it_pcl = pcl_out.points.end() - 1;
    double dt_j = 0.0;
    for (; it_pcl != pcl_out.points.begin(); it_pcl--) {
        dt_j = pcl_end_offset_time - it_pcl->curvature / double(1000);
        M3D R_jk(Exp(state_inout.bias_g, -dt_j));
        V3D P_j(it_pcl->x, it_pcl->y, it_pcl->z);
        V3D p_jk =
            -state_inout.rot_end.transpose() * state_inout.vel_end * dt_j;
        V3D P_compensate = R_jk * P_j + p_jk;
        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);
    }
}

void ImuProcess::ForwardWithoutImu(LidarMeasureGroup& meas,
                                   StatesGroup& state_inout,
                                   sensor::PointCloudXYZI& pcl_out) {
    pcl_out = *(meas.lidar);
    /*** sort point clouds by offset time ***/
    const double& pcl_beg_time = meas.lidar_frame_beg_time;
    std::sort(pcl_out.points.begin(), pcl_out.points.end(), TimeListLess);
    const double& pcl_end_time =
        pcl_beg_time + pcl_out.points.back().curvature / double(1000);
    meas.last_lio_update_time = pcl_end_time;
    const double& pcl_end_offset_time =
        pcl_out.points.back().curvature / double(1000);

    MD(kDimState, kDimState) F_x, cov_w;
    double dt = 0;

    if (is_first_frame_) {
        dt = 0.1;
        is_first_frame_ = false;
    } else {
        dt = pcl_beg_time - time_last_scan_;
    }

    time_last_scan_ = pcl_beg_time;

    /* covariance propagation */
    M3D Exp_f = Exp(state_inout.bias_g, dt);

    F_x.setIdentity();
    cov_w.setZero();

    F_x.block<3, 3>(0, 0) = Exp(state_inout.bias_g, -dt);
    F_x.block<3, 3>(0, 10) = M3D::Identity() * dt;
    F_x.block<3, 3>(3, 7) = M3D::Identity() * dt;

    cov_w.block<3, 3>(10, 10).diagonal() =
        cov_gyr * dt * dt;  // for omega in constant model
    cov_w.block<3, 3>(7, 7).diagonal() =
        cov_acc * dt * dt;  // for velocity in constant model

    state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;
    state_inout.rot_end = state_inout.rot_end * Exp_f;
    state_inout.pos_end = state_inout.pos_end + state_inout.vel_end * dt;

    if (enable_motion_compensate_) {
        CompensateCloudConstantVelocity(pcl_out, state_inout,
                                        pcl_end_offset_time);
    }
}

void ImuProcess::PrepareLioPendingCloud(LidarMeasureGroup& lidar_meas,
                                        StatesGroup& state_inout) {
    if (lidar_meas.ekf_phase != kLio) {
        return;
    }
    pending_cloud_.resize(lidar_meas.cloud_proc_cur->points.size());
    pending_cloud_ = *(lidar_meas.cloud_proc_cur);
    lidar_meas.lidar_scan_index_now = 0;
    imu_pose_.push_back(SetPose6D(0.0, acc_s_last_, angvel_last_,
                                  state_inout.vel_end, state_inout.pos_end,
                                  state_inout.rot_end));
}

void ImuProcess::PropagateImuInterval(
    StatesGroup& state_inout, double dt, double offs_t,
    const Eigen::Vector3d& angvel_avr, const Eigen::Vector3d& acc_avr,
    Eigen::Vector3d& acc_imu, Eigen::Vector3d& vel_imu,
    Eigen::Vector3d& pos_imu, Eigen::Matrix3d& R_imu) {
    MD(kDimState, kDimState) F_x, cov_w;
    M3D acc_avr_skew;
    M3D Exp_f = Exp(angvel_avr, dt);
    acc_avr_skew << SKEW_SYM_MATRX(acc_avr);

    F_x.setIdentity();
    cov_w.setZero();

    F_x.block<3, 3>(0, 0) = Exp(angvel_avr, -dt);
    if (enable_bias_estimate_)
        F_x.block<3, 3>(0, 10) = -M3D::Identity() * dt;
    F_x.block<3, 3>(3, 7) = M3D::Identity() * dt;
    F_x.block<3, 3>(7, 0) = -R_imu * acc_avr_skew * dt;
    if (enable_bias_estimate_)
        F_x.block<3, 3>(7, 13) = -R_imu * dt;
    if (enable_gravity_estimate_)
        F_x.block<3, 3>(7, 16) = M3D::Identity() * dt;

    if (enable_exposure_estimate_)
        cov_w(6, 6) = cov_inv_expo * dt * dt;
    cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt;
    cov_w.block<3, 3>(7, 7) =
        R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
    cov_w.block<3, 3>(10, 10).diagonal() = cov_bias_gyr * dt * dt;
    cov_w.block<3, 3>(13, 13).diagonal() = cov_bias_acc * dt * dt;

    state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;

    R_imu = R_imu * Exp_f;
    acc_imu = R_imu * acc_avr + state_inout.gravity;
    pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
    vel_imu = vel_imu + acc_imu * dt;

    angvel_last_ = angvel_avr;
    acc_s_last_ = acc_imu;
    imu_pose_.push_back(
        SetPose6D(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
}

void ImuProcess::ForwardPropagateImu(
    LidarMeasureGroup& lidar_meas, StatesGroup& state_inout,
    const std::deque<sensor::ImuData>& v_imu, double prop_beg_time,
    double prop_end_time, Eigen::Vector3d& acc_imu,
    Eigen::Vector3d& angvel_avr, Eigen::Vector3d& vel_imu,
    Eigen::Vector3d& pos_imu, Eigen::Matrix3d& R_imu) {
    switch (lidar_meas.ekf_phase) {
        case kWait:
            break;
        case kLo:
        case kLio:
        case kVio: {
            double dt = 0;
            double offs_t = 0;
            for (int i = 0; i < static_cast<int>(v_imu.size()) - 1; i++) {
                const auto& head = v_imu[i];
                const auto& tail = v_imu[i + 1];

                if (tail.timestamp < prop_beg_time)
                    continue;

                angvel_avr =
                    0.5 * (head.angular_velocity + tail.angular_velocity);
                V3D acc_avr =
                    0.5 * (head.linear_acceleration + tail.linear_acceleration);

                imu_log_ << std::setw(10) << head.timestamp - first_lidar_time
                         << " " << angvel_avr.transpose() << " "
                         << acc_avr.transpose() << std::endl;

                angvel_avr -= state_inout.bias_g;
                acc_avr =
                    acc_avr * kGravity / mean_acc_.norm() - state_inout.bias_a;

                if (head.timestamp < prop_beg_time) {
                    dt = tail.timestamp - last_prop_end_time_;
                    offs_t = tail.timestamp - prop_beg_time;
                } else if (i != static_cast<int>(v_imu.size()) - 2) {
                    dt = tail.timestamp - head.timestamp;
                    offs_t = tail.timestamp - prop_beg_time;
                } else {
                    dt = prop_end_time - head.timestamp;
                    offs_t = prop_end_time - prop_beg_time;
                }

                PropagateImuInterval(state_inout, dt, offs_t, angvel_avr,
                                     acc_avr, acc_imu, vel_imu, pos_imu, R_imu);
            }
            lidar_meas.last_lio_update_time = prop_end_time;
            break;
        }
    }
}

void ImuProcess::BackwardUndistortLioPoints(StatesGroup& state_inout,
                                            sensor::PointCloudXYZI& pcl_out) {
    auto it_pcl = pending_cloud_.points.end() - 1;
    M3D extR_Ri(lidar_rot_to_imu_.transpose() * state_inout.rot_end.transpose());
    V3D exrR_extT(lidar_rot_to_imu_.transpose() * lidar_offset_to_imu_);
    V3D acc_imu, angvel_avr, vel_imu, pos_imu;
    M3D R_imu;
    for (auto it_kp = imu_pose_.end() - 1; it_kp != imu_pose_.begin();
         it_kp--) {
        auto head = it_kp - 1;
        R_imu << MAT_FROM_ARRAY(head->rot);
        acc_imu << VEC_FROM_ARRAY(head->acc);
        vel_imu << VEC_FROM_ARRAY(head->vel);
        pos_imu << VEC_FROM_ARRAY(head->pos);
        angvel_avr << VEC_FROM_ARRAY(head->gyr);

        for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--) {
            const double dt =
                it_pcl->curvature / double(1000) - head->offset_time;
            M3D R_i(R_imu * Exp(angvel_avr, dt));
            V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt -
                     state_inout.pos_end);
            V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            V3D P_compensate =
                (extR_Ri *
                     (R_i * (lidar_rot_to_imu_ * P_i + lidar_offset_to_imu_) +
                      T_ei) -
                 exrR_extT);
            it_pcl->x = P_compensate(0);
            it_pcl->y = P_compensate(1);
            it_pcl->z = P_compensate(2);
            if (it_pcl == pending_cloud_.points.begin())
                break;
        }
    }
    pcl_out = pending_cloud_;
    pending_cloud_.clear();
    imu_pose_.clear();
}

void ImuProcess::UndistortPcl(LidarMeasureGroup& lidar_meas,
                              StatesGroup& state_inout,
                              sensor::PointCloudXYZI& pcl_out) {
    pcl_out.clear();
    /*** add the imu of the last frame-tail to the of current frame-head ***/
    MeasureGroup& meas = lidar_meas.measures.back();
    auto v_imu = meas.imu;
    v_imu.push_front(last_imu_);
    const double prop_beg_time = last_prop_end_time_;
    const double prop_end_time =
        lidar_meas.ekf_phase == kLio ? meas.lio_time : meas.vio_time;

    PrepareLioPendingCloud(lidar_meas, state_inout);

    /*** forward propagation at each imu point ***/
    V3D acc_imu(acc_s_last_), angvel_avr(angvel_last_),
        vel_imu(state_inout.vel_end), pos_imu(state_inout.pos_end);
    M3D R_imu(state_inout.rot_end);
    double tau;
    if (!imu_time_init) {
        tau = 1.0;
        imu_time_init = true;
    } else {
        tau = state_inout.inv_expo_time;
    }

    ForwardPropagateImu(lidar_meas, state_inout, v_imu, prop_beg_time,
                        prop_end_time, acc_imu, angvel_avr, vel_imu, pos_imu,
                        R_imu);

    state_inout.vel_end = vel_imu;
    state_inout.rot_end = R_imu;
    state_inout.pos_end = pos_imu;
    state_inout.inv_expo_time = tau;

    last_imu_ = v_imu.back();
    last_prop_end_time_ = prop_end_time;

    if (pending_cloud_.points.size() < 1)
        return;

    /*** undistort each lidar point (backward propagation), ONLY working for LIO
     * update ***/
    if (lidar_meas.ekf_phase == kLio) {
        BackwardUndistortLioPoints(state_inout, pcl_out);
    }
}

void ImuProcess::Process(LidarMeasureGroup& lidar_meas, StatesGroup& state,
                         sensor::PointCloudXYZI::Ptr undistorted_cloud) {
    assert(lidar_meas.lidar != nullptr);
    assert(undistorted_cloud != nullptr);
    if (!enable_imu_) {
        ForwardWithoutImu(lidar_meas, state, *undistorted_cloud);
        return;
    }

    MeasureGroup meas = lidar_meas.measures.back();

    if (need_imu_init) {
        if (meas.imu.empty()) {
            return;
        }
        ImuInit(meas, state, init_iter_num_);
        need_imu_init = true;
        last_imu_ = meas.imu.back();

        if (init_iter_num_ > max_init_count_) {
            need_imu_init = false;
            std::printf(
                "IMU Initials: Gravity: %.4f %.4f %.4f %.4f; acc covarience: "
                "%.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f \n",
                state.gravity[0], state.gravity[1], state.gravity[2],
                mean_acc_.norm(), cov_acc[0], cov_acc[1], cov_acc[2],
                cov_gyr[0], cov_gyr[1], cov_gyr[2]);
            std::printf(
                "IMU Initials: ba covarience: %.8f %.8f %.8f; bg covarience: "
                "%.8f %.8f %.8f",
                cov_bias_acc[0], cov_bias_acc[1], cov_bias_acc[2],
                cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2]);
            if (!log_dir_.empty()) {
                imu_log_.open(log_dir_ + "/imu.txt", std::ios::out);
            }
        }
        return;
    }

    UndistortPcl(lidar_meas, state, *undistorted_cloud);
}

}  // namespace frontend
}  // namespace argus
