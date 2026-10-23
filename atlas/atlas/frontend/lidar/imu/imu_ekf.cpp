/*
 * Copyright 2026 The Openbot Authors
 */

#include "frontend/lidar/imu/imu_ekf.hpp"

#include "common/math/so3.hpp"
#include "glog/logging.h"

namespace atlas {
namespace vxslam {

void ImuEkf::ImuInit(const std::deque<sensor::ImuData>& imus) {
  for (const sensor::ImuData& imu : imus) {
    const Eigen::Vector3d cur_acc = imu.linear_acceleration;
    const Eigen::Vector3d cur_gyr = imu.angular_velocity;
    if (init_num != 0) {
      mean_acc += (cur_acc - mean_acc) / static_cast<double>(init_num);
      mean_gyr += (cur_gyr - mean_gyr) / static_cast<double>(init_num);
    } else {
      mean_acc = cur_acc;
      mean_gyr = cur_gyr;
      init_num = 1;
    }
    ++init_num;
  }
  if (!imus.empty()) {
    last_imu = imus.back();
  }
}

void ImuEkf::MotionBlur(ImuState* xc, sensor::PointCloudXYZI* pcl_in,
                        std::deque<sensor::ImuData>* imus) {
  CHECK(xc != nullptr && pcl_in != nullptr && imus != nullptr);
  imus->push_front(last_imu);
  if (last_pcl_end_time - pcl_beg_time > 0.01) {
    LOG(WARNING) << "LiDAR time regress: beg=" << pcl_beg_time
                 << " last_end=" << last_pcl_end_time;
  }

  imu_poses.clear();
  Eigen::Vector3d acc_imu = Eigen::Vector3d::Zero();
  Eigen::Vector3d angvel_avr = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_avr = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel_imu(xc->v);
  Eigen::Vector3d pos_imu(xc->p);
  Eigen::Matrix3d R_imu(xc->R);
  Eigen::Matrix<double, kStateDim, kStateDim> F_x;
  Eigen::Matrix<double, kStateDim, kStateDim> cov_w;

  for (auto it = imus->begin(); it + 1 != imus->end(); ++it) {
    const sensor::ImuData& head = *it;
    const sensor::ImuData& tail = *(it + 1);
    if (tail.time < last_pcl_end_time) {
      continue;
    }
    angvel_avr = 0.5 * (head.angular_velocity + tail.angular_velocity);
    acc_avr = 0.5 * (head.linear_acceleration + tail.linear_acceleration);
    angvel_avr -= xc->bg;
    acc_avr = acc_avr * scale_gravity - xc->ba;
    acc_imu = R_imu * acc_avr + xc->g;

    double cur_time = head.time;
    if (cur_time < last_pcl_end_time) {
      cur_time = last_pcl_end_time;
    }
    const double dt = tail.time - cur_time;
    const double offt = cur_time - pcl_beg_time;
    imu_poses.emplace_back(offt, R_imu, pos_imu, vel_imu, angvel_avr, acc_imu);

    const Eigen::Matrix3d acc_avr_skew = math::SkewSymmetric(acc_avr);
    const Eigen::Matrix3d Exp_f = math::Exp(angvel_avr, dt);
    F_x.setIdentity();
    cov_w.setZero();
    F_x.block<3, 3>(0, 0) = math::Exp(Eigen::Vector3d(-angvel_avr * dt));
    F_x.block<3, 3>(0, 9) = -Eigen::Matrix3d::Identity() * dt;
    F_x.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt;
    F_x.block<3, 3>(6, 0) = -R_imu * acc_avr_skew * dt;
    F_x.block<3, 3>(6, 12) = -R_imu * dt;
    cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt;
    cov_w.block<3, 3>(6, 6) =
        R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
    cov_w.block<3, 3>(9, 9).diagonal() = cov_bias_gyr * dt * dt;
    cov_w.block<3, 3>(12, 12).diagonal() = cov_bias_acc * dt * dt;
    xc->cov = F_x * xc->cov * F_x.transpose() + cov_w;

    pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
    vel_imu = vel_imu + acc_imu * dt;
    R_imu = R_imu * Exp_f;
  }

  const double imu_end_time = imus->back().time;
  const double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  const double dt = note * (pcl_end_time - imu_end_time);
  xc->v = vel_imu + note * acc_imu * dt;
  xc->R = R_imu * math::Exp(note * angvel_avr, dt);
  xc->p = pos_imu + note * vel_imu * dt + note * 0.5 * acc_imu * dt * dt;
  xc->t = pcl_end_time;

  sensor::ImuData imu1 = imus->front();
  sensor::ImuData imu2 = imus->back();
  imu1.time = last_pcl_end_time;
  imu2.time = pcl_end_time;
  last_imu = imus->back();
  last_pcl_end_time = pcl_end_time;
  imus->front() = imu1;
  imus->back() = imu2;

  if (point_notime || pcl_in->empty() || imu_poses.empty()) {
    return;
  }

  auto it_pcl = pcl_in->points.end() - 1;
  for (int i = static_cast<int>(imu_poses.size()) - 1; i >= 0; --i) {
    const ImuState& head = imu_poses[static_cast<size_t>(i)];
    R_imu = head.R;
    acc_imu = head.ba;  // stored acc_imu in ba slot of pose buffer
    vel_imu = head.v;
    pos_imu = head.p;
    angvel_avr = head.bg;  // stored angvel in bg slot
    for (; it_pcl->curvature > static_cast<float>(head.t); --it_pcl) {
      const double dtp = it_pcl->curvature - head.t;
      const Eigen::Matrix3d R_i = R_imu * math::Exp(angvel_avr, dtp);
      const Eigen::Vector3d T_ei =
          pos_imu + vel_imu * dtp + 0.5 * acc_imu * dtp * dtp - xc->p;
      const Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      const Eigen::Vector3d P_compensate =
          lid_rot_to_imu.transpose() *
          (xc->R.transpose() *
               (R_i * (lid_rot_to_imu * P_i + lid_offset_to_imu) + T_ei) -
           lid_offset_to_imu);
      it_pcl->x = static_cast<float>(P_compensate(0));
      it_pcl->y = static_cast<float>(P_compensate(1));
      it_pcl->z = static_cast<float>(P_compensate(2));
      if (it_pcl == pcl_in->points.begin()) {
        break;
      }
    }
  }
}

int ImuEkf::Process(ImuState* x_curr, sensor::PointCloudXYZI* pcl_in,
                    std::deque<sensor::ImuData>* imus) {
  CHECK(x_curr != nullptr && pcl_in != nullptr && imus != nullptr);
  if (!init_flag) {
    ImuInit(*imus);
    if (mean_acc.norm() < 2.0 && livox_imu_scale) {
      scale_gravity = kGravityMs2;
    }
    LOG(INFO) << "vxslam scale_gravity=" << scale_gravity
              << " mean_acc_norm=" << mean_acc.norm()
              << " init_num=" << init_num;
    x_curr->g = -mean_acc * scale_gravity;
    if (init_num > min_init_num) {
      init_flag = true;
    }
    last_pcl_end_time = pcl_end_time;
    return 0;
  }
  MotionBlur(x_curr, pcl_in, imus);
  return 1;
}

}  // namespace vxslam
}  // namespace atlas
