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

#include "autonomy/localization/atlas/frontend/lio/imu_process.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy::localization::atlas {
namespace frontend {
namespace lio {
namespace {

ImuPoseSample SampleFromEstimator(const LocalEstimator& est,
                                  double offset_t,
                                  const Vec3_t& angvel,
                                  const Vec3_t& acc_world) {
    ImuPoseSample s;
    s.offset_t = offset_t;
    const Mat44_t T = est.T_wb();
    s.R = T.block<3, 3>(0, 0);
    s.pos = T.block<3, 1>(0, 3);
    s.vel = est.velocity();
    s.angvel = angvel;
    s.acc = acc_world;
    return s;
}

void InterpolatePose(const ImuPoseSample& a, const ImuPoseSample& b, double t,
                     Mat33_t* R_i, Vec3_t* t_i) {
    const double dt = b.offset_t - a.offset_t;
    double alpha = 0.0;
    if (dt > 1e-12) {
        alpha = (t - a.offset_t) / dt;
    }
    alpha = std::clamp(alpha, 0.0, 1.0);
    // Linear pose + SLERP-ish via AngleAxis on relative rotation.
    const Mat33_t dR = a.R.transpose() * b.R;
    Eigen::AngleAxisd aa(dR);
    Mat33_t R_rel = Mat33_t::Identity();
    if (aa.angle() > 1e-12) {
        R_rel = Eigen::AngleAxisd(aa.angle() * alpha, aa.axis()).toRotationMatrix();
    }
    *R_i = a.R * R_rel;
    *t_i = (1.0 - alpha) * a.pos + alpha * b.pos;
}

}  // namespace

void BuildImuPoses(LocalEstimator* est,
                   const MeasureGroup& meas,
                   std::vector<ImuPoseSample>* imu_poses) {
    if (!imu_poses) {
        return;
    }
    imu_poses->clear();
    if (!est || meas.imu.empty()) {
        return;
    }

    const double t_beg = meas.lidar_begin_time;
    const double t_end = (meas.lidar_end_time > t_beg)
                             ? meas.lidar_end_time
                             : t_beg;

    // Seed sample at scan begin (current estimator state before this window).
    {
        const auto& front = meas.imu.front();
        const Mat33_t R = est->T_wb().block<3, 3>(0, 0);
        const Vec3_t acc_w =
            R * front.acc + Vec3_t(0.0, 0.0, -9.81);
        imu_poses->push_back(
            SampleFromEstimator(*est, 0.0, front.gyro, acc_w));
    }

    for (std::size_t i = 1; i < meas.imu.size(); ++i) {
        const auto& head = meas.imu[i - 1];
        const auto& tail = meas.imu[i];
        double dt = tail.timestamp - head.timestamp;
        if (dt <= 0.0 || dt > 0.5) {
            continue;
        }
        const Vec3_t gyro = 0.5 * (head.gyro + tail.gyro);
        const Vec3_t acc = 0.5 * (head.acc + tail.acc);
        est->PredictImu(dt, gyro, acc);
        const Mat33_t R = est->T_wb().block<3, 3>(0, 0);
        const Vec3_t acc_w = R * acc + Vec3_t(0.0, 0.0, -9.81);
        const double offs = tail.timestamp - t_beg;
        imu_poses->push_back(
            SampleFromEstimator(*est, offs, gyro, acc_w));
    }

    // Align to lidar end if last IMU is short of scan end.
    if (!meas.imu.empty() && t_end > meas.imu.back().timestamp + 1e-6) {
        const double dt = t_end - meas.imu.back().timestamp;
        if (dt > 0.0 && dt < 0.5) {
            const auto& last = meas.imu.back();
            est->PredictImu(dt, last.gyro, last.acc);
            const Mat33_t R = est->T_wb().block<3, 3>(0, 0);
            const Vec3_t acc_w =
                R * last.acc + Vec3_t(0.0, 0.0, -9.81);
            imu_poses->push_back(SampleFromEstimator(
                *est, t_end - t_beg, last.gyro, acc_w));
        }
    }
}

std::vector<Vec3_t> UndistortByImuTrajectory(
    const std::vector<Vec3_t>& points_body,
    const std::vector<double>& point_time_rel,
    const std::vector<ImuPoseSample>& imu_poses,
    const Mat44_t& T_imu_lidar) {
    if (points_body.empty() || imu_poses.size() < 2 ||
        point_time_rel.size() != points_body.size()) {
        return points_body;
    }

    const ImuPoseSample& end = imu_poses.back();
    const Mat33_t R_end = end.R;
    const Vec3_t t_end = end.pos;
    const double scan_dt =
        std::max(1e-6, end.offset_t - imu_poses.front().offset_t);

    const Mat33_t R_il = T_imu_lidar.block<3, 3>(0, 0);
    const Vec3_t t_il = T_imu_lidar.block<3, 1>(0, 3);
    const bool use_ext = !R_il.isIdentity(1e-12) || t_il.norm() > 1e-12;

    std::vector<Vec3_t> out;
    out.reserve(points_body.size());

    std::size_t kp = 1;
    for (std::size_t i = 0; i < points_body.size(); ++i) {
        double alpha = point_time_rel[i];
        if (!std::isfinite(alpha)) {
            out.push_back(points_body[i]);
            continue;
        }
        // Accept [0,1] fraction; if clearly an absolute offset (>>1), treat as sec.
        double t_off = alpha;
        if (alpha <= 1.0 + 1e-6) {
            if (alpha < 0.0) {
                alpha = 0.0;
            } else if (alpha > 1.0) {
                alpha = 1.0;
            }
            t_off = alpha * scan_dt + imu_poses.front().offset_t;
        }

        while (kp + 1 < imu_poses.size() &&
               imu_poses[kp].offset_t < t_off) {
            ++kp;
        }
        const std::size_t hi = std::min(kp, imu_poses.size() - 1);
        const std::size_t lo = (hi == 0) ? 0 : hi - 1;

        Mat33_t R_i;
        Vec3_t t_i;
        InterpolatePose(imu_poses[lo], imu_poses[hi], t_off, &R_i, &t_i);

        Vec3_t p = points_body[i];
        if (use_ext) {
            p = R_il * p + t_il;
        }
        // p_end = R_end^T * (R_i * p + t_i - t_end)
        Vec3_t p_end = R_end.transpose() * (R_i * p + t_i - t_end);
        if (use_ext) {
            p_end = R_il.transpose() * (p_end - t_il);
        }
        out.push_back(p_end);
    }
    return out;
}

}  // namespace lio
}  // namespace frontend
}  // namespace autonomy::localization::atlas
