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

#include "autonomy/localization/atlas/util/constant.hpp"

#include <algorithm>
#include <cmath>

#include "autolink/common/log.hpp"

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

void ImuProcess::ResetImuInit() {
    mean_acc_ = Vec3_t(0.0, 0.0, -1.0);
    mean_gyr_ = Vec3_t::Zero();
    cov_acc_ = Vec3_t(0.1, 0.1, 0.1);
    cov_gyr_ = Vec3_t(0.1, 0.1, 0.1);
    init_iter_num_ = 1;
    b_first_frame_ = true;
    imu_need_init_ = true;
    acc_scale_factor_ = 1.0;
    filter_.Reset();
}

void ImuProcess::FilterMeasure(MeasureGroup* meas) {
    if (!meas || !use_imu_filter_ || meas->imu.empty()) {
        return;
    }
    for (auto& s : meas->imu) {
        s = filter_.Filter(s);
    }
}

bool ImuProcess::TryImuInit(LocalEstimator* est, const MeasureGroup& meas) {
    if (!imu_need_init_) {
        return true;
    }
    if (!est || meas.imu.empty()) {
        return false;
    }

    if (b_first_frame_) {
        init_iter_num_ = 1;
        b_first_frame_ = false;
        mean_acc_ = meas.imu.front().acc;
        mean_gyr_ = meas.imu.front().gyro;
    }

    int N = init_iter_num_;
    for (const auto& imu : meas.imu) {
        const Vec3_t cur_acc = imu.acc;
        const Vec3_t cur_gyr = imu.gyro;
        mean_acc_ += (cur_acc - mean_acc_) / static_cast<double>(N);
        mean_gyr_ += (cur_gyr - mean_gyr_) / static_cast<double>(N);
        cov_acc_ = cov_acc_ * (N - 1.0) / N +
                   (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) *
                       (N - 1.0) / (N * N);
        cov_gyr_ = cov_gyr_ * (N - 1.0) / N +
                   (cur_gyr - mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) *
                       (N - 1.0) / (N * N);
        ++N;
    }
    init_iter_num_ = N;

    if (init_iter_num_ <= max_init_count_) {
        AINFO_EVERY(10) << "Atlas IMUInit waiting ... " << init_iter_num_ << "/"
                        << max_init_count_;
        return false;
    }

    const double mean_norm = mean_acc_.norm();
    if (mean_norm < 1e-6) {
        AWARN << "Atlas IMUInit: mean_acc near zero, keep waiting";
        return false;
    }

    // Gravity from −mean_acc normalized * g; bg ≈ mean_gyr (lightning IMUInit).
    const Vec3_t gravity =
        -mean_acc_ / mean_norm * util::constant::kGRAVITY;
    const Mat44_t Twb = est->T_wb();
    est->Reset(Twb);
    est->set_gravity(gravity);
    est->set_gyro_bias(mean_gyr_);
    est->SetImuInitCovariance();

    imu_need_init_ = false;

    // Lightning: g-unit IMU → scale by g; already m/s² → 1.0.
    const double mean_acc_norm = mean_acc_.norm();
    if (mean_acc_norm > 0.5 && mean_acc_norm < 1.5) {
        acc_scale_factor_ = util::constant::kGRAVITY;
    } else if (mean_acc_norm > 7.0 && mean_acc_norm < 12.0) {
        acc_scale_factor_ = 1.0;
    } else {
        acc_scale_factor_ = 1.0;
        AWARN << "Atlas IMUInit: abnormal mean_acc norm=" << mean_acc_norm
              << ", keep acc_scale=1";
    }

    AINFO << "Atlas IMUInit done, bg=" << mean_gyr_.transpose()
          << " grav=" << gravity.transpose()
          << " mean_acc=" << mean_acc_.transpose()
          << " acc_scale=" << acc_scale_factor_ << " N=" << init_iter_num_;
    return true;
}

void BuildImuPoses(LocalEstimator* est, const MeasureGroup& meas,
                   std::vector<ImuPoseSample>* imu_poses, double acc_scale) {
    if (!imu_poses) {
        return;
    }
    imu_poses->clear();
    if (!est || meas.imu.empty()) {
        return;
    }
    const double scale = (acc_scale > 1e-9) ? acc_scale : 1.0;

    const Vec3_t gravity = est->gravity();
    const double t_beg = meas.lidar_begin_time;
    const double t_end = (meas.lidar_end_time > t_beg)
                             ? meas.lidar_end_time
                             : t_beg;

    // Seed sample at scan begin (current estimator state before this window).
    {
        const auto& front = meas.imu.front();
        const Vec3_t acc = front.acc * scale;
        const Mat33_t R = est->T_wb().block<3, 3>(0, 0);
        const Vec3_t acc_w = R * acc + gravity;
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
        const Vec3_t acc = 0.5 * (head.acc + tail.acc) * scale;
        est->PredictImu(dt, gyro, acc);
        const Mat33_t R = est->T_wb().block<3, 3>(0, 0);
        const Vec3_t acc_w = R * acc + gravity;
        const double offs = tail.timestamp - t_beg;
        imu_poses->push_back(
            SampleFromEstimator(*est, offs, gyro, acc_w));
    }

    // Align to lidar end if last IMU is short of scan end.
    if (!meas.imu.empty() && t_end > meas.imu.back().timestamp + 1e-6) {
        const double dt = t_end - meas.imu.back().timestamp;
        if (dt > 0.0 && dt < 0.5) {
            const auto& last = meas.imu.back();
            const Vec3_t acc = last.acc * scale;
            est->PredictImu(dt, last.gyro, acc);
            const Mat33_t R = est->T_wb().block<3, 3>(0, 0);
            const Vec3_t acc_w = R * acc + gravity;
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
