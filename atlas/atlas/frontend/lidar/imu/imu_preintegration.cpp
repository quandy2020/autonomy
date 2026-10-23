/*
 * Copyright 2026 The Openbot Authors
 */

#include "frontend/lidar/imu/imu_preintegration.hpp"

#include "common/math/so3.hpp"

namespace atlas {
namespace vxslam {

double g_scale_gravity = 1.0;
Eigen::Matrix<double, 6, 6> g_noise_meas =
    Eigen::Matrix<double, 6, 6>::Identity();
Eigen::Matrix<double, 6, 6> g_noise_walk =
    Eigen::Matrix<double, 6, 6>::Identity();

ImuPreintegration::ImuPreintegration(const Eigen::Vector3d& bg1,
                                     const Eigen::Vector3d& ba1)
    : bg(bg1), ba(ba1) {}

void ImuPreintegration::PushImu(const std::deque<sensor::ImuData>& imu_batch) {
  imus.insert(imus.end(), imu_batch.begin(), imu_batch.end());
  if (imu_batch.size() < 2) {
    return;
  }
  for (size_t i = 1; i < imu_batch.size(); ++i) {
    const sensor::ImuData& imu1 = imu_batch[i - 1];
    const sensor::ImuData& imu2 = imu_batch[i];
    const double dt = imu2.time - imu1.time;
    Eigen::Vector3d cur_gyr =
        0.5 * (imu1.angular_velocity + imu2.angular_velocity);
    Eigen::Vector3d cur_acc =
        0.5 * (imu1.linear_acceleration + imu2.linear_acceleration);
    cur_gyr = cur_gyr - bg;
    cur_acc = cur_acc * g_scale_gravity - ba;
    AddImu(cur_gyr, cur_acc, dt);
  }
}

void ImuPreintegration::AddImu(Eigen::Vector3d cur_gyr, Eigen::Vector3d cur_acc,
                               double dt) {
  dtime += dt;
  const Eigen::Matrix3d R_inc = math::Exp(cur_gyr, dt);
  const Eigen::Matrix3d R_jr(Jr(cur_gyr * dt));
  const Eigen::Matrix3d R_dt = dt * R_delta;
  const Eigen::Matrix3d R_dt2_2 = 0.5 * dt * dt * R_delta;
  const Eigen::Matrix3d acc_skew = math::SkewSymmetric(cur_acc);

  p_ba = p_ba + v_ba * dt - R_dt2_2;
  p_bg = p_bg + v_bg * dt - R_dt2_2 * acc_skew * R_bg;
  v_ba = v_ba - R_dt;
  v_bg = v_bg - R_dt * acc_skew * R_bg;
  R_bg = R_inc.transpose() * R_bg - R_jr * dt;

  Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Identity();
  Eigen::Matrix<double, 9, 6> B = Eigen::Matrix<double, 9, 6>::Zero();
  A.block<3, 3>(0, 0) = R_inc.transpose();
  A.block<3, 3>(3, 0) = -R_dt2_2 * acc_skew;
  A.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt;
  A.block<3, 3>(6, 0) = -R_dt * acc_skew;
  B.block<3, 3>(0, 0) = R_jr * dt;
  B.block<3, 3>(3, 3) = R_dt2_2;
  B.block<3, 3>(6, 3) = R_dt;

  cov.block<9, 9>(0, 0) = A * cov.block<9, 9>(0, 0) * A.transpose() +
                          B * g_noise_meas * B.transpose();
  cov.block<6, 6>(9, 9) += g_noise_walk * dt;

  p_delta += v_delta * dt + R_dt2_2 * cur_acc;
  v_delta += R_dt * cur_acc;
  R_delta = R_delta * R_inc;
}

double ImuPreintegration::Evaluate(ImuState& st1, ImuState& st2,
                                   Eigen::MatrixXd& jtj, Eigen::VectorXd& gg,
                                   bool jac_enable) {
  Eigen::Matrix<double, kStateDim, kStateDim> joca =
      Eigen::Matrix<double, kStateDim, kStateDim>::Zero();
  Eigen::Matrix<double, kStateDim, kStateDim> jocb =
      Eigen::Matrix<double, kStateDim, kStateDim>::Zero();
  Eigen::Matrix<double, kStateDim, 1> rr =
      Eigen::Matrix<double, kStateDim, 1>::Zero();

  const Eigen::Matrix3d R_correct = R_delta * math::Exp(R_bg * dbg);
  const Eigen::Vector3d t_correct = p_delta + p_bg * dbg + p_ba * dba;
  const Eigen::Vector3d v_correct = v_delta + v_bg * dbg + v_ba * dba;

  const Eigen::Matrix3d res_r =
      R_correct.transpose() * st1.R.transpose() * st2.R;
  const Eigen::Vector3d exp_v =
      st1.R.transpose() * (st2.v - st1.v - dtime * st1.g);
  const Eigen::Vector3d res_v = exp_v - v_correct;
  const Eigen::Vector3d exp_t =
      st1.R.transpose() *
      (st2.p - st1.p - st1.v * dtime - 0.5 * dtime * dtime * st1.g);
  const Eigen::Vector3d res_t = exp_t - t_correct;
  const Eigen::Vector3d res_bg = st2.bg - st1.bg;
  const Eigen::Vector3d res_ba = st2.ba - st1.ba;

  rr.block<3, 1>(0, 0) = math::Log(res_r);
  rr.block<3, 1>(3, 0) = res_t;
  rr.block<3, 1>(6, 0) = res_v;
  rr.block<3, 1>(9, 0) = res_bg;
  rr.block<3, 1>(12, 0) = res_ba;

  const Eigen::Matrix<double, 15, 15> cov_inv = cov.inverse();
  if (jac_enable) {
    const Eigen::Matrix3d JR_inv = JrInv(res_r);
    joca.block<3, 3>(0, 0) = -JR_inv * st2.R.transpose() * st1.R;
    jocb.block<3, 3>(0, 0) = JR_inv;
    joca.block<3, 3>(0, 9) =
        -JR_inv * res_r.transpose() * Jr(R_bg * dbg) * R_bg;

    joca.block<3, 3>(3, 0) = math::SkewSymmetric(exp_t);
    joca.block<3, 3>(3, 3) = -st1.R.transpose();
    joca.block<3, 3>(3, 6) = -st1.R.transpose() * dtime;
    joca.block<3, 3>(3, 9) = -p_bg;
    joca.block<3, 3>(3, 12) = -p_ba;
    jocb.block<3, 3>(3, 3) = st1.R.transpose();

    joca.block<3, 3>(6, 0) = math::SkewSymmetric(exp_v);
    joca.block<3, 3>(6, 6) = -st1.R.transpose();
    joca.block<3, 3>(6, 9) = -v_bg;
    joca.block<3, 3>(6, 12) = -v_ba;
    jocb.block<3, 3>(6, 6) = st1.R.transpose();

    joca.block<3, 3>(9, 9) = -Eigen::Matrix3d::Identity();
    joca.block<3, 3>(12, 12) = -Eigen::Matrix3d::Identity();
    jocb.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
    jocb.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, kStateDim, 2 * kStateDim> joc;
    joc.block<kStateDim, kStateDim>(0, 0) = joca;
    joc.block<kStateDim, kStateDim>(0, kStateDim) = jocb;
    jtj = joc.transpose() * cov_inv * joc;
    gg = joc.transpose() * cov_inv * rr;
  }
  return rr.dot(cov_inv * rr);
}

double ImuPreintegration::EvaluateGravity(ImuState& st1, ImuState& st2,
                                          Eigen::MatrixXd& jtj,
                                          Eigen::VectorXd& gg,
                                          bool jac_enable) {
  Eigen::Matrix<double, kStateDim, kStateDim> joca =
      Eigen::Matrix<double, kStateDim, kStateDim>::Zero();
  Eigen::Matrix<double, kStateDim, kStateDim> jocb =
      Eigen::Matrix<double, kStateDim, kStateDim>::Zero();
  Eigen::Matrix<double, kStateDim, 1> rr =
      Eigen::Matrix<double, kStateDim, 1>::Zero();
  Eigen::Matrix<double, kStateDim, 3> jocg =
      Eigen::Matrix<double, kStateDim, 3>::Zero();

  const Eigen::Matrix3d R_correct = R_delta * math::Exp(R_bg * dbg);
  const Eigen::Vector3d t_correct = p_delta + p_bg * dbg + p_ba * dba;
  const Eigen::Vector3d v_correct = v_delta + v_bg * dbg + v_ba * dba;
  const Eigen::Matrix3d res_r =
      R_correct.transpose() * st1.R.transpose() * st2.R;
  const Eigen::Vector3d exp_v =
      st1.R.transpose() * (st2.v - st1.v - dtime * st1.g);
  const Eigen::Vector3d res_v = exp_v - v_correct;
  const Eigen::Vector3d exp_t =
      st1.R.transpose() *
      (st2.p - st1.p - st1.v * dtime - 0.5 * dtime * dtime * st1.g);
  const Eigen::Vector3d res_t = exp_t - t_correct;
  rr.block<3, 1>(0, 0) = math::Log(res_r);
  rr.block<3, 1>(3, 0) = res_t;
  rr.block<3, 1>(6, 0) = res_v;
  rr.block<3, 1>(9, 0) = st2.bg - st1.bg;
  rr.block<3, 1>(12, 0) = st2.ba - st1.ba;
  const Eigen::Matrix<double, 15, 15> cov_inv = cov.inverse();
  if (jac_enable) {
    const Eigen::Matrix3d JR_inv = JrInv(res_r);
    joca.block<3, 3>(0, 0) = -JR_inv * st2.R.transpose() * st1.R;
    jocb.block<3, 3>(0, 0) = JR_inv;
    joca.block<3, 3>(0, 9) =
        -JR_inv * res_r.transpose() * Jr(R_bg * dbg) * R_bg;
    joca.block<3, 3>(3, 0) = math::SkewSymmetric(exp_t);
    joca.block<3, 3>(3, 3) = -st1.R.transpose();
    joca.block<3, 3>(3, 6) = -st1.R.transpose() * dtime;
    joca.block<3, 3>(3, 9) = -p_bg;
    joca.block<3, 3>(3, 12) = -p_ba;
    jocb.block<3, 3>(3, 3) = st1.R.transpose();
    joca.block<3, 3>(6, 0) = math::SkewSymmetric(exp_v);
    joca.block<3, 3>(6, 6) = -st1.R.transpose();
    joca.block<3, 3>(6, 9) = -v_bg;
    joca.block<3, 3>(6, 12) = -v_ba;
    jocb.block<3, 3>(6, 6) = st1.R.transpose();
    joca.block<3, 3>(9, 9) = -Eigen::Matrix3d::Identity();
    joca.block<3, 3>(12, 12) = -Eigen::Matrix3d::Identity();
    jocb.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();
    jocb.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity();
    jocg.block<3, 3>(3, 0) = st1.R.transpose() * (-0.5 * dtime * dtime);
    jocg.block<3, 3>(6, 0) = st1.R.transpose() * (-dtime);
    Eigen::Matrix<double, kStateDim, 2 * kStateDim + 3> joc;
    joc.block<kStateDim, kStateDim>(0, 0) = joca;
    joc.block<kStateDim, kStateDim>(0, kStateDim) = jocb;
    joc.block<kStateDim, 3>(0, 2 * kStateDim) = jocg;
    jtj = joc.transpose() * cov_inv * joc;
    gg = joc.transpose() * cov_inv * rr;
  }
  return rr.dot(cov_inv * rr);
}

void ImuPreintegration::ApplyDelta(
    const Eigen::Matrix<double, kStateDim, 1>& dxi) {
  dbg_buf = dbg;
  dba_buf = dba;
  dbg += dxi.block<3, 1>(9, 0);
  dba += dxi.block<3, 1>(12, 0);
}

void ImuPreintegration::Merge(const ImuPreintegration& imu2) {
  p_bg += v_bg * imu2.dtime +
          R_delta * (imu2.p_bg - math::SkewSymmetric(imu2.p_delta) * R_bg);
  p_ba += v_ba * imu2.dtime + R_delta * imu2.p_ba;
  v_bg += R_delta * (imu2.v_bg - math::SkewSymmetric(imu2.v_delta) * R_bg);
  v_ba += R_delta * imu2.v_ba;
  R_bg = imu2.R_delta.transpose() * R_bg + imu2.R_bg;

  Eigen::Matrix<double, kStateDim, kStateDim> Ai =
      Eigen::Matrix<double, kStateDim, kStateDim>::Identity();
  Eigen::Matrix<double, kStateDim, kStateDim> Bi =
      Eigen::Matrix<double, kStateDim, kStateDim>::Identity();
  Ai.block<3, 3>(0, 0) = imu2.R_delta.transpose();
  Ai.block<3, 3>(3, 0) = -R_delta * math::SkewSymmetric(imu2.p_delta);
  Ai.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * imu2.dtime;
  Ai.block<3, 3>(6, 0) = -R_delta * math::SkewSymmetric(imu2.v_delta);
  Bi.block<3, 3>(3, 3) = R_delta;
  Bi.block<3, 3>(6, 6) = R_delta;
  cov = Ai * cov * Ai.transpose() + Bi * imu2.cov * Bi.transpose();

  p_delta += v_delta * imu2.dtime + R_delta * imu2.p_delta;
  v_delta += R_delta * imu2.v_delta;
  R_delta = R_delta * imu2.R_delta;
  dtime += imu2.dtime;
}

}  // namespace vxslam
}  // namespace atlas
