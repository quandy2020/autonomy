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

#include "autonomy/localization/atla2/backend/graph/graph_optimizer.hpp"

#include "Eigen/Geometry"
#include "ceres/ceres.h"

#include "autonomy/localization/atla2/backend/factors/imu_factor.hpp"
#include "autonomy/localization/atla2/backend/factors/pose_param.hpp"
#include "autonomy/localization/atla2/backend/factors/relative_pose_factor.hpp"
#include "autonomy/localization/atla2/backend/factors/visual_factor.hpp"
#include "autonomy/localization/atla2/common/time.hpp"

namespace autonomy::localization::atla2 {

bool GraphOptimizer::Init(const Atla2Config& cfg) {
  cfg_ = cfg;
  window_.SetMaxSize(cfg.window_size);
  cam_.fx = cfg.cam_fx;
  cam_.fy = cfg.cam_fy;
  cam_.cx = cfg.cam_cx;
  cam_.cy = cfg.cam_cy;
  max_iterations_ = cfg.ceres_max_iterations;
  pose_weight_ = cfg.ceres_pose_weight;
  imu_weight_ = cfg.ceres_imu_weight;
  visual_weight_ = cfg.ceres_visual_weight;
  Reset();
  return true;
}

void GraphOptimizer::Reset() {
  window_.Clear();
  state_ = OdometryResult{};
  has_state_ = false;
}

WindowFrame GraphOptimizer::MakeFrame(const OdometryResult& odom, const SensorData* data) {
  WindowFrame f;
  f.t = odom.t;
  Se3ToPq(odom.pose, f.p, f.aa);
  Se3ToPq(odom.pose, f.prior_p, f.prior_aa);
  Vec3ToArray(odom.velocity, f.v);

  if (!window_.frames().empty()) {
    const auto& prev = window_.frames().back();
    const SE3 Ti = PqToSe3(prev.prior_p, prev.prior_aa);
    const SE3 Tij = Se3Inverse(Ti) * odom.pose;
    Se3ToPq(Tij, f.meas_dp, f.meas_daa);
    f.has_rel = true;

    if (data && data->has_imu && data->imu.size() >= 2) {
      ImuPreintegration preint;
      preint.Reset(prev.t);
      const Vec3 ba = ArrayToVec3(window_.ba());
      const Vec3 bg = ArrayToVec3(window_.bg());
      for (const auto& s : data->imu) {
        if (s.t < prev.t) {
          continue;
        }
        if (s.t > odom.t) {
          break;
        }
        preint.Push(s, ba, bg);
      }
      preint.Finish(odom.t);
      if (preint.Dt() > 1e-4) {
        Vec3ToArray(preint.DeltaP(), f.dP);
        Vec3ToArray(preint.DeltaV(), f.dV);
        const Eigen::AngleAxisd A(preint.DeltaR());
        const Vec3 w = A.angle() * A.axis();
        f.dR_aa[0] = w.x();
        f.dR_aa[1] = w.y();
        f.dR_aa[2] = w.z();
        f.dt = preint.Dt();
        f.has_imu = true;
      }
    }
  }

  for (const auto& lm : odom.landmarks) {
    if (lm.id < 0) {
      continue;
    }
    if (lm.has_uv) {
      f.observations.emplace_back(lm.id, lm.uv);
    }
    window_.GetOrCreateLandmark(lm.id, lm.position);
  }
  return f;
}

bool GraphOptimizer::Update(const OdometryResult& odom) {
  return UpdateWithSensors(odom, SensorData{});
}

bool GraphOptimizer::UpdateWithSensors(const OdometryResult& odom, const SensorData& data) {
  if (!odom.valid) {
    return false;
  }
  WindowFrame frame = MakeFrame(odom, data.has_imu ? &data : nullptr);
  window_.Push(std::move(frame));

  if (window_.frames().size() >= 2) {
    if (!Optimize()) {
      // Keep last frontend pose if solve fails.
      state_ = odom;
      has_state_ = true;
      return true;
    }
  } else {
    state_ = odom;
    has_state_ = true;
    return true;
  }

  const auto& latest = window_.frames().back();
  state_.t = latest.t;
  state_.pose = PqToSe3(latest.p, latest.aa);
  state_.velocity = ArrayToVec3(latest.v);
  state_.accel_bias = ArrayToVec3(window_.ba());
  state_.gyro_bias = ArrayToVec3(window_.bg());
  state_.landmarks = odom.landmarks;
  state_.local_map = odom.local_map;
  state_.cov.matrix = Mat66::Identity() * 0.01;
  state_.valid = true;
  has_state_ = true;
  return true;
}

bool GraphOptimizer::Optimize() {
  auto& frames = window_.frames();
  if (frames.size() < 2) {
    return false;
  }

  ceres::Problem problem;
  ceres::LossFunction* loss = new ceres::HuberLoss(1.0);

  for (auto& f : frames) {
    problem.AddParameterBlock(f.p, 3);
    problem.AddParameterBlock(f.aa, 3);
    problem.AddParameterBlock(f.v, 3);
  }
  // Fix first pose gauge.
  problem.SetParameterBlockConstant(frames.front().p);
  problem.SetParameterBlockConstant(frames.front().aa);

  double g[3] = {gravity_.x(), gravity_.y(), gravity_.z()};

  for (size_t i = 1; i < frames.size(); ++i) {
    auto& fi = frames[i - 1];
    auto& fj = frames[i];

    if (fj.has_rel) {
      problem.AddResidualBlock(
          RelativePoseFactor::Create(fj.meas_dp, fj.meas_daa, pose_weight_), loss, fi.p, fi.aa,
          fj.p, fj.aa);
    }

    if (fj.has_imu && fj.dt > 1e-4) {
      problem.AddResidualBlock(
          ImuFactor::Create(fj.dP, fj.dV, fj.dR_aa, fj.dt, g, imu_weight_), loss, fi.p, fi.aa,
          fi.v, fj.p, fj.aa, fj.v);
    }
  }

  // Visual reprojection for landmarks observed in window.
  for (auto& f : frames) {
    for (const auto& obs : f.observations) {
      auto* lm = window_.GetOrCreateLandmark(obs.first, Vec3::Zero());
      problem.AddParameterBlock(lm->xyz, 3);
      problem.AddResidualBlock(
          ReprojectionFactor::Create(obs.second.x(), obs.second.y(), cam_.fx, cam_.fy, cam_.cx,
                                     cam_.cy, visual_weight_),
          loss, f.p, f.aa, lm->xyz);
    }
  }

  ceres::Solver::Options options;
  options.max_num_iterations = max_iterations_;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false;
  options.num_threads = 1;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  return summary.IsSolutionUsable();
}

bool GraphOptimizer::GetState(OdometryResult* out) const {
  if (!out || !has_state_) {
    return false;
  }
  *out = state_;
  return true;
}

}  // namespace autonomy::localization::atla2
