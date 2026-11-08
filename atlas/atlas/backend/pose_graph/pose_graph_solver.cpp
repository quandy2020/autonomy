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

#include "backend/pose_graph/pose_graph_solver.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <cmath>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace atlas {
namespace backend {
namespace pose_graph {
namespace {

gtsam::Pose3 ToPose3(const transform::Rigid3d& pose) {
  const Eigen::Quaterniond rotation = pose.rotation();
  return gtsam::Pose3(
      gtsam::Rot3(rotation.w(), rotation.x(), rotation.y(), rotation.z()),
      gtsam::Point3(pose.translation()));
}

transform::Rigid3d FromPose3(const gtsam::Pose3& pose) {
  return transform::Rigid3d(Eigen::Vector3d(pose.translation()),
                            Eigen::Quaterniond(pose.rotation().matrix()));
}

gtsam::SharedNoiseModel MakeNoiseModel(double rotation_weight,
                                       double translation_weight) {
  gtsam::Vector6 sigmas;
  const double rotation_sigma =
      rotation_weight > 0.0 ? 1.0 / std::sqrt(rotation_weight) : 1e3;
  const double translation_sigma =
      translation_weight > 0.0 ? 1.0 / std::sqrt(translation_weight) : 1e3;
  sigmas << rotation_sigma, rotation_sigma, rotation_sigma, translation_sigma,
      translation_sigma, translation_sigma;
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

gtsam::SharedNoiseModel MakeNoiseModel(const BetweenFactor& factor) {
  if (!factor.diagonal_information_provided) {
    return MakeNoiseModel(factor.rotation_weight, factor.translation_weight);
  }
  gtsam::Vector6 sigmas;
  for (int axis = 0; axis < 6; ++axis) {
    const double information =
        factor.diagonal_information[static_cast<std::size_t>(axis)];
    sigmas[axis] = information > 0.0 ? 1.0 / std::sqrt(information) : 1e3;
  }
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

bool NewPosesHaveFactors(const FactorGraph& additions) {
  std::unordered_set<std::int64_t> constrained;
  for (const auto& entry : additions.GetPoses()) {
    if (entry.second.fixed) {
      constrained.insert(entry.first);
    }
  }
  for (const PriorFactor& factor : additions.GetPriorFactors()) {
    constrained.insert(factor.pose_id);
  }
  for (const BetweenFactor& factor : additions.GetBetweenFactors()) {
    constrained.insert(factor.from_id);
    constrained.insert(factor.to_id);
  }
  for (const auto& entry : additions.GetPoses()) {
    if (constrained.count(entry.first) == 0) {
      return false;
    }
  }
  return true;
}

}  // namespace

struct PoseGraphSolver::State {
  std::mutex mutex;
  std::unique_ptr<gtsam::ISAM2> isam;
  gtsam::NonlinearFactorGraph measurements;
  std::unordered_set<std::int64_t> pose_ids;
  double convergence_tolerance = 0.0;

  void Reset() {
    isam.reset();
    measurements = gtsam::NonlinearFactorGraph();
    pose_ids.clear();
    convergence_tolerance = 0.0;
  }
};

PoseGraphSolver::PoseGraphSolver() : state_(std::make_unique<State>()) {}

PoseGraphSolver::~PoseGraphSolver() = default;

UpdateResult PoseGraphSolver::Update(const FactorGraph& additions,
                                     const SolverOptions& options) {
  std::lock_guard<std::mutex> lock(state_->mutex);
  UpdateResult result;
  State& state = *state_;

  if ((state.pose_ids.empty() && additions.GetPoses().empty()) ||
      options.maximum_update_count <= 0 ||
      !std::isfinite(options.convergence_tolerance) ||
      options.convergence_tolerance <= 0.0 ||
      !additions.IsValid(state.pose_ids) || !NewPosesHaveFactors(additions)) {
    result.diagnostic_message = "invalid incremental pose graph input";
    return result;
  }
  for (const BetweenFactor& factor : additions.GetBetweenFactors()) {
    if (factor.similarity_constraint) {
      result.status = SolverStatus::kUnsupportedConstraint;
      result.diagnostic_message =
          "Pose3 optimization does not support similarity constraints";
      return result;
    }
  }
  if (state.isam != nullptr &&
      state.convergence_tolerance != options.convergence_tolerance) {
    result.diagnostic_message =
        "relinearization threshold cannot change during incremental updates";
    return result;
  }

  const char* stage = "graph assembly";
  try {
    if (state.isam == nullptr) {
      gtsam::ISAM2Params params;
      params.relinearizeThreshold = options.convergence_tolerance;
      params.relinearizeSkip = 1;
      params.factorization = gtsam::ISAM2Params::QR;
      state.isam = std::make_unique<gtsam::ISAM2>(params);
      state.convergence_tolerance = options.convergence_tolerance;
    }

    gtsam::NonlinearFactorGraph new_factors;
    gtsam::NonlinearFactorGraph new_measurements;
    gtsam::Values new_values;
    for (const auto& entry : additions.GetPoses()) {
      const gtsam::Key key = static_cast<gtsam::Key>(entry.first);
      const gtsam::Pose3 pose = ToPose3(entry.second.pose);
      new_values.insert(key, pose);
      if (entry.second.fixed) {
        new_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
            key, pose, MakeNoiseModel(1e9, 1e9)));
      }
    }
    for (const PriorFactor& factor : additions.GetPriorFactors()) {
      const auto prior = std::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          static_cast<gtsam::Key>(factor.pose_id), ToPose3(factor.pose),
          MakeNoiseModel(factor.rotation_weight, factor.translation_weight));
      new_factors.add(prior);
      new_measurements.add(prior);
    }
    for (const BetweenFactor& factor : additions.GetBetweenFactors()) {
      const auto between = std::make_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          static_cast<gtsam::Key>(factor.from_id),
          static_cast<gtsam::Key>(factor.to_id), ToPose3(factor.relative_pose),
          MakeNoiseModel(factor));
      new_factors.add(between);
      new_measurements.add(between);
    }

    stage = "ISAM2 update";
    if (!new_factors.empty() || !new_values.empty()) {
      state.isam->update(new_factors, new_values);
      ++result.update_count;
      while (result.update_count < options.maximum_update_count) {
        const gtsam::ISAM2Result refinement = state.isam->update();
        ++result.update_count;
        if (refinement.variablesRelinearized == 0 &&
            refinement.variablesReeliminated == 0) {
          break;
        }
      }
    }
    for (const auto& factor : new_measurements) {
      state.measurements.add(factor);
    }
    for (const auto& entry : additions.GetPoses()) {
      state.pose_ids.insert(entry.first);
    }

    result.status = SolverStatus::kSucceeded;
  } catch (const std::exception& error) {
    state.Reset();
    result.status = SolverStatus::kNumericalFailure;
    result.diagnostic_message = std::string(stage) + " failed: " + error.what();
  }
  return result;
}

SolverResult PoseGraphSolver::CalculateEstimate() {
  std::lock_guard<std::mutex> lock(state_->mutex);
  SolverResult result;
  State& state = *state_;
  if (state.isam == nullptr || state.pose_ids.empty()) {
    result.diagnostic_message = "pose graph has not been initialized";
    return result;
  }

  try {
    const gtsam::Values estimate = state.isam->calculateEstimate();
    result.optimized_poses.reserve(state.pose_ids.size());
    for (std::int64_t pose_id : state.pose_ids) {
      const gtsam::Pose3& pose =
          estimate.at<gtsam::Pose3>(static_cast<gtsam::Key>(pose_id));
      const transform::Rigid3d optimized = FromPose3(pose);
      if (!optimized.translation().allFinite() ||
          !optimized.rotation().coeffs().allFinite()) {
        throw std::runtime_error("optimizer returned a non-finite pose");
      }
      result.optimized_poses.emplace(pose_id, optimized);
    }
    result.final_cost = state.measurements.error(estimate);
    if (!std::isfinite(result.final_cost)) {
      throw std::runtime_error("objective is not finite");
    }
    result.status = SolverStatus::kSucceeded;
  } catch (const std::exception& error) {
    state.Reset();
    result.optimized_poses.clear();
    result.status = SolverStatus::kNumericalFailure;
    result.diagnostic_message =
        std::string("estimate extraction failed: ") + error.what();
  }
  return result;
}

}  // namespace pose_graph
}  // namespace backend
}  // namespace atlas
