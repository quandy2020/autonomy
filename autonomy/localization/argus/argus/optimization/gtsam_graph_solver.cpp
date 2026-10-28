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

#include "optimization/gtsam_graph_solver.hpp"

#include <cmath>
#include <exception>
#include <unordered_set>

#if defined(__has_include)
#if __has_include(<gtsam/nonlinear/ISAM2.h>)
#define ARGUS_HAS_GTSAM 1
#endif
#endif

#ifdef ARGUS_HAS_GTSAM
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#endif

namespace argus {
namespace optimization {
namespace {

#ifdef ARGUS_HAS_GTSAM
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

gtsam::SharedNoiseModel Noise(double rotation_weight,
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

class GtsamGraphSolver : public GraphSolver {
 public:
  const char* name() const override { return "gtsam"; }

  bool Optimize(FactorGraph* graph) override {
    if (graph == nullptr) {
      return false;
    }
    for (const BetweenFactor& between : graph->betweens()) {
      if (between.similarity) {
        return false;
      }
    }
    gtsam::NonlinearFactorGraph new_factors;
    gtsam::Values new_values;
    for (const auto& entry : graph->poses()) {
      if (!inserted_.insert(entry.first).second) {
        continue;
      }
      const gtsam::Key key = static_cast<gtsam::Key>(entry.first);
      const gtsam::Pose3 pose = ToPose3(entry.second.pose);
      new_values.insert(key, pose);
      if (entry.second.fixed) {
        new_factors.add(
            gtsam::PriorFactor<gtsam::Pose3>(key, pose, Noise(1e6, 1e6)));
      }
    }
    const std::vector<PriorFactor>& priors = graph->priors();
    if (prior_cursor_ > priors.size() ||
        between_cursor_ > graph->betweens().size()) {
      return false;
    }
    for (; prior_cursor_ < priors.size(); ++prior_cursor_) {
      const PriorFactor& prior = priors[prior_cursor_];
      if (!graph->Contains(prior.id)) {
        return false;
      }
      new_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
          static_cast<gtsam::Key>(prior.id), ToPose3(prior.pose),
          Noise(prior.rotation_weight, prior.translation_weight)));
    }
    const std::vector<BetweenFactor>& betweens = graph->betweens();
    for (; between_cursor_ < betweens.size(); ++between_cursor_) {
      const BetweenFactor& between = betweens[between_cursor_];
      if (!graph->Contains(between.from) || !graph->Contains(between.to)) {
        return false;
      }
      new_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
          static_cast<gtsam::Key>(between.from),
          static_cast<gtsam::Key>(between.to), ToPose3(between.relative),
          Noise(between.rotation_weight, between.translation_weight)));
    }
    try {
      if (!new_factors.empty() || !new_values.empty()) {
        isam_.update(new_factors, new_values);
      }
    } catch (const std::exception&) {
      return false;
    }
    const gtsam::Values result = isam_.calculateEstimate();
    for (const auto& entry : graph->poses()) {
      if (entry.second.fixed) {
        continue;
      }
      const gtsam::Pose3 pose =
          result.at<gtsam::Pose3>(static_cast<gtsam::Key>(entry.first));
      if (!graph->SetPose(entry.first, FromPose3(pose))) {
        return false;
      }
    }
    return true;
  }

 private:
  gtsam::ISAM2 isam_;
  std::unordered_set<PoseId> inserted_;
  std::size_t prior_cursor_ = 0;
  std::size_t between_cursor_ = 0;
};
#endif

}  // namespace

bool GtsamSolverAvailable() {
#ifdef ARGUS_HAS_GTSAM
  return true;
#else
  return false;
#endif
}

std::unique_ptr<GraphSolver> CreateGtsamSolver() {
#ifdef ARGUS_HAS_GTSAM
  return std::make_unique<GtsamGraphSolver>();
#else
  return nullptr;
#endif
}

}  // namespace optimization
}  // namespace argus
