/*
 * Copyright 2026 The Openbot Authors
 *
 * DynamicsSolver factory: Pinocchio when FEATURE, else zero-torque stub.
 */

#include "autonomy/manipulation/motion/dynamics/dynamics_solver_factory.hpp"

#include <algorithm>

#include "autonomy/common/logging.hpp"

#if defined(AUTONOMY_HAS_PINOCCHIO)
#include <Eigen/Core>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#endif

namespace autonomy {
namespace manipulation {
namespace dynamics {
namespace {

#if defined(AUTONOMY_HAS_PINOCCHIO)
class PinocchioDynamicsSolver : public DynamicsSolver {
 public:
  explicit PinocchioDynamicsSolver(std::string urdf_path)
      : urdf_path_(std::move(urdf_path)) {}

  bool Init(const std::string& /*group*/) override {
    try {
      pinocchio::urdf::buildModel(urdf_path_, model_);
      data_ = pinocchio::Data(model_);
      ready_ = model_.nq > 0;
      if (ready_) {
        AINFO << "PinocchioDynamicsSolver nq=" << model_.nq << " from "
              << urdf_path_;
      }
      return ready_;
    } catch (const std::exception& e) {
      AERROR << "Pinocchio buildModel failed: " << e.what();
      ready_ = false;
      return false;
    }
  }

  bool GetGravityTorques(const std::vector<double>& positions,
                         std::vector<double>* torques) const override {
    if (!torques || !ready_) {
      return false;
    }
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);
    const int n = std::min(model_.nq, static_cast<int>(positions.size()));
    for (int i = 0; i < n; ++i) {
      q[i] = positions[static_cast<std::size_t>(i)];
    }
    pinocchio::Data data = data_;
    const Eigen::VectorXd g =
        pinocchio::computeGeneralizedGravity(model_, data, q);
    torques->resize(static_cast<std::size_t>(g.size()));
    for (int i = 0; i < g.size(); ++i) {
      (*torques)[static_cast<std::size_t>(i)] = g[i];
    }
    return true;
  }

  bool GetCoriolisTorques(const std::vector<double>& positions,
                          const std::vector<double>& velocities,
                          std::vector<double>* torques) const override {
    if (!torques || !ready_) {
      return false;
    }
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
    const int nq = std::min(model_.nq, static_cast<int>(positions.size()));
    const int nv = std::min(model_.nv, static_cast<int>(velocities.size()));
    for (int i = 0; i < nq; ++i) {
      q[i] = positions[static_cast<std::size_t>(i)];
    }
    for (int i = 0; i < nv; ++i) {
      v[i] = velocities[static_cast<std::size_t>(i)];
    }
    pinocchio::Data data = data_;
    const Eigen::VectorXd nle =
        pinocchio::nonLinearEffects(model_, data, q, v);
    torques->resize(static_cast<std::size_t>(nle.size()));
    for (int i = 0; i < nle.size(); ++i) {
      (*torques)[static_cast<std::size_t>(i)] = nle[i];
    }
    return true;
  }

 private:
  std::string urdf_path_;
  pinocchio::Model model_;
  mutable pinocchio::Data data_;
  bool ready_ = false;
};
#endif

}  // namespace

bool HasPinocchioDynamics() {
#if defined(AUTONOMY_HAS_PINOCCHIO)
  return true;
#else
  return false;
#endif
}

std::shared_ptr<DynamicsSolver> CreateDynamicsSolver(
    const std::string& urdf_path) {
#if defined(AUTONOMY_HAS_PINOCCHIO)
  if (!urdf_path.empty()) {
    auto solver = std::make_shared<PinocchioDynamicsSolver>(urdf_path);
    if (solver->Init("")) {
      return solver;
    }
  }
#else
  (void)urdf_path;
#endif
  return std::make_shared<DynamicsSolver>();
}

}  // namespace dynamics
}  // namespace manipulation
}  // namespace autonomy
