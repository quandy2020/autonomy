/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/kinematics/kdl_kinematics.hpp"

#include <chrono>
#include <cmath>
#include <random>
#include <unordered_map>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>

#include <Eigen/Core>

#include "autonomy/common/logging.hpp"
#include "autolink/plugin_manager/plugin_manager.hpp"

namespace autonomy {
namespace manipulation {
namespace kinematics {
namespace {

automsgs::msgs::geometry_msgs::Pose FrameToPose(const KDL::Frame& frame) {
  automsgs::msgs::geometry_msgs::Pose pose;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
  frame.M.GetQuaternion(x, y, z, w);
  SetPose(&pose, frame.p.x(), frame.p.y(), frame.p.z(), x, y, z, w);
  return pose;
}

KDL::Frame PoseToFrame(const automsgs::msgs::geometry_msgs::Pose& pose) {
  return KDL::Frame(
      KDL::Rotation::Quaternion(pose.orientation().x(), pose.orientation().y(),
                                pose.orientation().z(), pose.orientation().w()),
      KDL::Vector(pose.position().x(), pose.position().y(),
                  pose.position().z()));
}

}  // namespace

bool KdlKinematics::Init(const std::string& group, const std::string& base_frame,
                         const std::string& tip_frame) {
  group_ = group;
  base_frame_ = base_frame.empty() ? "base_link" : base_frame;
  tip_frame_ = tip_frame.empty() ? "tool0" : tip_frame;
  ready_ = false;
  return true;
}

bool KdlKinematics::LoadUrdf(const std::string& urdf_path) {
  ready_ = false;
  if (urdf_path.empty()) {
    AERROR << "KdlKinematics: empty URDF path";
    return false;
  }
  std::string error;
  if (!model::BuildKdlChainFromUrdfFile(urdf_path, base_frame_, tip_frame_, &model_,
                                   &error)) {
    AERROR << "KdlKinematics: " << error;
    return false;
  }
  ready_ = true;
  AINFO << "KdlKinematics ready group=" << group_
        << " joints=" << model_.joint_names.size() << " " << base_frame_ << "→"
        << tip_frame_;
  return true;
}

bool KdlKinematics::MapJoints(const automsgs::msgs::sensor_msgs::JointState& joints,
                              KDL::JntArray* q) const {
  if (!q || !ready_) {
    return false;
  }
  const unsigned int n = model_.chain.getNrOfJoints();
  q->resize(n);
  const int n_i = static_cast<int>(n);
  if (joints.position_size() == n_i &&
      (joints.name_size() == 0 || joints.name_size() == n_i)) {
    if (joints.name_size() > 0) {
      std::unordered_map<std::string, double> by_name;
      for (int i = 0; i < joints.name_size(); ++i) {
        by_name[joints.name(i)] = joints.position(i);
      }
      for (unsigned int i = 0; i < n; ++i) {
        const auto it = by_name.find(model_.joint_names[i]);
        if (it == by_name.end()) {
          return false;
        }
        (*q)(i) = it->second;
      }
      return true;
    }
    for (unsigned int i = 0; i < n; ++i) {
      (*q)(i) = joints.position(static_cast<int>(i));
    }
    return true;
  }

  if (joints.name_size() != joints.position_size()) {
    return false;
  }
  std::unordered_map<std::string, double> by_name;
  for (int i = 0; i < joints.name_size(); ++i) {
    by_name[joints.name(i)] = joints.position(i);
  }
  for (unsigned int i = 0; i < n; ++i) {
    const auto it = by_name.find(model_.joint_names[i]);
    (*q)(i) = (it == by_name.end()) ? 0.0 : it->second;
  }
  return true;
}

bool KdlKinematics::GetPositionFK(const automsgs::msgs::sensor_msgs::JointState& joints,
                                  automsgs::msgs::geometry_msgs::Pose* tip_pose) const {
  if (!tip_pose || !ready_) {
    return false;
  }
  KDL::JntArray q;
  if (!MapJoints(joints, &q)) {
    return false;
  }
  KDL::ChainFkSolverPos_recursive fk(model_.chain);
  KDL::Frame tip;
  if (fk.JntToCart(q, tip) < 0) {
    return false;
  }
  *tip_pose = FrameToPose(tip);
  return true;
}

bool KdlKinematics::SolveOnce(const automsgs::msgs::geometry_msgs::Pose& tip_pose, const KDL::JntArray& q_seed,
                              bool position_only, KDL::JntArray* q_out) const {
  KDL::Frame target = PoseToFrame(tip_pose);
  if (position_only) {
    Eigen::Matrix<double, 6, 1> weights;
    weights << 1.0, 1.0, 1.0, 1e-5, 1e-5, 1e-5;
    KDL::ChainIkSolverPos_LMA ik(model_.chain, weights, /*eps=*/1e-4,
                                 /*maxiter=*/200);
    return ik.CartToJnt(q_seed, target, *q_out) >= 0;
  }

  KDL::ChainFkSolverPos_recursive fk(model_.chain);
  KDL::ChainIkSolverVel_pinv vik(model_.chain);
  KDL::ChainIkSolverPos_NR_JL ik(model_.chain, model_.position_lower_bounds, model_.position_upper_bounds, fk,
                                 vik, /*maxiter=*/200, /*eps=*/1e-5);
  return ik.CartToJnt(q_seed, target, *q_out) >= 0;
}

ErrorCode KdlKinematics::GetPositionIK(const automsgs::msgs::geometry_msgs::Pose& tip_pose,
                                       const automsgs::msgs::sensor_msgs::JointState& seed,
                                       const InverseKinematicsOptions& options,
                                       automsgs::msgs::sensor_msgs::JointState* solution) const {
  if (!solution || !ready_) {
    return ErrorCode::NO_INVERSE_KINEMATICS_SOLUTION;
  }

  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(options.timeout()));

  KDL::JntArray q_seed;
  if (!MapJoints(seed, &q_seed)) {
    q_seed = KDL::JntArray(model_.chain.getNrOfJoints());
  }

  KDL::JntArray q_out(model_.chain.getNrOfJoints());
  std::mt19937 rng{std::random_device{}()};

  const int attempts = std::max(1, options.max_attempts());
  for (int attempt = 0; attempt < attempts; ++attempt) {
    if (std::chrono::steady_clock::now() > deadline) {
      return ErrorCode::TIMED_OUT;
    }
    KDL::JntArray seed_try = q_seed;
    if (attempt > 0) {
      for (unsigned int i = 0; i < seed_try.rows(); ++i) {
        std::uniform_real_distribution<double> dist(model_.position_lower_bounds(i),
                                                    model_.position_upper_bounds(i));
        seed_try(i) = dist(rng);
        if (!options.consistency_limits.empty() &&
            i < options.consistency_limits.size()) {
          const double lim = options.consistency_limits[i];
          seed_try(i) =
              std::clamp(seed_try(i), q_seed(i) - lim, q_seed(i) + lim);
        }
      }
    }
    if (SolveOnce(tip_pose, seed_try, options.position_only(), &q_out)) {
      std::vector<double> positions(model_.joint_names.size());
      for (std::size_t i = 0; i < model_.joint_names.size(); ++i) {
        positions[i] = q_out(static_cast<unsigned int>(i));
      }
      SetJointState(solution, model_.joint_names, positions);
      return ErrorCode::SUCCESS;
    }
  }
  AWARN << "KdlKinematics: IK failed after " << attempts << " attempts";
  return ErrorCode::NO_INVERSE_KINEMATICS_SOLUTION;
}


AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(KdlKinematics, KinematicsInterface);

}  // namespace kinematics
}  // namespace manipulation
}  // namespace autonomy
