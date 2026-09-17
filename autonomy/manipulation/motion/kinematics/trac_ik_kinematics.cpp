/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/kinematics/trac_ik_kinematics.hpp"

#include <cmath>
#include <fstream>
#include <random>
#include <sstream>
#include <vector>

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/common/logging.hpp"

#if defined(AUTONOMY_HAS_KDL)
#include "autonomy/manipulation/motion/kinematics/kdl_kinematics.hpp"
#endif

#if defined(AUTONOMY_HAS_TRAC_IK)
#if __has_include(<trac_ik/trac_ik.hpp>)
#include <trac_ik/trac_ik.hpp>
#elif __has_include(<trac_ik.hpp>)
#include <trac_ik.hpp>
#else
#error "AUTONOMY_HAS_TRAC_IK set but trac_ik.hpp not found"
#endif
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#endif

namespace autonomy {
namespace manipulation {
namespace kinematics {

struct TracIkKinematics::TracIkBackend {
#if defined(AUTONOMY_HAS_TRAC_IK)
  std::unique_ptr<TRAC_IK::TRAC_IK> solver;
  unsigned nq = 0;
#endif
};

bool TracIkKinematics::HasTracIkLibrary() {
#if defined(AUTONOMY_HAS_TRAC_IK)
  return true;
#else
  return false;
#endif
}

bool TracIkKinematics::Init(const std::string& group,
                            const std::string& base_frame,
                            const std::string& tip_frame) {
  group_ = group;
  base_frame_ = base_frame;
  tip_frame_ = tip_frame;
  backend_ = std::make_shared<TracIkBackend>();
#if defined(AUTONOMY_HAS_KDL)
  auto kdl = std::make_shared<KdlKinematics>();
  if (!kdl->Init(group, base_frame, tip_frame)) {
    return false;
  }
  inner_ = std::move(kdl);
#endif
  if (HasTracIkLibrary()) {
    AINFO << "TracIkKinematics: TRAC_IK library FEATURE enabled";
  } else {
    AINFO << "TracIkKinematics: multi-seed KDL path (TRAC-IK lite)";
  }
  return true;
}

bool TracIkKinematics::LoadUrdf(const std::string& urdf_path) {
  urdf_path_ = urdf_path;
#if defined(AUTONOMY_HAS_KDL)
  auto* kdl = dynamic_cast<KdlKinematics*>(inner_.get());
  if (kdl && !kdl->LoadUrdf(urdf_path)) {
    return false;
  }
#endif
#if defined(AUTONOMY_HAS_TRAC_IK)
  if (!backend_) {
    backend_ = std::make_shared<TracIkBackend>();
  }
  std::ifstream in(urdf_path);
  if (!in) {
    AERROR << "TracIkKinematics: cannot read URDF " << urdf_path;
    return false;
  }
  std::ostringstream buf;
  buf << in.rdbuf();
  const std::string urdf_xml = buf.str();
  try {
    // URDF XML constructor (trac_ik ≥ industrial forks); timeout/eps MoveIt-like.
    backend_->solver = std::make_unique<TRAC_IK::TRAC_IK>(
        base_frame_, tip_frame_, urdf_xml, 0.05, 1e-5, TRAC_IK::Speed);
    KDL::Chain chain;
    if (backend_->solver->getKDLChain(chain)) {
      backend_->nq = static_cast<unsigned>(chain.getNrOfJoints());
    }
    AINFO << "TracIkKinematics: TRAC_IK solver ready nq=" << backend_->nq;
  } catch (const std::exception& e) {
    AWARN << "TracIkKinematics: TRAC_IK construct failed (" << e.what()
          << "); falling back to KDL multi-seed";
    backend_->solver.reset();
  }
#endif
  return true;
}

bool TracIkKinematics::GetPositionFK(const automsgs::msgs::sensor_msgs::JointState& joints,
                                     automsgs::msgs::geometry_msgs::Pose* tip_pose) const {
  return inner_ && inner_->GetPositionFK(joints, tip_pose);
}

ErrorCode TracIkKinematics::GetPositionIK(const automsgs::msgs::geometry_msgs::Pose& tip_pose,
                                          const automsgs::msgs::sensor_msgs::JointState& seed,
                                          const InverseKinematicsOptions& options,
                                          automsgs::msgs::sensor_msgs::JointState* solution) const {
  if (!solution) {
    return ErrorCode::FAILURE;
  }
#if defined(AUTONOMY_HAS_TRAC_IK)
  if (backend_ && backend_->solver && backend_->nq > 0) {
    KDL::JntArray q_init(backend_->nq);
    for (unsigned i = 0; i < backend_->nq; ++i) {
      q_init(i) = i < static_cast<unsigned>(seed.position_size())
                      ? seed.position(static_cast<int>(i))
                      : 0.0;
    }
    KDL::Frame tip = KDL::Frame(
        KDL::Rotation::Quaternion(
            tip_pose.orientation().x(), tip_pose.orientation().y(),
            tip_pose.orientation().z(), tip_pose.orientation().w()),
        KDL::Vector(tip_pose.position().x(), tip_pose.position().y(),
                    tip_pose.position().z()));
    KDL::JntArray q_out(backend_->nq);
    const double timeout =
        options.timeout() > 0.0 ? options.timeout() : 0.05;
    (void)timeout;
    const int rc = backend_->solver->CartToJnt(q_init, tip, q_out);
    if (rc >= 0) {
      std::vector<std::string> names;
      names.reserve(static_cast<std::size_t>(seed.name_size()));
      for (int i = 0; i < seed.name_size(); ++i) {
        names.push_back(seed.name(i));
      }
      if (names.empty() && inner_) {
#if defined(AUTONOMY_HAS_KDL)
        if (auto* kdl = dynamic_cast<KdlKinematics*>(inner_.get())) {
          names = kdl->JointNames();
        }
#endif
      }
      std::vector<double> positions(backend_->nq);
      for (unsigned i = 0; i < backend_->nq; ++i) {
        positions[i] = q_out(i);
      }
      SetJointState(solution, names, positions);
      return ErrorCode::SUCCESS;
    }
  }
#endif
  if (!inner_) {
    return ErrorCode::FAILURE;
  }
  if (inner_->GetPositionIK(tip_pose, seed, options, solution) ==
      ErrorCode::SUCCESS) {
    return ErrorCode::SUCCESS;
  }
  std::mt19937 rng(42);
  std::normal_distribution<double> noise(0.0, 0.35);
  const int extra = std::max(2, options.max_attempts());
  for (int i = 0; i < extra; ++i) {
    automsgs::msgs::sensor_msgs::JointState alt = seed;
    for (int j = 0; j < alt.position_size(); ++j) {
      alt.set_position(j, alt.position(j) + noise(rng));
    }
    InverseKinematicsOptions opt = options;
    opt.max_attempts = std::max(1, options.max_attempts() / 2);
    if (inner_->GetPositionIK(tip_pose, alt, opt, solution) ==
        ErrorCode::SUCCESS) {
      return ErrorCode::SUCCESS;
    }
  }
  return ErrorCode::NO_INVERSE_KINEMATICS_SOLUTION;
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(TracIkKinematics, KinematicsInterface);

}  // namespace kinematics
}  // namespace manipulation
}  // namespace autonomy
