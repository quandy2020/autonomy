/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/servo/servo.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace manipulation {
namespace servo {
namespace {

kinematics::Pose PerturbFk(kinematics::KinematicsBase* kin,
                           const core::JointState& q, std::size_t j,
                           double eps) {
  core::JointState qp = q;
  qp.positions[j] += eps;
  kinematics::Pose pose;
  kin->GetPositionFK(qp, &pose);
  return pose;
}

}  // namespace

void DampedLeastSquaresServo::SetKinematics(
    std::shared_ptr<kinematics::KinematicsBase> kin) {
  std::lock_guard<std::mutex> lock(mutex_);
  kinematics_ = std::move(kin);
}

void DampedLeastSquaresServo::SetScene(
    std::shared_ptr<scene::PlanningScene> scene) {
  std::lock_guard<std::mutex> lock(mutex_);
  scene_ = std::move(scene);
}

void DampedLeastSquaresServo::SetModel(
    std::shared_ptr<const core::SimpleRobotModel> model) {
  std::lock_guard<std::mutex> lock(mutex_);
  model_ = std::move(model);
}

void DampedLeastSquaresServo::SetState(const core::JointState& state) {
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = state;
}

bool DampedLeastSquaresServo::Init() {
  std::lock_guard<std::mutex> lock(mutex_);
  halted_ = false;
  status_ = ServoStatus::kOk;
  return kinematics_ != nullptr && !state_.positions.empty();
}

bool DampedLeastSquaresServo::NumericJacobian(
    const core::JointState& q, std::vector<std::vector<double>>* J) const {
  if (!kinematics_ || !J || q.positions.empty()) {
    return false;
  }
  kinematics::Pose p0;
  if (!kinematics_->GetPositionFK(q, &p0)) {
    return false;
  }
  const double eps = 1e-4;
  const std::size_t n = q.positions.size();
  J->assign(6, std::vector<double>(n, 0.0));
  for (std::size_t j = 0; j < n; ++j) {
    const auto p1 = PerturbFk(kinematics_.get(), q, j, eps);
    (*J)[0][j] = (p1.x - p0.x) / eps;
    (*J)[1][j] = (p1.y - p0.y) / eps;
    (*J)[2][j] = (p1.z - p0.z) / eps;
    (*J)[3][j] = (p1.qx - p0.qx) / eps;
    (*J)[4][j] = (p1.qy - p0.qy) / eps;
    (*J)[5][j] = (p1.qz - p0.qz) / eps;
  }
  return true;
}

double DampedLeastSquaresServo::Manipulability(
    const std::vector<std::vector<double>>& J) const {
  // approx √det(J J^T) via product of row norms (cheap singularity proxy)
  double prod = 1.0;
  for (int i = 0; i < 6; ++i) {
    double n2 = 0.0;
    for (double v : J[static_cast<std::size_t>(i)]) {
      n2 += v * v;
    }
    prod *= std::sqrt(std::max(n2, 0.0));
  }
  return prod;
}

TwistCommand DampedLeastSquaresServo::FilterTwist(const TwistCommand& twist) {
  const double a = std::clamp(twist_alpha_, 0.0, 1.0);
  filtered_twist_.vx = a * twist.vx + (1.0 - a) * filtered_twist_.vx;
  filtered_twist_.vy = a * twist.vy + (1.0 - a) * filtered_twist_.vy;
  filtered_twist_.vz = a * twist.vz + (1.0 - a) * filtered_twist_.vz;
  filtered_twist_.wx = a * twist.wx + (1.0 - a) * filtered_twist_.wx;
  filtered_twist_.wy = a * twist.wy + (1.0 - a) * filtered_twist_.wy;
  filtered_twist_.wz = a * twist.wz + (1.0 - a) * filtered_twist_.wz;
  return filtered_twist_;
}

bool DampedLeastSquaresServo::Update(const TwistCommand& twist,
                                     core::JointState* joint_command) {
  if (!joint_command) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!kinematics_ || state_.positions.empty() || halted_) {
    status_ = halted_ ? ServoStatus::kHalted : ServoStatus::kOk;
    *joint_command = state_;
    return !halted_;
  }

  const TwistCommand cmd = FilterTwist(twist);
  std::vector<std::vector<double>> J;
  if (!NumericJacobian(state_, &J)) {
    status_ = ServoStatus::kHalted;
    return false;
  }

  const double manip = Manipulability(J);
  if (manip < singularity_threshold_) {
    status_ = ServoStatus::kSingularity;
    *joint_command = state_;
    AWARN << "Servo singularity halt manip=" << manip;
    return false;
  }

  const std::size_t n = state_.positions.size();
  std::vector<double> twist_v = {cmd.vx, cmd.vy, cmd.vz,
                                 cmd.wx, cmd.wy, cmd.wz};
  std::vector<double> qdot(n, 0.0);
  for (std::size_t j = 0; j < n; ++j) {
    double num = 0.0;
    double den = lambda_ * lambda_;
    for (int i = 0; i < 6; ++i) {
      num += J[static_cast<std::size_t>(i)][j] * twist_v[static_cast<std::size_t>(i)];
      den += J[static_cast<std::size_t>(i)][j] * J[static_cast<std::size_t>(i)][j];
    }
    qdot[j] = num / den;
    qdot[j] = std::clamp(qdot[j], -max_qdot_, max_qdot_);
  }

  core::JointState next = state_;
  for (std::size_t j = 0; j < n; ++j) {
    next.positions[j] += qdot[j] * dt_;
  }

  if (model_) {
    for (std::size_t j = 0; j < n && j < next.names.size(); ++j) {
      const auto* lim = model_->GetJointLimits(next.names[j]);
      if (!lim || !lim->has_position_limits) {
        continue;
      }
      if (next.positions[j] < lim->min_position ||
          next.positions[j] > lim->max_position) {
        status_ = ServoStatus::kJointLimit;
        *joint_command = state_;
        return false;
      }
    }
  }

  if (scene_ && scene_->CheckCollision(next)) {
    status_ = ServoStatus::kCollision;
    *joint_command = state_;
    return false;
  }

  state_ = next;
  status_ = ServoStatus::kOk;
  *joint_command = state_;
  return true;
}

void DampedLeastSquaresServo::Stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  halted_ = true;
  status_ = ServoStatus::kHalted;
  filtered_twist_ = {};
}

void ServoNode::SetServo(std::shared_ptr<Servo> servo) {
  std::lock_guard<std::mutex> lock(mutex_);
  servo_ = std::move(servo);
}

void ServoNode::SetCommand(const TwistCommand& twist) {
  std::lock_guard<std::mutex> lock(mutex_);
  twist_ = twist;
}

void ServoNode::Start() {
  if (running_.exchange(true)) {
    return;
  }
  thread_ = std::thread([this] { Loop(); });
}

void ServoNode::Stop() {
  running_.store(false);
  if (thread_.joinable()) {
    thread_.join();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (servo_) {
    servo_->Stop();
  }
}

void ServoNode::Loop() {
  while (running_.load()) {
    TwistCommand cmd;
    std::shared_ptr<Servo> servo;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cmd = twist_;
      servo = servo_;
    }
    if (servo) {
      core::JointState out;
      servo->Update(cmd, &out);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

}  // namespace servo
}  // namespace manipulation
}  // namespace autonomy
