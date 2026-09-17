/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/servo/servo.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_map>

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace manipulation {
namespace servo {
namespace {

Pose PerturbFk(kinematics::KinematicsBase* kin, const JointState& q,
               int j, double eps) {
  JointState qp = q;
  qp.set_position(j, qp.position(j) + eps);
  Pose pose;
  kin->GetPositionFK(qp, &pose);
  return pose;
}

Twist PoseErrorTwist(const Pose& cur, const Pose& goal, double kp_linear,
                     double kp_angular) {
  Twist t;
  t.mutable_linear()->set_x(kp_linear *
                            (goal.position().x() - cur.position().x()));
  t.mutable_linear()->set_y(kp_linear *
                            (goal.position().y() - cur.position().y()));
  t.mutable_linear()->set_z(kp_linear *
                            (goal.position().z() - cur.position().z()));

  // q_err ≈ q_goal ⊗ q_cur⁻¹; use vector part × 2 as angle-axis proxy.
  const double cw = cur.orientation().w();
  const double cx = -cur.orientation().x();
  const double cy = -cur.orientation().y();
  const double cz = -cur.orientation().z();
  const double gw = goal.orientation().w();
  const double gx = goal.orientation().x();
  const double gy = goal.orientation().y();
  const double gz = goal.orientation().z();
  const double ew = gw * cw - gx * cx - gy * cy - gz * cz;
  double ex = gw * cx + gx * cw + gy * cz - gz * cy;
  double ey = gw * cy - gx * cz + gy * cw + gz * cx;
  double ez = gw * cz + gx * cy - gy * cx + gz * cw;
  if (ew < 0.0) {
    ex = -ex;
    ey = -ey;
    ez = -ez;
  }
  t.mutable_angular()->set_x(kp_angular * 2.0 * ex);
  t.mutable_angular()->set_y(kp_angular * 2.0 * ey);
  t.mutable_angular()->set_z(kp_angular * 2.0 * ez);
  return t;
}

void AssignZeroVelocities(JointState* state, int n) {
  if (!state) {
    return;
  }
  state->clear_velocity();
  for (int i = 0; i < n; ++i) {
    state->add_velocity(0.0);
  }
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

void DampedLeastSquaresServo::SetState(const JointState& state) {
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = state;
}

bool DampedLeastSquaresServo::Init() {
  std::lock_guard<std::mutex> lock(mutex_);
  halted_ = false;
  status_ = ServoStatus::kOk;
  return kinematics_ != nullptr && GetJointStateDof(state_) > 0;
}

bool DampedLeastSquaresServo::NumericJacobian(
    const JointState& q, std::vector<std::vector<double>>* J) const {
  if (!kinematics_ || !J || GetJointStateDof(q) <= 0) {
    return false;
  }
  Pose p0;
  if (!kinematics_->GetPositionFK(q, &p0)) {
    return false;
  }
  const double eps = 1e-4;
  const int n = GetJointStateDof(q);
  J->assign(6, std::vector<double>(static_cast<std::size_t>(n), 0.0));
  for (int j = 0; j < n; ++j) {
    const auto p1 = PerturbFk(kinematics_.get(), q, j, eps);
    (*J)[0][static_cast<std::size_t>(j)] =
        (p1.position().x() - p0.position().x()) / eps;
    (*J)[1][static_cast<std::size_t>(j)] =
        (p1.position().y() - p0.position().y()) / eps;
    (*J)[2][static_cast<std::size_t>(j)] =
        (p1.position().z() - p0.position().z()) / eps;
    (*J)[3][static_cast<std::size_t>(j)] =
        (p1.orientation().x() - p0.orientation().x()) / eps;
    (*J)[4][static_cast<std::size_t>(j)] =
        (p1.orientation().y() - p0.orientation().y()) / eps;
    (*J)[5][static_cast<std::size_t>(j)] =
        (p1.orientation().z() - p0.orientation().z()) / eps;
  }
  return true;
}

double DampedLeastSquaresServo::ComputeManipulability(
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

Twist DampedLeastSquaresServo::FilterTwist(const Twist& twist) {
  const double a = std::clamp(twist_alpha_, 0.0, 1.0);
  auto* fl = filtered_twist_.mutable_linear();
  auto* fa = filtered_twist_.mutable_angular();
  const auto& l = twist.linear();
  const auto& ang = twist.angular();
  fl->set_x(a * l.x() + (1.0 - a) * fl->x());
  fl->set_y(a * l.y() + (1.0 - a) * fl->y());
  fl->set_z(a * l.z() + (1.0 - a) * fl->z());
  fa->set_x(a * ang.x() + (1.0 - a) * fa->x());
  fa->set_y(a * ang.y() + (1.0 - a) * fa->y());
  fa->set_z(a * ang.z() + (1.0 - a) * fa->z());
  return filtered_twist_;
}

double DampedLeastSquaresServo::CollisionApproachScale(
    const JointState& q) const {
  if (!scene_) {
    return 1.0;
  }
  const auto dinfo = scene_->DistanceRobotWorld(q);
  if (dinfo.collision) {
    return 0.0;
  }
  const double halt = std::max(0.0, collision_halt_threshold_);
  const double prox =
      std::max(halt + 1e-6, collision_proximity_threshold_);
  if (dinfo.distance <= halt) {
    return 0.0;
  }
  if (dinfo.distance >= prox) {
    return 1.0;
  }
  // Linear ramp halt → proximity (MoveIt decelerate-for-collision lite).
  return std::clamp((dinfo.distance - halt) / (prox - halt), 0.05, 1.0);
}

bool DampedLeastSquaresServo::CommitState(const JointState& next,
                                          JointState* joint_command) {
  if (model_) {
    const int n = std::min(next.position_size(), next.name_size());
    for (int j = 0; j < n; ++j) {
      const auto* lim = model_->GetJointLimits(next.name(j));
      if (!lim || !lim->has_position_limits) {
        continue;
      }
      if (next.position(j) < lim->min_position ||
          next.position(j) > lim->max_position) {
        status_ = ServoStatus::kJointLimit;
        *joint_command = state_;
        return false;
      }
    }
  }

  JointState candidate = next;
  bool decelerated = false;
  if (scene_) {
    if (scene_->CheckCollision(candidate)) {
      // Binary-search largest step fraction that stays collision-free.
      double lo = 0.0;
      double hi = 1.0;
      JointState best = state_;
      const int n = GetJointStateDof(candidate);
      for (int it = 0; it < 8; ++it) {
        const double mid = 0.5 * (lo + hi);
        JointState trial = state_;
        if (GetJointStateDof(trial) != n) {
          break;
        }
        AssignZeroVelocities(&trial, n);
        for (int j = 0; j < n; ++j) {
          trial.set_position(
              j, state_.position(j) +
                     mid * (candidate.position(j) - state_.position(j)));
          if (j < candidate.velocity_size()) {
            trial.set_velocity(j, mid * candidate.velocity(j));
          }
        }
        if (scene_->CheckCollision(trial)) {
          hi = mid;
        } else {
          lo = mid;
          best = trial;
        }
      }
      if (lo < 1e-3) {
        status_ = ServoStatus::kCollision;
        *joint_command = state_;
        return false;
      }
      candidate = best;
      decelerated = true;
    }
  }

  state_ = candidate;
  status_ = decelerated ? ServoStatus::kDecelerateForCollision
                        : ServoStatus::kOk;
  *joint_command = state_;
  return true;
}

bool DampedLeastSquaresServo::Update(const Twist& twist,
                                     JointState* joint_command) {
  if (!joint_command) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!kinematics_ || GetJointStateDof(state_) <= 0 || halted_) {
    status_ = halted_ ? ServoStatus::kHalted : ServoStatus::kOk;
    *joint_command = state_;
    return !halted_;
  }

  const Twist cmd = FilterTwist(twist);
  std::vector<std::vector<double>> J;
  if (!NumericJacobian(state_, &J)) {
    status_ = ServoStatus::kHalted;
    return false;
  }

  const double manip = ComputeManipulability(J);
  double scale = 1.0;
  if (manip < singularity_threshold_) {
    status_ = ServoStatus::kSingularity;
    *joint_command = state_;
    AWARN << "Servo singularity halt manip=" << manip;
    return false;
  }
  // Soft decelerate approaching singularity (MoveIt decelerate-for-singularity).
  const double soft = singularity_threshold_ * 10.0;
  if (manip < soft) {
    scale = std::clamp(manip / soft, 0.05, 1.0);
  }
  // Soft decelerate approaching collision (clearance band).
  const double cscale = CollisionApproachScale(state_);
  if (cscale <= 0.0) {
    status_ = ServoStatus::kCollision;
    *joint_command = state_;
    return false;
  }
  scale *= cscale;

  const int n = GetJointStateDof(state_);
  std::vector<double> twist_v = {
      cmd.linear().x() * scale,  cmd.linear().y() * scale,
      cmd.linear().z() * scale,  cmd.angular().x() * scale,
      cmd.angular().y() * scale, cmd.angular().z() * scale};
  std::vector<double> qdot(static_cast<std::size_t>(n), 0.0);
  for (int j = 0; j < n; ++j) {
    double num = 0.0;
    double den = lambda_ * lambda_;
    for (int i = 0; i < 6; ++i) {
      num += J[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] *
             twist_v[static_cast<std::size_t>(i)];
      den += J[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] *
             J[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
    }
    qdot[static_cast<std::size_t>(j)] = num / den;
    qdot[static_cast<std::size_t>(j)] =
        std::clamp(qdot[static_cast<std::size_t>(j)], -max_qdot_, max_qdot_);
  }

  JointState next = state_;
  AssignZeroVelocities(&next, n);
  for (int j = 0; j < n; ++j) {
    next.set_position(j, next.position(j) +
                             qdot[static_cast<std::size_t>(j)] * dt_);
    next.set_velocity(j, qdot[static_cast<std::size_t>(j)]);
  }
  const bool ok = CommitState(next, joint_command);
  if (ok && cscale < 1.0 && status_ == ServoStatus::kOk) {
    status_ = ServoStatus::kDecelerateForCollision;
  }
  return ok;
}

bool DampedLeastSquaresServo::UpdateJointJog(const JointJog& jog,
                                             JointState* joint_command) {
  if (!joint_command) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (GetJointStateDof(state_) <= 0 || halted_) {
    status_ = halted_ ? ServoStatus::kHalted : ServoStatus::kOk;
    *joint_command = state_;
    return !halted_;
  }

  std::unordered_map<std::string, double> vmap;
  const int njog = std::min(jog.joint_names_size(), jog.velocities_size());
  for (int i = 0; i < njog; ++i) {
    vmap[jog.joint_names(i)] =
        std::clamp(jog.velocities(i), -max_qdot_, max_qdot_);
  }

  JointState next = state_;
  const int n = GetJointStateDof(next);
  AssignZeroVelocities(&next, n);
  const double cscale = CollisionApproachScale(state_);
  if (cscale <= 0.0) {
    status_ = ServoStatus::kCollision;
    *joint_command = state_;
    return false;
  }
  for (int j = 0; j < n; ++j) {
    double v = 0.0;
    if (j < next.name_size()) {
      const auto it = vmap.find(next.name(j));
      if (it != vmap.end()) {
        v = it->second * cscale;
      }
    }
    next.set_position(j, next.position(j) + v * dt_);
    next.set_velocity(j, v);
  }
  const bool ok = CommitState(next, joint_command);
  if (ok && cscale < 1.0 && status_ == ServoStatus::kOk) {
    status_ = ServoStatus::kDecelerateForCollision;
  }
  return ok;
}

bool DampedLeastSquaresServo::UpdatePose(const PoseStamped& cmd,
                                         JointState* joint_command) {
  if (!joint_command) {
    return false;
  }
  Pose cur;
  double kp_l = 1.0;
  double kp_a = 1.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!kinematics_ || GetJointStateDof(state_) <= 0 || halted_) {
      status_ = halted_ ? ServoStatus::kHalted : ServoStatus::kOk;
      *joint_command = state_;
      return !halted_;
    }
    if (!kinematics_->GetPositionFK(state_, &cur)) {
      status_ = ServoStatus::kHalted;
      return false;
    }
    kp_l = kp_linear_;
    kp_a = kp_angular_;
  }
  // Re-enter Update without holding mutex (Update locks again).
  return Update(PoseErrorTwist(cur, cmd.pose(), kp_l, kp_a), joint_command);
}

void DampedLeastSquaresServo::Stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  halted_ = true;
  status_ = ServoStatus::kHalted;
  filtered_twist_.Clear();
}

void ServoNode::SetServo(std::shared_ptr<Servo> servo) {
  std::lock_guard<std::mutex> lock(mutex_);
  servo_ = std::move(servo);
}

void ServoNode::SetCommandType(CommandType type) {
  std::lock_guard<std::mutex> lock(mutex_);
  command_type_ = type;
}

void ServoNode::SetCommand(const Twist& twist) {
  std::lock_guard<std::mutex> lock(mutex_);
  twist_ = twist;
  command_type_ = CommandType::kTwist;
}

void ServoNode::SetJointJog(const JointJog& jog) {
  std::lock_guard<std::mutex> lock(mutex_);
  joint_jog_ = jog;
  command_type_ = CommandType::kJointJog;
}

void ServoNode::SetPose(const PoseStamped& pose) {
  std::lock_guard<std::mutex> lock(mutex_);
  pose_ = pose;
  command_type_ = CommandType::kPose;
}

void ServoNode::SetCommandPublisher(CommandPublisher pub) {
  std::lock_guard<std::mutex> lock(mutex_);
  publisher_ = std::move(pub);
}

void ServoNode::SetStatusPublisher(StatusPublisher pub) {
  std::lock_guard<std::mutex> lock(mutex_);
  status_publisher_ = std::move(pub);
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
    CommandType type = CommandType::kTwist;
    Twist twist;
    JointJog jog;
    PoseStamped pose;
    std::shared_ptr<Servo> servo;
    CommandPublisher publisher;
    StatusPublisher status_publisher;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      type = command_type_;
      twist = twist_;
      jog = joint_jog_;
      pose = pose_;
      servo = servo_;
      publisher = publisher_;
      status_publisher = status_publisher_;
    }
    if (servo) {
      JointState out;
      bool ok = false;
      switch (type) {
        case CommandType::kJointJog:
          ok = servo->UpdateJointJog(jog, &out);
          break;
        case CommandType::kPose:
          ok = servo->UpdatePose(pose, &out);
          break;
        case CommandType::kTwist:
        default:
          ok = servo->Update(twist, &out);
          break;
      }
      if (ok && publisher && out.position_size() > 0) {
        publisher(out);
      }
      if (status_publisher) {
        status_publisher(servo->status());
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

}  // namespace servo
}  // namespace manipulation
}  // namespace autonomy
