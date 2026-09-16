/*
 * Copyright 2026 The Openbot Authors
 *
 * Realtime Cartesian servo (MoveIt moveit_servo analogue).
 */

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include "autonomy/manipulation/collision/collision_detector.hpp"
#include "autonomy/manipulation/core/robot_model.hpp"
#include "autonomy/manipulation/core/simple_robot_model.hpp"
#include "autonomy/manipulation/kinematics/kinematics_base.hpp"
#include "autonomy/manipulation/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace servo {

/**
 * @brief Cartesian twist command in the servo frame (linear m/s, angular rad/s).
 */
struct TwistCommand {
  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;
  double wx = 0.0;
  double wy = 0.0;
  double wz = 0.0;
};

/** @brief Outcome of the last Servo::Update (safety / kinematics gates). */
enum class ServoStatus {
  kOk = 0,
  kHalted = 1,
  kCollision = 2,
  kSingularity = 3,
  kJointLimit = 4,
};

/**
 * @brief Cartesian servo interface: Init / Update / Stop.
 *
 * Implementations map twist commands to joint commands each cycle.
 */
class Servo {
 public:
  virtual ~Servo() = default;

  /** @brief Prepare internal state; must succeed before Update. */
  virtual bool Init() = 0;

  /**
   * @brief One servo cycle: map @p twist → joint command.
   * @param[in] twist Desired Cartesian velocity.
   * @param[out] joint_command Updated joint positions / velocities when successful.
   * @return true if a command was produced; false if halted or gated.
   */
  virtual bool Update(const TwistCommand& twist,
                      core::JointState* joint_command) = 0;

  /** @brief Halt servo output (subsequent Update fails until re-enabled). */
  virtual void Stop() = 0;
};

/**
 * @brief Numerical-Jacobian damped least-squares servo with safety gates.
 *
 * Gates on singularity (manipulability), joint limits, and optional scene
 * collision before integrating the twist into joint space.
 */
class DampedLeastSquaresServo : public Servo {
 public:
  /** @brief IK / FK backend used for Jacobian and pose evaluation. */
  void SetKinematics(std::shared_ptr<kinematics::KinematicsBase> kin);

  /** @brief Optional planning scene for collision gating. */
  void SetScene(std::shared_ptr<scene::PlanningScene> scene);

  /** @brief Robot model providing joint limits / DOF. */
  void SetModel(std::shared_ptr<const core::SimpleRobotModel> model);

  /** @brief Seed / current joint state for the next Update. */
  void SetState(const core::JointState& state);

  /** @brief DLS damping λ (larger → more singularity-robust, less accurate). */
  void SetDamping(double lambda) { lambda_ = lambda; }

  /** @brief Integration timestep in seconds. */
  void SetDt(double dt) { dt_ = dt; }

  /** @brief Per-joint |q̇| clamp (rad/s or m/s per joint type). */
  void SetMaxJointVelocity(double v) { max_qdot_ = v; }

  /** @brief Manipulability below this → kSingularity and no motion. */
  void SetSingularityThreshold(double t) { singularity_threshold_ = t; }

  /** @brief Low-pass α on twist (0 = freeze, 1 = no filter). */
  void SetTwistFilterAlpha(double a) { twist_alpha_ = a; }

  /** @brief @return Status from the most recent Update / Stop. */
  ServoStatus status() const { return status_; }

  /**
   * @brief Validate dependencies and clear halted state.
   * @return false if kinematics or model is missing.
   */
  bool Init() override;

  /**
   * @brief Filter twist, solve DLS Δq, apply safety gates, integrate state.
   * @param[in] twist Desired Cartesian velocity.
   * @param[out] joint_command Integrated joint command on success.
   * @return true if a command was written; false if gated / halted.
   */
  bool Update(const TwistCommand& twist,
              core::JointState* joint_command) override;

  /** @brief Set halted and status to kHalted. */
  void Stop() override;

 private:
  /** @brief Finite-difference Jacobian of FK at @p q into @p J (6 × n). */
  bool NumericJacobian(const core::JointState& q,
                       std::vector<std::vector<double>>* J) const;

  /** @brief Yoshikawa manipulability √det(J Jᵀ) (or equivalent scalar). */
  double Manipulability(const std::vector<std::vector<double>>& J) const;

  /** @brief Exponential low-pass on twist components. */
  TwistCommand FilterTwist(const TwistCommand& twist);

  std::shared_ptr<kinematics::KinematicsBase> kinematics_;
  std::shared_ptr<scene::PlanningScene> scene_;
  std::shared_ptr<const core::SimpleRobotModel> model_;
  core::JointState state_;
  TwistCommand filtered_twist_;
  double lambda_ = 0.05;
  double dt_ = 0.01;
  double max_qdot_ = 1.0;
  double singularity_threshold_ = 1e-4;
  double twist_alpha_ = 0.3;
  ServoStatus status_ = ServoStatus::kOk;
  bool halted_ = false;
  mutable std::mutex mutex_;
};

/**
 * @brief Background loop that periodically calls Servo::Update with the latest twist.
 *
 * Start() spawns a worker thread; Stop() joins it. SetCommand is thread-safe.
 */
class ServoNode {
 public:
  /** @brief Install the servo implementation driven by the worker loop. */
  void SetServo(std::shared_ptr<Servo> servo);

  /** @brief Replace the latest twist command consumed by Loop(). */
  void SetCommand(const TwistCommand& twist);

  /** @brief Spawn the servo worker thread if not already running. */
  void Start();

  /** @brief Request stop and join the worker thread. */
  void Stop();

 private:
  /** @brief Worker: read twist, call Update, sleep for servo dt. */
  void Loop();

  std::shared_ptr<Servo> servo_;
  TwistCommand twist_;
  std::mutex mutex_;
  std::atomic<bool> running_{false};
  std::thread thread_;
};

}  // namespace servo
}  // namespace manipulation
}  // namespace autonomy
