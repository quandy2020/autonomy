/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file cartesian_servo.hpp
 * @brief Realtime Cartesian servo (MoveIt moveit_servo analogue).
 */

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <automsgs/msgs/control_msgs/joint_jog.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>

#include "autonomy/manipulation/common/collision_interface.hpp"
#include "autonomy/manipulation/model/joint_state_utilities.hpp"
#include "autonomy/manipulation/common/kinematics_interface.hpp"
#include "autonomy/manipulation/model/robot_model.hpp"
#include "autonomy/manipulation/model/simple_robot_model.hpp"
#include "autonomy/manipulation/model/simple_robot_state.hpp"
#include "autonomy/manipulation/motion/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace servo {

/** @brief Active CartesianServoNode command modality (MoveIt CommandType). */
enum class CommandType {
  kTwist = 0,
  kJointJog = 1,
  kPose = 2,
};

/** @brief Outcome of the last CartesianServo::Update (safety / kinematics gates). */
enum class CartesianServoStatus {
  kOk = 0,
  kHalted = 1,
  kCollision = 2,
  kSingularity = 3,
  kJointLimit = 4,
  /** Soft scale applied near obstacle (MoveIt decelerate-for-collision). */
  kDecelerateForCollision = 5,
};

/**
 * @brief Cartesian / joint servo interface: Init / Update / Stop.
 *
 * Implementations map twist / joint-jog / pose commands to joint commands.
 * Wire types are automsgs (automsgs::msgs::geometry_msgs::Twist, automsgs::msgs::control_msgs::JointJog, automsgs::msgs::geometry_msgs::PoseStamped, automsgs::msgs::sensor_msgs::JointState).
 */
class CartesianServo {
 public:
  virtual ~CartesianServo() = default;

  /** @brief Prepare internal state; must succeed before Update. */
  virtual bool Init() = 0;

  /**
   * @brief One servo cycle: map @p twist → joint command.
   * @param[in] twist Desired Cartesian velocity (geometry_msgs::Twist).
   * @param[out] joint_command Updated joint positions / velocities when successful.
   * @return true if a command was produced; false if halted or gated.
   */
  virtual bool Update(const automsgs::msgs::geometry_msgs::Twist& twist, automsgs::msgs::sensor_msgs::JointState* joint_command) = 0;

  /**
   * @brief Integrate joint velocities (MoveIt jointDeltaFromJointJog lite).
   * @param[in] jog Per-joint velocity / displacement command.
   * @param[out] joint_command Updated joint state when successful.
   * @return true if a command was produced; false if unsupported / gated.
   */
  virtual bool UpdateJointJog(const automsgs::msgs::control_msgs::JointJog& jog, automsgs::msgs::sensor_msgs::JointState* joint_command) {
    (void)jog;
    (void)joint_command;
    return false;
  }

  /**
   * @brief Drive tip toward @p cmd.pose via pose-error twist + DLS.
   * @param[in] cmd Desired tip pose (geometry_msgs::PoseStamped).
   * @param[out] joint_command Updated joint state when successful.
   * @return true if a command was produced; false if unsupported / gated.
   */
  virtual bool UpdatePose(const automsgs::msgs::geometry_msgs::PoseStamped& cmd, automsgs::msgs::sensor_msgs::JointState* joint_command) {
    (void)cmd;
    (void)joint_command;
    return false;
  }

  /** @brief Halt servo output (subsequent Update fails until re-enabled). */
  virtual void Stop() = 0;

  /** @brief Latest safety / kinematics status (default kOk). */
  virtual CartesianServoStatus GetStatus() const { return CartesianServoStatus::kOk; }
};

/**
 * @brief Numerical-Jacobian damped least-squares servo with safety gates.
 *
 * Gates on singularity (manipulability), joint limits, and optional scene
 * collision before integrating the twist into joint space.
 */
class DampedLeastSquaresCartesianServo : public CartesianServo {
 public:
  /**
   * @brief IK / FK backend used for Jacobian and pose evaluation.
   * @param[in] kin Kinematics plugin (required before Init).
   */
  void SetKinematics(common::KinematicsInterface::SharedPtr kin);

  /**
   * @brief Optional planning scene for collision gating.
   * @param[in] scene Planning scene; null disables collision checks.
   */
  void SetScene(std::shared_ptr<scene::PlanningScene> scene);

  /**
   * @brief Robot model providing joint limits / DOF.
   * @param[in] model Simple robot model (required before Init).
   */
  void SetModel(std::shared_ptr<const model::SimpleRobotModel> model);

  /**
   * @brief Seed / current joint state for the next Update.
   * @param[in] state Current measured or commanded joints.
   */
  void SetState(const automsgs::msgs::sensor_msgs::JointState& state);

  /**
   * @brief DLS damping λ (larger → more singularity-robust, less accurate).
   * @param[in] lambda Damping factor (≥ 0).
   */
  void SetDamping(double lambda) { lambda_ = lambda; }

  /**
   * @brief Integration timestep in seconds.
   * @param[in] dt Control period (s).
   */
  void SetControlPeriod(double dt) { dt_ = dt; }

  /**
   * @brief Per-joint |q̇| clamp (rad/s or m/s per joint type).
   * @param[in] v Maximum absolute joint velocity.
   */
  void SetMaxJointVelocity(double v) { max_qdot_ = v; }

  /**
   * @brief Manipulability below this → kSingularity and no motion.
   * @param[in] t Yoshikawa manipulability threshold.
   */
  void SetSingularityThreshold(double t) { singularity_threshold_ = t; }

  /**
   * @brief Robot–world distance below this → scale command (m).
   * Soft band is [@p halt_m, @p proximity_m]; inside @p halt_m → hard stop.
   * @param[in] proximity_m Soft decelerate threshold (m).
   * @param[in] halt_m Hard stop clearance (m).
   */
  void SetCollisionProximityThreshold(double proximity_m, double halt_m = 0.01) {
    collision_proximity_threshold_ = proximity_m;
    collision_halt_threshold_ = halt_m;
  }

  /**
   * @brief Low-pass α on twist (0 = freeze, 1 = no filter).
   * @param[in] a Filter coefficient in [0, 1].
   */
  void SetTwistFilterAlpha(double a) { twist_alpha_ = a; }

  /**
   * @brief automsgs::msgs::geometry_msgs::Pose-tracking gains for UpdatePose (m/s per m, rad/s per rad).
   * @param[in] kp_linear Linear pose-error gain.
   * @param[in] kp_angular Angular pose-error gain.
   */
  void SetPoseGains(double kp_linear, double kp_angular) {
    kp_linear_ = kp_linear;
    kp_angular_ = kp_angular;
  }

  /**
   * @brief Status from the most recent Update / Stop.
   * @return Latest CartesianServoStatus.
   */
  CartesianServoStatus GetStatus() const override { return status_; }

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
  bool Update(const automsgs::msgs::geometry_msgs::Twist& twist, automsgs::msgs::sensor_msgs::JointState* joint_command) override;

  /**
   * @brief Integrate @p jog joint velocities with the same safety gates as Update.
   * @param[in] jog Per-joint velocity / displacement command.
   * @param[out] joint_command Integrated joint command on success.
   * @return true if a command was written; false if gated / halted.
   */
  bool UpdateJointJog(const automsgs::msgs::control_msgs::JointJog& jog,
                      automsgs::msgs::sensor_msgs::JointState* joint_command) override;

  /**
   * @brief automsgs::msgs::geometry_msgs::Pose-tracking Update: pose error → twist → DLS Δq.
   * @param[in] cmd Desired tip pose.
   * @param[out] joint_command Integrated joint command on success.
   * @return true if a command was written; false if gated / halted.
   */
  bool UpdatePose(const automsgs::msgs::geometry_msgs::PoseStamped& cmd,
                  automsgs::msgs::sensor_msgs::JointState* joint_command) override;

  /** @brief Set halted and status to kHalted. */
  void Stop() override;

 private:
  /** @brief Finite-difference Jacobian of FK at @p q into @p J (6 × n). */
  bool NumericJacobian(const automsgs::msgs::sensor_msgs::JointState& q,
                       std::vector<std::vector<double>>* J) const;

  /** @brief Yoshikawa manipulability √det(J Jᵀ) (or equivalent scalar). */
  double ComputeManipulability(const std::vector<std::vector<double>>& J) const;

  /** @brief Exponential low-pass on twist components. */
  automsgs::msgs::geometry_msgs::Twist FilterTwist(const automsgs::msgs::geometry_msgs::Twist& twist);

  /**
   * @brief Apply joint-limit + collision gates; may shrink step toward
   * collision-free (sets kDecelerateForCollision).
   */
  bool CommitState(const automsgs::msgs::sensor_msgs::JointState& next, automsgs::msgs::sensor_msgs::JointState* joint_command);

  /** @brief Scale ∈ (0,1] from clearance vs proximity / halt thresholds. */
  double CollisionApproachScale(const automsgs::msgs::sensor_msgs::JointState& q) const;

  common::KinematicsInterface::SharedPtr kinematics_;
  std::shared_ptr<scene::PlanningScene> scene_;
  std::shared_ptr<const model::SimpleRobotModel> model_;
  automsgs::msgs::sensor_msgs::JointState state_;
  automsgs::msgs::geometry_msgs::Twist filtered_twist_;
  double lambda_ = 0.05;
  double dt_ = 0.01;
  double max_qdot_ = 1.0;
  double singularity_threshold_ = 1e-4;
  double collision_proximity_threshold_ = 0.05;
  double collision_halt_threshold_ = 0.01;
  double twist_alpha_ = 0.3;
  double kp_linear_ = 1.0;
  double kp_angular_ = 1.0;
  CartesianServoStatus status_ = CartesianServoStatus::kOk;
  bool halted_ = false;
  mutable std::mutex mutex_;
};

/**
 * @brief Background loop that periodically calls CartesianServo::Update with the latest command.
 *
 * Start() spawns a worker thread; Stop() joins it. SetCommand* are thread-safe.
 */
class CartesianServoNode {
 public:
  /** @brief Publish joint command each cycle (MoveIt servo output lite). */
  using CommandPublisher = std::function<void(const automsgs::msgs::sensor_msgs::JointState&)>;
  /** @brief Optional status sink each cycle. */
  using StatusPublisher = std::function<void(CartesianServoStatus)>;

  /**
   * @brief Install the CartesianServo implementation driven by the worker loop.
   * @param[in] servo Non-null servo instance.
   */
  void SetServo(std::shared_ptr<CartesianServo> servo);

  /**
   * @brief Select which command modality the loop feeds to CartesianServo.
   * @param[in] type automsgs::msgs::geometry_msgs::Twist, joint-jog, or pose.
   */
  void SetCommandType(CommandType type);

  /**
   * @brief Thread-safe store of the latest Cartesian twist command.
   * @param[in] twist Desired twist (used when type is kTwist).
   */
  void SetCommand(const automsgs::msgs::geometry_msgs::Twist& twist);

  /**
   * @brief Thread-safe store of the latest joint-jog command.
   * @param[in] jog Joint jog (used when type is kJointJog).
   */
  void SetJointJog(const automsgs::msgs::control_msgs::JointJog& jog);

  /**
   * @brief Thread-safe store of the latest pose command.
   * @param[in] pose Desired tip pose (used when type is kPose).
   */
  void SetPose(const automsgs::msgs::geometry_msgs::PoseStamped& pose);

  /**
   * @brief Optional sink for integrated joint commands.
   * @param[in] pub Callback invoked each successful cycle (may be empty).
   */
  void SetCommandPublisher(CommandPublisher pub);

  /**
   * @brief Optional sink for CartesianServoStatus each cycle.
   * @param[in] pub Callback invoked each cycle (may be empty).
   */
  void SetStatusPublisher(StatusPublisher pub);

  /** @brief Spawn the worker thread if not already running. */
  void Start();

  /** @brief Request stop and join the worker thread. */
  void Stop();

 private:
  void Loop();

  std::shared_ptr<CartesianServo> servo_;
  CommandType command_type_ = CommandType::kTwist;
  automsgs::msgs::geometry_msgs::Twist twist_;
  automsgs::msgs::control_msgs::JointJog joint_jog_;
  automsgs::msgs::geometry_msgs::PoseStamped pose_;
  CommandPublisher publisher_;
  StatusPublisher status_publisher_;
  std::mutex mutex_;
  std::atomic<bool> running_{false};
  std::thread thread_;
};

}  // namespace servo
}  // namespace manipulation
}  // namespace autonomy
