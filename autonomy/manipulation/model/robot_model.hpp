/*
 * Copyright 2026 The Openbot Authors
 *
 * Robot model domain types (URDF/SRDF). JointState / trajectories use automsgs.
 */

#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "autonomy/manipulation/common/joint_state_util.hpp"
#include "autonomy/manipulation/common/msg_types.hpp"

namespace autonomy {
namespace manipulation {
namespace core {

/** @brief Position / velocity / acceleration limits for one joint. */
struct JointLimits {
  double min_position = -3.141592653589793;
  double max_position = 3.141592653589793;
  double max_velocity = 1.0;
  double max_acceleration = 2.0;
  bool has_position_limits = true;
};

/** @brief URDF-derived joint description including limits and mimic. */
struct JointModel {
  std::string name;
  std::string type;  // revolute / continuous / prismatic / fixed
  std::string parent_link;
  std::string child_link;
  JointLimits limits;
  std::string mimic_joint;
  double mimic_factor = 1.0;
  double mimic_offset = 0.0;
};

/** @brief Link identity and its parent joint name. */
struct LinkModel {
  std::string name;
  std::string parent_joint;
};

/** @brief Planning group: joints, links, and optional chain frames. */
struct JointModelGroup {
  std::string name;
  std::vector<std::string> joint_names;
  std::vector<std::string> link_names;
  std::string base_frame;
  std::string tip_frame;
  bool is_chain = true;
};

/**
 * @brief Abstract robot kinematic model loaded from URDF/SRDF.
 */
class RobotModel {
 public:
  virtual ~RobotModel() = default;

  /**
   * @brief Load model definition from URDF and SRDF text or paths.
   * @param[in] urdf URDF source (implementation-defined: path or XML).
   * @param[in] srdf SRDF source (implementation-defined: path or XML).
   * @return true on successful load.
   */
  virtual bool Load(const std::string& urdf, const std::string& srdf) = 0;

  /**
   * @brief Joint names belonging to a planning group.
   * @param[in] group Planning group name.
   * @return Ordered joint names for @p group (empty if unknown).
   */
  virtual std::vector<std::string> GetJointNames(
      const std::string& group) const = 0;

  /** @brief All joints in the model. */
  virtual const std::vector<JointModel>& Joints() const = 0;

  /**
   * @brief Look up a planning group by name.
   * @param[in] name Group name.
   * @return Pointer to the group, or nullptr if missing.
   */
  virtual const JointModelGroup* GetJointModelGroup(
      const std::string& name) const = 0;

  /**
   * @brief Look up limits for a joint.
   * @param[in] joint_name Joint name.
   * @return Pointer to limits, or nullptr if missing.
   */
  virtual const JointLimits* GetJointLimits(
      const std::string& joint_name) const = 0;
};

/**
 * @brief Mutable robot configuration (joint positions / group subsets).
 */
class RobotState {
 public:
  virtual ~RobotState() = default;

  /**
   * @brief Replace the full joint state.
   * @param[in] state Named positions (and optional velocities).
   */
  virtual void SetJointState(const JointState& state) = 0;

  /** @brief Current full joint state. */
  virtual JointState GetJointState() const = 0;

  /**
   * @brief Set positions for joints in a planning group.
   * @param[in] group Planning group name.
   * @param[in] positions Values ordered as @ref RobotModel::GetJointNames.
   */
  virtual void SetJointGroupPositions(const std::string& group,
                                      const std::vector<double>& positions) = 0;

  /**
   * @brief Read positions for joints in a planning group.
   * @param[in] group Planning group name.
   * @return Positions ordered as @ref RobotModel::GetJointNames.
   */
  virtual std::vector<double> GetJointGroupPositions(
      const std::string& group) const = 0;

  /**
   * @brief Linearly interpolate this state toward @p to.
   * @param[in] to Target state.
   * @param[in] t Interpolation factor in [0, 1].
   * @param[out] result Interpolated state (must be non-null).
   */
  virtual void Interpolate(const RobotState& to, double t,
                           RobotState* result) const = 0;
};

/**
 * @brief Convert JointTrajectory to flat joint-name / series form (debug / IO).
 */
struct FlatTrajectory {
  std::vector<std::string> joint_names;
  std::vector<std::vector<double>> positions;  // [time][joint]
  std::vector<double> time_from_start;
};

/**
 * @brief Convert a JointTrajectory to flat form.
 * @param[in] traj Source trajectory.
 * @return Flattened representation.
 */
FlatTrajectory ToFlat(const RobotTrajectory& traj);

/**
 * @brief Rebuild a JointTrajectory from flat form.
 * @param[in] flat Flattened trajectory.
 * @return JointTrajectory message.
 */
RobotTrajectory FromFlat(const FlatTrajectory& flat);

}  // namespace core
}  // namespace manipulation
}  // namespace autonomy
