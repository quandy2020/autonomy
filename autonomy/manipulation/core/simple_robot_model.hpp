/*
 * Copyright 2026 The Openbot Authors
 *
 * Concrete robot model / state with groups, limits, SRDF.
 */

#pragma once

#include <memory>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "autonomy/manipulation/core/robot_model.hpp"
#include "autonomy/manipulation/core/link_fk.hpp"

namespace autonomy {
namespace manipulation {
namespace core {

/**
 * @brief Concrete @ref RobotModel with URDF/SRDF groups, limits, and ACM seeds.
 */
class SimpleRobotModel : public RobotModel {
 public:
  /**
   * @brief Load joints/groups/limits from URDF and SRDF paths.
   * @param[in] urdf Path to URDF file.
   * @param[in] srdf Path to SRDF file.
   * @return true on successful load.
   */
  bool Load(const std::string& urdf, const std::string& srdf) override;

  /**
   * @brief Joint names for a planning group.
   * @param[in] group Planning group name.
   * @return Ordered joint names (empty if unknown).
   */
  std::vector<std::string> GetJointNames(
      const std::string& group) const override;

  /**
   * @brief Override or create a group's joint list.
   * @param[in] group Planning group name.
   * @param[in] joint_names Ordered joint names for the group.
   */
  void SetGroupJoints(const std::string& group,
                      std::vector<std::string> joint_names);

  /** @brief All movable joint names known to the model. */
  const std::vector<std::string>& AllJoints() const { return all_joints_; }

  /** @brief All joint models. */
  const std::vector<JointModel>& Joints() const override { return joints_; }

  /**
   * @brief Look up a planning group by name.
   * @param[in] name Group name.
   * @return Pointer to the group, or nullptr if missing.
   */
  const JointModelGroup* GetJointModelGroup(
      const std::string& name) const override;

  /**
   * @brief Look up limits for a joint.
   * @param[in] joint_name Joint name.
   * @return Pointer to limits, or nullptr if missing.
   */
  const JointLimits* GetJointLimits(
      const std::string& joint_name) const override;

  /**
   * @brief Set base/tip frames for a chain-style group.
   * @param[in] group Planning group name.
   * @param[in] base Base frame / link.
   * @param[in] tip Tip frame / link.
   */
  void SetGroupMeta(const std::string& group, const std::string& base,
                    const std::string& tip);

  /**
   * @brief Whether a planning group exists.
   * @param[in] group Planning group name.
   * @return true if @p group is registered.
   */
  bool HasGroup(const std::string& group) const {
    return groups_.count(group) > 0;
  }

  /**
   * @brief SRDF `disable_collisions` pairs used to seed allowed-collision matrix.
   * @return Link-name pairs that should not collide-check against each other.
   */
  const std::vector<std::pair<std::string, std::string>>&
  DisabledCollisions() const {
    return disabled_collisions_;
  }

 private:
  std::vector<JointModel> joints_;
  std::vector<std::string> all_joints_;
  std::unordered_map<std::string, JointModelGroup> groups_;
  std::unordered_map<std::string, JointLimits> limits_;
  std::vector<std::pair<std::string, std::string>> disabled_collisions_;
};

/**
 * @brief Thread-safe @ref RobotState backed by an optional @ref SimpleRobotModel.
 *
 * Optionally holds a @ref LinkFkTree for link-frame forward kinematics.
 */
class SimpleRobotState : public RobotState {
 public:
  /**
   * @brief Construct with an optional shared model.
   * @param[in] model Robot model used for groups/limits (may be null).
   */
  explicit SimpleRobotState(std::shared_ptr<const SimpleRobotModel> model =
                                nullptr);

  /**
   * @brief Replace the associated robot model.
   * @param[in] model New shared model (may be null).
   */
  void SetModel(std::shared_ptr<const SimpleRobotModel> model);

  /**
   * @brief Replace the full joint state.
   * @param[in] state Named positions (and optional velocities).
   */
  void SetJointState(const JointState& state) override;

  /** @brief Current full joint state. */
  JointState GetJointState() const override;

  /**
   * @brief Set positions for joints in a planning group.
   * @param[in] group Planning group name.
   * @param[in] positions Values ordered as the group's joint names.
   */
  void SetJointGroupPositions(const std::string& group,
                              const std::vector<double>& positions) override;

  /**
   * @brief Read positions for joints in a planning group.
   * @param[in] group Planning group name.
   * @return Positions ordered as the group's joint names.
   */
  std::vector<double> GetJointGroupPositions(
      const std::string& group) const override;

  /**
   * @brief Linearly interpolate this state toward @p to.
   * @param[in] to Target state.
   * @param[in] t Interpolation factor in [0, 1].
   * @param[out] result Interpolated state (must be non-null).
   */
  void Interpolate(const RobotState& to, double t,
                   RobotState* result) const override;

  /** @brief Clamp positions into model joint limits (no-op without model). */
  void EnforceBounds();

  /**
   * @brief Whether current positions lie within model joint limits.
   * @return true if in bounds (or no model / limits).
   */
  bool SatisfiesBounds() const;

  /**
   * @brief Euclidean distance between this state and @p other in joint space.
   * @param[in] other Other robot state.
   * @return L2 distance over matching joint positions.
   */
  double Distance(const RobotState& other) const;

  /**
   * @brief Load URDF kinematic tree for link FK (independent of KDL).
   * @param[in] urdf_path Path to URDF file.
   * @return true on successful load.
   */
  bool LoadLinkTree(const std::string& urdf_path);

  /**
   * @brief Pose of a single link under the current joint state.
   * @param[in] link Link name.
   * @param[out] pose Computed transform (must be non-null).
   * @return true if the link pose was computed.
   */
  bool GetLinkPose(const std::string& link, Transform* pose) const;

  /**
   * @brief Forward kinematics for all links under the current joint state.
   * @param[out] poses Map from link name to pose (must be non-null).
   * @return true if poses were computed.
   */
  bool ComputeLinkPoses(
      std::unordered_map<std::string, Transform>* poses) const;

  /** @brief Underlying link FK tree (may be empty until LoadLinkTree). */
  const LinkFkTree& LinkTree() const { return link_tree_; }

 private:
  std::shared_ptr<const SimpleRobotModel> model_;
  JointState state_;
  LinkFkTree link_tree_;
  mutable std::shared_mutex mutex_;
};

}  // namespace core
}  // namespace manipulation
}  // namespace autonomy
