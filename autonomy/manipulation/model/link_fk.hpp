/*
 * Copyright 2026 The Openbot Authors
 *
 * Lightweight URDF link FK tree (no KDL). Computes link poses from joint state.
 */

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/manipulation/model/robot_model.hpp"

namespace autonomy {
namespace manipulation {
namespace core {

/** @brief Rigid transform as translation plus unit quaternion (w, x, y, z). */
struct Transform {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double qw = 1.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
};

/**
 * @brief One URDF joint edge used by the lightweight FK tree.
 *
 * Origin pose, axis, and optional mimic parameters are stored in SI units /
 * radians as parsed from URDF.
 */
struct LinkFkJoint {
  std::string name;
  std::string type;  // revolute / continuous / prismatic / fixed
  std::string parent;
  std::string child;
  double ox = 0.0;
  double oy = 0.0;
  double oz = 0.0;
  double oroll = 0.0;
  double opitch = 0.0;
  double oyaw = 0.0;
  double ax = 0.0;
  double ay = 0.0;
  double az = 1.0;
  std::string mimic_joint;
  double mimic_multiplier = 1.0;
  double mimic_offset = 0.0;
};

/**
 * @brief Apply mimic relationships: `q_mimic = mult * q_master + offset`.
 * @param[in] joints Joint models that declare mimic targets.
 * @param[in,out] state Joint state updated in place.
 */
void ApplyMimicJoints(const std::vector<JointModel>& joints, JointState* state);

/**
 * @brief Apply mimic relationships using FK-tree joint descriptors.
 * @param[in] joints Link-FK joints that declare mimic targets.
 * @param[in,out] state Joint state updated in place.
 */
void ApplyMimicJoints(const std::vector<LinkFkJoint>& joints, JointState* state);

/**
 * @brief Lightweight URDF link FK tree (no KDL dependency).
 *
 * Loads parent/child joint edges and computes link poses from a @ref JointState.
 */
class LinkFkTree {
 public:
  /**
   * @brief Parse a URDF file into the internal joint/link tree.
   * @param[in] urdf_path Path to URDF file.
   * @param[out] error Optional human-readable failure reason.
   * @return true on successful load.
   */
  bool LoadUrdf(const std::string& urdf_path, std::string* error = nullptr);

  /** @brief Parsed joints in tree order. */
  const std::vector<LinkFkJoint>& Joints() const { return joints_; }

  /** @brief All link names discovered in the URDF. */
  const std::vector<std::string>& LinkNames() const { return links_; }

  /** @brief Root link name of the kinematic tree. */
  std::string RootLink() const { return root_; }

  /**
   * @brief Forward kinematics for all links.
   * @param[in] state Joint positions (mimic applied as needed).
   * @param[out] poses Map from link name to pose (must be non-null).
   * @return false if required joint values are missing.
   */
  bool Compute(const JointState& state,
               std::unordered_map<std::string, Transform>* poses) const;

  /**
   * @brief Pose of a single link under @p state.
   * @param[in] state Joint positions.
   * @param[in] link Link name.
   * @param[out] pose Computed transform (must be non-null).
   * @return true if the link pose was computed.
   */
  bool GetLinkPose(const JointState& state, const std::string& link,
                   Transform* pose) const;

 private:
  std::vector<LinkFkJoint> joints_;
  std::vector<std::string> links_;
  std::string root_;
  // child_link -> joint index
  std::unordered_map<std::string, std::size_t> child_to_joint_;
};

/**
 * @brief Compose rigid transforms: result = @p a * @p b.
 * @param[in] a Left (parent) transform.
 * @param[in] b Right (child) transform.
 * @return Composed transform.
 */
Transform Compose(const Transform& a, const Transform& b);

/** @brief Identity transform (zero translation, unit quaternion). */
Transform IdentityTransform();

}  // namespace core
}  // namespace manipulation
}  // namespace autonomy
