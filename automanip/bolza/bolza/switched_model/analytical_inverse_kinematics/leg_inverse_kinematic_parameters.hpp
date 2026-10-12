#pragma once

/**
 * @file leg_inverse_kinematic_parameters.hpp
 * @brief Contains the relative positions of base, hip, thigh, shank, and foot in the zero configuration.
 */


#include <Eigen/Core>

namespace switched_model {
namespace analytical_inverse_kinematics {

/**
 * Contains the relative positions of base, hip, thigh, shank, and foot in the zero configuration.
 * From there it derives constant parameters for the analytical inverse kinematics computation.
 */
class LegInverseKinematicParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief Constructs a LegInverseKinematicParameters.
   */
  LegInverseKinematicParameters() = default;

  // Construct these parameters with the robot configuration at all 0.0 joint angles
  /**
   * @brief Constructs a LegInverseKinematicParameters.
   * @param [in] positionBaseToHipInBaseFrame: Position Base To Hip In Base Frame.
   * @param [in] positionHipToThighInBaseFrame: Position Hip To Thigh In Base Frame.
   * @param [in] positionThighToShankInBaseFrame: Position Thigh To Shank In Base Frame.
   * @param [in] positionShankToFootInBaseFrame: Position Shank To Foot In Base Frame.
   */
  LegInverseKinematicParameters(const Eigen::Vector3d& positionBaseToHipInBaseFrame, const Eigen::Vector3d& positionHipToThighInBaseFrame,
                                const Eigen::Vector3d& positionThighToShankInBaseFrame,
                                const Eigen::Vector3d& positionShankToFootInBaseFrame);

  // Zero configuration
  Eigen::Vector3d positionBaseToHipInBaseFrame_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d positionHipToThighInBaseFrame_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d positionThighToShankInBaseFrame_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d positionShankToFootInBaseFrame_{Eigen::Vector3d::Zero()};

  // Inverse kinematics parameters (derived from zero configuration)
  Eigen::Vector3d positionBaseToHaaCenterInBaseFrame_{Eigen::Vector3d::Zero()};
  double positionHipToFootYoffset_{0.0};
  double positionHipToFootYoffsetSquared_{0.0};
  double minReach_{0.0};
  double minReach_SP_{0.0};
  double minReachSquared_{0.0};
  double maxReach_SP_{0.0};
  double maxReach_{0.0};
  double maxReachSquared_{0.0};
  double KFEOffset_{0.0};
  double a1_{0.0};
  double a1_squared_{0.0};
  double a2_{0.0};
  double a2_squared_{0.0};

 private:
  /** @brief Initializes all derived parameters. Requires the base parameters to be set. */
  void Initialize();
};

}  // namespace analytical_inverse_kinematics
}  // namespace switched_model
