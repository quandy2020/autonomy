#pragma once

/**
 * @file analytical_inverse_kinematics.hpp
 * @brief Computes the inverse kinematics for a single leg given the position in base frame.
 */


#include "bolza/switched_model/analytical_inverse_kinematics/leg_inverse_kinematic_parameters.hpp"

namespace switched_model {
namespace analytical_inverse_kinematics {

/**
 * Computes the inverse kinematics for a single leg given the position in base frame
 * @param [in] legJoints: will be filled with the resulting [HAA, HFE, KFE] joint angles
 * @param [in] positionBaseToFootInBaseFrame: position of the foot in base frame to compute the IK for.
 * @param [in] parameters: precomputed inverse kinematics parameters
 * @param [in] limb: limb number in {LF = 0, RF, LH, RH}
 */
void GetLimbJointPositionsFromPositionBaseToFootInBaseFrame(Eigen::Vector3d& legJoints,
                                                            const Eigen::Vector3d& positionBaseToFootInBaseFrame,
                                                            const LegInverseKinematicParameters& parameters, size_t limb);

}  // namespace analytical_inverse_kinematics
}  // namespace switched_model
