//
// Created by rgrandia on 08.12.21.
//

#pragma once

/**
 * @file torque_approximation.hpp
 * @brief Approximate joint torques with J(q)^T F, i.e. neglecting leg dynamics.
 */


#include "bolza/switched_model/core/kinematics_model_base.hpp"
#include "bolza/switched_model/core/switched_model.hpp"

namespace switched_model {

/**
 * Approximate joint torques with J(q)^T F, i.e. neglecting leg dynamics.
 */
template <typename SCALAR_T>
/**
 * @brief Torque Approximation.
 * @param [in] jointPositions: Joint Positions.
 * @param [in] contactForcesInBase: Contact Forces In Base.
 * @param [in] kinematics: Kinematics.
 * @return Result value.
 */
joint_coordinate_s_t<SCALAR_T> TorqueApproximation(const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                   const feet_array_t<vector3_s_t<SCALAR_T>>& contactForcesInBase,
                                                   const KinematicsModelBase<SCALAR_T>& kinematics);

// Explicit instantiations
extern template joint_coordinate_s_t<scalar_t> TorqueApproximation<scalar_t>(const joint_coordinate_s_t<scalar_t>& jointPositions,
                                                                             const feet_array_t<vector3_s_t<scalar_t>>& contactForcesInBase,
                                                                             const KinematicsModelBase<scalar_t>& kinematics);
extern template joint_coordinate_s_t<ad_scalar_t> TorqueApproximation<ad_scalar_t>(
    const joint_coordinate_s_t<ad_scalar_t>& jointPositions, const feet_array_t<vector3_s_t<ad_scalar_t>>& contactForcesInBase,
    const KinematicsModelBase<ad_scalar_t>& kinematics);

}  // namespace switched_model