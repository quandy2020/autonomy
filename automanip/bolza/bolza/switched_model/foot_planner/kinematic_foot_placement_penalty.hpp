//
// Created by rgrandia on 17.02.22.
//

#pragma once

/**
 * @file kinematic_foot_placement_penalty.hpp
 * @brief Configuration for the kinematic penalty taking into account:.
 */


#include "bolza/switched_model/core/switched_model.hpp"

namespace switched_model {

/**
 * Configuration for the kinematic penalty taking into account:
 *  - maximum leg extension
 *  - the hip not rotation inwards w.r.t gravity. Prevents placing the foot too far below the body.
 */
struct ApproximateKinematicsConfig {
  scalar_t maxLegExtension;
  scalar_t kinematicPenaltyWeight;
};

/**
 * Compute the kinematic foot placement penalty.
 *
 * @param [in] footPositionInHip: relative foot position in the hip frame
 * @param [in] gravityNormalInHip: gravity vector expressed in the hip frame
 * @param [in] config: settings
 * @return positive kinematic penalty value.
 */
scalar_t ComputeKinematicPenalty(const vector3_t& footPositionInHip, const vector3_t& gravityNormalInHip,
                                 const ApproximateKinematicsConfig& config);

/**
 * Compute the kinematic foot placement penalty.
 *
 * @param [in] footPositionInWorld: foot position in the world frame
 * @param [in] hipPositionInWorld: hip position in the world frame
 * @param [in] rotationHipToWorld: rotation hip to world. Rotation around the positive x-axis of the hip frame should make the leg rotate
 * outwards.
 * @param [in] config: settings
 * @return positive kinematic penalty value.
 */
scalar_t ComputeKinematicPenalty(const vector3_t& footPositionInWorld, const vector3_t& hipPositionInWorld,
                                 const matrix3_t& rotationHipToWorld, const ApproximateKinematicsConfig& config);

}  // namespace switched_model