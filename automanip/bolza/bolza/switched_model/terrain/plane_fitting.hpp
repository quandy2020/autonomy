//
// Created by rgrandia on 27.11.20.
//

#pragma once

/**
 * @file plane_fitting.hpp
 * @brief plane fitting (bolza).
 */


#include "bolza/switched_model/core/switched_model.hpp"
#include "bolza/switched_model/terrain/terrain_plane.hpp"

namespace switched_model {

/**
 * @brief Estimate Plane.
 * @param [in] regressionPoints: Regression Points.
 * @return Result value.
 */
NormalAndPosition EstimatePlane(const std::vector<vector3_t>& regressionPoints);

}  // namespace switched_model
