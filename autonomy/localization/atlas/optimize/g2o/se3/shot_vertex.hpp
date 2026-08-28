/*
 * Alias PLP g2o::se3::shot_vertex to Atlas internal implementation.
 */
#pragma once

#include "autonomy/localization/atlas/optimize/internal/se3/shot_vertex.hpp"

namespace autonomy::localization::atlas::optimize::plp_g2o::se3 {
using shot_vertex = internal::se3::shot_vertex;
}  // namespace autonomy::localization::atlas::optimize::plp_g2o::se3
