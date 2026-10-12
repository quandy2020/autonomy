/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

/**
 * @file types.hpp
 * @brief Scalar type.
 */


#include <Eigen/Dense>
#include <Eigen/StdVector>

// CppAD
#include <cppad/cg.hpp>

#include <bolza/core/types.hpp>

namespace bolza {

/** Scalar type. */
using ad_base_t = CppAD::cg::CG<scalar_t>;

using ad_scalar_t = CppAD::AD<ad_base_t>;

/** Dynamic-size vector type. */
using ad_vector_t = Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, 1>;

using ad_matrix_t = Eigen::Matrix<ad_scalar_t, Eigen::Dynamic, Eigen::Dynamic>;

}  // namespace bolza
