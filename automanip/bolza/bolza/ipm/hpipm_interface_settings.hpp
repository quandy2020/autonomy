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
 * @file hpipm_interface_settings.hpp
 * @brief HPIPM interface.
 */



extern "C" {
#include <hpipm_common.h>
}

#include <bolza/core/types.hpp>

namespace bolza {
namespace ipm {

/**
 * Settings forwarded to HPIPM
 */
struct HpipmSettings {
  hpipm_mode hpipmMode = hpipm_mode::SPEED;
  int iter_max = 30;
  scalar_t alpha_min = 1e-12;
  scalar_t mu0 = 1e1;
  scalar_t tol_stat = 1e-6;  // res_g_max
  scalar_t tol_eq = 1e-8;    // res_b_max
  scalar_t tol_ineq = 1e-8;  // res_d_max
  scalar_t tol_comp = 1e-8;  // res_m_max
  scalar_t reg_prim = 1e-12;
  int warm_start = 0;
  int pred_corr = 1;
  int ric_alg = 0;  // square root ricatti recursion
};

std::ostream& operator<<(std::ostream& stream, const HpipmSettings& settings);

}  // namespace ipm
}  // namespace bolza