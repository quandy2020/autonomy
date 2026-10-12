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

#include <cstddef>
#include <string>
#include <vector>

#include <bolza/core/types.hpp>
#include <robotic_assets/package_path.hpp>

namespace bolza {
namespace centroidal_model {

enum anymal : size_t {
  STATE_DIM = 6 + 6 + 12,  // centroidal momentum, generalized coordinates
  INPUT_DIM = 4 * 3 + 12,  // end effector forces, joint configuration
};

static const std::vector<std::string> anymal3DofContactNames = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
static const std::vector<std::string> anymal6DofContactNames = {};
static const std::string anymalUrdfFile = bolza::robotic_assets::GetPath() + "/resources/anymal_c/urdf/anymal.urdf";

inline bolza::vector_t GetInitialState() {
  bolza::vector_t x0 = bolza::vector_t::Zero(anymal::STATE_DIM);

  x0(0) = 0.0;  // vcom_x
  x0(1) = 0.0;  // vcom_y
  x0(2) = 0.0;  // vcom_z
  x0(3) = 0.0;  // L_x / mass
  x0(4) = 0.0;  // L_y / mass
  x0(5) = 0.0;  // L_z / mass

  // Base Pose: [position, orientation]
  x0(6) = 0.0;   // p_base_x
  x0(7) = 0.0;   // p_base_y
  x0(8) = 0.53;  // p_base_z
  x0(9) = 0.0;   // theta_base_z
  x0(10) = 0.0;  // theta_base_y
  x0(11) = 0.0;  // theta_base_x

  // Leg Joint Positions: [LF, LH, RF, RH]
  x0(12) = -0.10;  // LF_HAA
  x0(13) = 0.7;    // LF_HFE
  x0(14) = -1.0;   // LF_KFE
  x0(15) = -0.10;  // LH_HAA
  x0(16) = -0.7;   // LH_HFE
  x0(17) = 1.0;    // LH_KFE
  x0(18) = 0.10;   // RF_HAA
  x0(19) = 0.7;    // RF_HFE
  x0(20) = -1.0;   // RF_KFE
  x0(21) = 0.10;   // RH_HAA
  x0(22) = -0.7;   // RH_HFE
  x0(23) = 1.0;    // RH_KFE

  return x0;
}

}  // namespace centroidal_model
}  // namespace bolza
