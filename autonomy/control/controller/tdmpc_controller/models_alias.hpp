/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

/**
 * @file models_alias.hpp
 * @brief Re-export NMPC kinematic types for T-D MPC (shared planar models).
 *
 * Undef Coin/Ipopt macros that collide with namespace tokens (nmpc, optimization).
 */

#ifdef nmpc
#undef nmpc
#endif
#ifdef optimization
#undef optimization
#endif
#ifdef CostTerm
#undef CostTerm
#endif
#ifdef opt
#undef opt
#endif
#ifdef ifopt
#undef ifopt
#endif

#include "autonomy/control/controller/nmpc_controller/models/kinematic_model.hpp"
#include "autonomy/control/controller/nmpc_controller/models/kinematic_pose.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace tdmpc {

namespace models = ::autonomy::control::controller::nmpc::models;

}  // namespace tdmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
