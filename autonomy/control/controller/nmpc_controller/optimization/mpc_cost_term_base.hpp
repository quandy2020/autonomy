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
 * @file mpc_cost_term_base.hpp
 * @brief Thin wrapper around ifopt CostTerm (must not include kinematic_model.hpp).
 *
 * Some third-party headers define CostTerm as a macro; keep this include minimal.
 */

#include "autonomy/control/controller/nmpc_controller/optimization/optimization_guard.hpp"
#include "autonomy/common/optimization/core/cost_term.hpp"

#ifdef CostTerm
#undef CostTerm
#endif

namespace autonomy {
namespace control {
namespace controller {
namespace nmpc {
namespace mpc_opt {

struct MpcCostTermBase : ::autonomy::common::optimization::CostTerm {
    explicit MpcCostTermBase(const char* name)
        : ::autonomy::common::optimization::CostTerm(name) {}
};

}  // namespace mpc_opt
}  // namespace nmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
