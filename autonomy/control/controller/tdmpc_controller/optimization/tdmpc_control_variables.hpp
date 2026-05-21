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

#include <vector>

#include "autonomy/control/controller/nmpc_controller/optimization/optimization_guard.hpp"
#include "autonomy/common/optimization/core/variable_set.hpp"
#include "autonomy/control/controller/nmpc_controller/models/kinematic_model.hpp"
#include "autonomy/control/controller/tdmpc_controller/models_alias.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace tdmpc {
namespace mpc_opt {

namespace ifopt = ::autonomy::common::optimization;

class TdmpcControlVariables : public ifopt::VariableSet
{
public:
    static constexpr const char* kName = "tdmpc_controls";

    TdmpcControlVariables(int horizon,
                          const models::KinematicModel& kinematic_model);

    void SetVariables(const ifopt::Component::VectorXd& x) override;
    ifopt::Component::VectorXd GetValues() const override;
    ifopt::Component::VecBound GetBounds() const override;

    const std::vector<models::Control2D>& controls() const { return controls_; }
    void SetControls(const std::vector<models::Control2D>& controls);
    void ShiftWarmStart();

private:
    models::KinematicModel kinematic_model_;
    std::vector<models::Control2D> controls_;
};

}  // namespace mpc_opt
}  // namespace tdmpc
}  // namespace controller
}  // namespace control
}  // namespace autonomy
