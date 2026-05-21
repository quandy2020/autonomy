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

#include <algorithm>
#include <cmath>
#include <memory>

#include "autonomy/control/controller/mppi_controller/models/control_sequence.hpp"
#include "autonomy/control/controller/mppi_controller/models/constraints.hpp"
#include "autonomy/control/controller/mppi_controller/models/state.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {

inline float Clamp(float lower_bound, float upper_bound, float input) {
    return std::max(lower_bound, std::min(upper_bound, input));
}

class MotionModel
{
public:
    virtual ~MotionModel() = default;

    void initialize(const models::ControlConstraints& control_constraints,
                    float model_dt) {
        control_constraints_ = control_constraints;
        model_dt_ = model_dt;
    }

    virtual void predict(models::State& state) {
        const bool is_holo = isHolonomic();
        const float max_delta_vx = model_dt_ * control_constraints_.ax_max;
        const float min_delta_vx = model_dt_ * control_constraints_.ax_min;
        const float max_delta_vy = model_dt_ * control_constraints_.ay_max;
        const float min_delta_vy = model_dt_ * control_constraints_.ay_min;
        const float max_delta_wz = model_dt_ * control_constraints_.az_max;

        const unsigned int n_cols = state.vx.cols();
        for (unsigned int i = 1; i < n_cols; ++i) {
            auto lower_bound_vx =
                (state.vx.col(i - 1) > 0)
                    .select(state.vx.col(i - 1) + min_delta_vx,
                            state.vx.col(i - 1) - max_delta_vx);
            auto upper_bound_vx =
                (state.vx.col(i - 1) > 0)
                    .select(state.vx.col(i - 1) + max_delta_vx,
                            state.vx.col(i - 1) - min_delta_vx);

            state.cvx.col(i - 1) = state.cvx.col(i - 1)
                                       .cwiseMax(lower_bound_vx)
                                       .cwiseMin(upper_bound_vx);
            state.vx.col(i) = state.cvx.col(i - 1);

            state.cwz.col(i - 1) = state.cwz.col(i - 1)
                                       .cwiseMax(state.wz.col(i - 1) -
                                                 max_delta_wz)
                                       .cwiseMin(state.wz.col(i - 1) +
                                                 max_delta_wz);
            state.wz.col(i) = state.cwz.col(i - 1);

            if (is_holo) {
                auto lower_bound_vy =
                    (state.vy.col(i - 1) > 0)
                        .select(state.vy.col(i - 1) + min_delta_vy,
                                state.vy.col(i - 1) - max_delta_vy);
                auto upper_bound_vy =
                    (state.vy.col(i - 1) > 0)
                        .select(state.vy.col(i - 1) + max_delta_vy,
                                state.vy.col(i - 1) - min_delta_vy);
                state.cvy.col(i - 1) = state.cvy.col(i - 1)
                                           .cwiseMax(lower_bound_vy)
                                           .cwiseMin(upper_bound_vy);
                state.vy.col(i) = state.cvy.col(i - 1);
            }
        }
    }

    virtual bool isHolonomic() = 0;

    virtual void applyConstraints(models::ControlSequence& /*control_sequence*/) {
    }

protected:
    float model_dt_{0.0f};
    models::ControlConstraints control_constraints_;
};

class DiffDriveMotionModel : public MotionModel
{
public:
    bool isHolonomic() override { return false; }
};

class OmniMotionModel : public MotionModel
{
public:
    bool isHolonomic() override { return true; }
};

class AckermannMotionModel : public MotionModel
{
public:
    explicit AckermannMotionModel(float min_turning_r)
        : min_turning_r_(min_turning_r) {}

    bool isHolonomic() override { return false; }

    void applyConstraints(models::ControlSequence& control_sequence) override {
        const auto vx_ptr = control_sequence.vx.data();
        auto wz_ptr = control_sequence.wz.data();
        const int steps = static_cast<int>(control_sequence.vx.size());
        for (int i = 0; i < steps; ++i) {
            const float wz_constrained = std::fabs(*(vx_ptr + i) / min_turning_r_);
            float& wz_curr = *(wz_ptr + i);
            wz_curr = Clamp(-wz_constrained, wz_constrained, wz_curr);
        }
    }

    float getMinTurningRadius() const { return min_turning_r_; }

private:
    float min_turning_r_{0.2f};
};

}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
