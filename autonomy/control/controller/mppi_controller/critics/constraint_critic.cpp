/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/mppi_controller/critics/constraint_critic.hpp"

#include "autonomy/control/controller/mppi_controller/motion_models.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {
namespace critics {

void ConstraintCritic::Configure(
    const proto::MPPIControllerOptions& options,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> /*costmap_wrapper*/) {
    const auto& cfg = options.constraint_critic();
    enabled_ = cfg.enabled();
    power_ = cfg.cost_power() > 0 ? cfg.cost_power() : 1;
    weight_ = static_cast<float>(cfg.cost_weight() > 0.0 ? cfg.cost_weight()
                                                           : 4.0);

    const float vx_max = static_cast<float>(options.vx_max());
    const float vy_max = static_cast<float>(options.vy_max());
    const float vx_min = static_cast<float>(options.vx_min());
    const float min_sgn = vx_min > 0.0f ? 1.0f : -1.0f;
    max_vel_ = std::sqrt(vx_max * vx_max + vy_max * vy_max);
    min_vel_ = min_sgn * std::sqrt(vx_min * vx_min + vy_max * vy_max);
}

void ConstraintCritic::score(CriticData& data) {
    if (!enabled_) {
        return;
    }

    if (auto* diff = dynamic_cast<DiffDriveMotionModel*>(data.motion_model.get())) {
        (void)diff;
        if (power_ > 1) {
            data.costs +=
                (((((data.state.vx - max_vel_).max(0.0f) +
                    (min_vel_ - data.state.vx).max(0.0f)) *
                   data.model_dt)
                      .rowwise()
                      .sum()
                      .eval()) *
                 weight_)
                    .pow(power_)
                    .eval();
        } else {
            data.costs +=
                (((((data.state.vx - max_vel_).max(0.0f) +
                    (min_vel_ - data.state.vx).max(0.0f)) *
                   data.model_dt)
                      .rowwise()
                      .sum()
                      .eval()) *
                 weight_)
                    .eval();
        }
        return;
    }

    if (auto* omni = dynamic_cast<OmniMotionModel*>(data.motion_model.get())) {
        (void)omni;
        auto& vx = data.state.vx;
        const int n_rows = static_cast<int>(data.state.vx.rows());
        const int n_cols = static_cast<int>(data.state.vx.cols());
        Eigen::ArrayXXf sgn(n_rows, n_cols);
        sgn = vx.unaryExpr([](float x) { return std::copysignf(1.0f, x); });
        auto vel_total =
            sgn * (data.state.vx.square() + data.state.vy.square()).sqrt();
        if (power_ > 1) {
            data.costs +=
                ((((vel_total - max_vel_).max(0.0f) +
                   (min_vel_ - vel_total).max(0.0f)) *
                  data.model_dt)
                     .rowwise()
                     .sum()
                     .eval() *
                 weight_)
                    .pow(power_)
                    .eval();
        } else {
            data.costs +=
                ((((vel_total - max_vel_).max(0.0f) +
                   (min_vel_ - vel_total).max(0.0f)) *
                  data.model_dt)
                     .rowwise()
                     .sum()
                     .eval() *
                 weight_)
                    .eval();
        }
        return;
    }

    if (auto* acker =
            dynamic_cast<AckermannMotionModel*>(data.motion_model.get())) {
        auto& vx = data.state.vx;
        auto& wz = data.state.wz;
        const float min_turning_rad = acker->getMinTurningRadius();
        auto out_of_turning_rad_motion =
            (min_turning_rad - (vx.abs() / wz.abs())).max(0.0f);
        if (power_ > 1) {
            data.costs +=
                ((((vx - max_vel_).max(0.0f) + (min_vel_ - vx).max(0.0f) +
                   out_of_turning_rad_motion) *
                  data.model_dt)
                     .rowwise()
                     .sum()
                     .eval() *
                 weight_)
                    .pow(power_)
                    .eval();
        } else {
            data.costs +=
                ((((vx - max_vel_).max(0.0f) + (min_vel_ - vx).max(0.0f) +
                   out_of_turning_rad_motion) *
                  data.model_dt)
                     .rowwise()
                     .sum()
                     .eval() *
                 weight_)
                    .eval();
        }
    }
}

}  // namespace critics
}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
