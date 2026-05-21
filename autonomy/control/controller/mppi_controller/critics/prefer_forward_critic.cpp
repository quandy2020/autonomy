#include "autonomy/control/controller/mppi_controller/critics/prefer_forward_critic.hpp"

#include "autonomy/control/controller/mppi_controller/tools/utils.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {
namespace critics {

void PreferForwardCritic::Configure(
    const proto::MPPIControllerOptions& options,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper>) {
    const auto& cfg = options.prefer_forward_critic();
    enabled_ = cfg.enabled();
    power_ = cfg.cost_power() > 0 ? cfg.cost_power() : 1;
    weight_ = static_cast<float>(cfg.cost_weight() > 0.0 ? cfg.cost_weight() : 5.0);
    threshold_ = static_cast<float>(cfg.threshold_to_consider() > 0.0
                                         ? cfg.threshold_to_consider()
                                         : 0.5);
}

void PreferForwardCritic::score(CriticData& data) {
    if (!enabled_) {
        return;
    }
    const auto goal = utils::getCriticGoal(data, enforce_path_inversion_);
    if (utils::withinPositionGoalTolerance(threshold_, data.state.pose.pose,
                                           goal)) {
        return;
    }
    if (power_ > 1) {
        data.costs +=
            ((data.state.vx.unaryExpr([](float x) { return std::max(-x, 0.0f); }) *
              data.model_dt)
                 .rowwise()
                 .sum() *
             weight_)
                .pow(power_);
    } else {
        data.costs +=
            (data.state.vx.unaryExpr([](float x) { return std::max(-x, 0.0f); }) *
             data.model_dt)
                .rowwise()
                .sum() *
            weight_;
    }
}

}  // namespace critics
}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
