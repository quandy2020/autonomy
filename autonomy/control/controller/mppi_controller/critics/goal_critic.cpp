#include "autonomy/control/controller/mppi_controller/critics/goal_critic.hpp"

#include "autonomy/control/controller/mppi_controller/tools/utils.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {
namespace critics {

void GoalCritic::Configure(const proto::MPPIControllerOptions& options,
                           std::shared_ptr<map::costmap_2d::Costmap2DWrapper>) {
    const auto& cfg = options.goal_critic();
    enabled_ = cfg.enabled();
    power_ = cfg.cost_power() > 0 ? cfg.cost_power() : 1;
    weight_ = static_cast<float>(cfg.cost_weight() > 0.0 ? cfg.cost_weight() : 5.0);
    threshold_ = static_cast<float>(cfg.threshold_to_consider() > 0.0
                                         ? cfg.threshold_to_consider()
                                         : 1.4);
    (void)options;
}

void GoalCritic::score(CriticData& data) {
    if (!enabled_) {
        return;
    }
    const auto goal = utils::getCriticGoal(data, enforce_path_inversion_);
    if (!utils::withinPositionGoalTolerance(threshold_, data.state.pose.pose,
                                            goal)) {
        return;
    }
    const auto goal_x = goal.position.x;
    const auto goal_y = goal.position.y;
    const auto delta_x = data.trajectories.x - goal_x;
    const auto delta_y = data.trajectories.y - goal_y;
    if (power_ > 1) {
        data.costs +=
            (((delta_x.square() + delta_y.square()).sqrt()).rowwise().mean() *
             weight_)
                .pow(power_);
    } else {
        data.costs +=
            (((delta_x.square() + delta_y.square()).sqrt()).rowwise().mean() *
             weight_)
                .eval();
    }
}

}  // namespace critics
}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
