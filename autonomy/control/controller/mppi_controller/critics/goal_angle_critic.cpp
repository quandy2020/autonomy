#include "autonomy/control/controller/mppi_controller/critics/goal_angle_critic.hpp"

#include "autonomy/control/controller/mppi_controller/tools/utils.hpp"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {
namespace critics {

void GoalAngleCritic::Configure(const proto::MPPIControllerOptions& options,
                                std::shared_ptr<map::costmap_2d::Costmap2DWrapper>) {
    const auto& cfg = options.goal_angle_critic();
    enabled_ = cfg.enabled();
    power_ = cfg.cost_power() > 0 ? cfg.cost_power() : 1;
    weight_ = static_cast<float>(cfg.cost_weight() > 0.0 ? cfg.cost_weight() : 3.0);
    threshold_ = static_cast<float>(cfg.threshold_to_consider() > 0.0
                                         ? cfg.threshold_to_consider()
                                         : 0.5);
}

void GoalAngleCritic::score(CriticData& data) {
    if (!enabled_) {
        return;
    }
    const auto goal = utils::getCriticGoal(data, enforce_path_inversion_);
    if (!utils::withinPositionGoalTolerance(threshold_, data.state.pose.pose,
                                            goal)) {
        return;
    }
    const float goal_yaw =
        static_cast<float>(transform::tf2::getYaw(goal.orientation));
    if (power_ > 1) {
        data.costs +=
            (((utils::shortest_angular_distance(data.trajectories.yaws, goal_yaw)
                   .abs())
                  .rowwise()
                  .mean()) *
             weight_)
                .pow(power_)
                .eval();
    } else {
        data.costs +=
            (((utils::shortest_angular_distance(data.trajectories.yaws, goal_yaw)
                   .abs())
                  .rowwise()
                  .mean()) *
             weight_)
                .eval();
    }
}

}  // namespace critics
}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
