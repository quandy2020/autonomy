#include "autonomy/control/controller/mppi_controller/critics/path_follow_critic.hpp"

#include "autonomy/control/controller/mppi_controller/tools/utils.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {
namespace critics {

void PathFollowCritic::Configure(
    const proto::MPPIControllerOptions& options,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
    costmap_wrapper_ = std::move(costmap_wrapper);
    const auto& cfg = options.path_follow_critic();
    enabled_ = cfg.enabled();
    power_ = cfg.cost_power() > 0 ? cfg.cost_power() : 1;
    weight_ = static_cast<float>(cfg.cost_weight() > 0.0 ? cfg.cost_weight() : 5.0);
    threshold_ = static_cast<float>(cfg.threshold_to_consider() > 0.0
                                         ? cfg.threshold_to_consider()
                                         : 1.4);
    offset_ = cfg.offset_from_furthest() > 0
                  ? static_cast<size_t>(cfg.offset_from_furthest())
                  : 5u;
}

void PathFollowCritic::score(CriticData& data) {
    if (!enabled_) {
        return;
    }
    const auto goal = utils::getCriticGoal(data, enforce_path_inversion_);
    if (data.path.x.size() < 2 ||
        utils::withinPositionGoalTolerance(threshold_, data.state.pose.pose,
                                           goal)) {
        return;
    }
    utils::setPathFurthestPointIfNotSet(data);
    utils::setPathCostsIfNotSet(data, costmap_wrapper_);
    const size_t path_size = static_cast<size_t>(data.path.x.size()) - 1;
    auto offsetted_idx =
        std::min(*data.furthest_reached_path_point + offset_, path_size);

    bool valid = false;
    while (!valid && offsetted_idx < path_size - 1) {
        valid = (*data.path_pts_valid)[offsetted_idx];
        if (!valid) {
            ++offsetted_idx;
        }
    }

    const auto path_x = data.path.x(static_cast<Eigen::Index>(offsetted_idx));
    const auto path_y = data.path.y(static_cast<Eigen::Index>(offsetted_idx));
    const int rightmost_idx = data.trajectories.x.cols() - 1;
    const auto last_x = data.trajectories.x.col(rightmost_idx);
    const auto last_y = data.trajectories.y.col(rightmost_idx);
    const auto delta_x = last_x - path_x;
    const auto delta_y = last_y - path_y;
    if (power_ > 1) {
        data.costs +=
            (((delta_x.square() + delta_y.square()).sqrt()) * weight_).pow(power_);
    } else {
        data.costs += ((delta_x.square() + delta_y.square()).sqrt()) * weight_;
    }
}

}  // namespace critics
}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
