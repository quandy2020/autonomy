#include "autonomy/control/controller/mppi_controller/critics/path_angle_critic.hpp"

#include "autonomy/control/controller/mppi_controller/tools/utils.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {
namespace critics {

void PathAngleCritic::Configure(const proto::MPPIControllerOptions& options,
                                std::shared_ptr<map::costmap_2d::Costmap2DWrapper>) {
    const auto& cfg = options.path_angle_critic();
    enabled_ = cfg.enabled();
    power_ = cfg.cost_power() > 0 ? cfg.cost_power() : 1;
    weight_ = static_cast<float>(cfg.cost_weight() > 0.0 ? cfg.cost_weight() : 2.0);
    threshold_ = static_cast<float>(cfg.threshold_to_consider() > 0.0
                                         ? cfg.threshold_to_consider()
                                         : 0.5);
    max_angle_ = static_cast<float>(cfg.max_angle_to_furthest() > 0.0
                                        ? cfg.max_angle_to_furthest()
                                        : 1.0);
    offset_ = cfg.offset_from_furthest() > 0
                  ? static_cast<size_t>(cfg.offset_from_furthest())
                  : 4u;
    forward_preference_ = cfg.forward_preference();
    if (!forward_preference_ &&
        std::fabs(static_cast<float>(options.vx_min())) < 1e-6f) {
        forward_preference_ = true;
    }
}

void PathAngleCritic::score(CriticData& data) {
    if (!enabled_) {
        return;
    }
    const auto goal = utils::getCriticGoal(data, enforce_path_inversion_);
    if (utils::withinPositionGoalTolerance(threshold_, data.state.pose.pose,
                                           goal)) {
        return;
    }
    utils::setPathFurthestPointIfNotSet(data);
    const auto offsetted_idx = std::min(
        *data.furthest_reached_path_point + offset_,
        static_cast<size_t>(data.path.x.size()) - 1);

    const float goal_x =
        data.path.x(static_cast<Eigen::Index>(offsetted_idx));
    const float goal_y =
        data.path.y(static_cast<Eigen::Index>(offsetted_idx));
    const auto& pose = data.state.pose.pose;

    if (utils::posePointAngle(pose, goal_x, goal_y, forward_preference_) <
        max_angle_) {
        return;
    }

    const int last_idx = data.trajectories.y.cols() - 1;
    auto diff_y = goal_y - data.trajectories.y.col(last_idx);
    auto diff_x = goal_x - data.trajectories.x.col(last_idx);
    auto yaws_between_points =
        diff_y.binaryExpr(diff_x, [](const float y, const float x) {
            return std::atan2(y, x);
        }).eval();

    auto last_yaws = data.trajectories.yaws.col(last_idx);
    Eigen::ArrayXf corrected_yaws;
    if (!forward_preference_) {
        corrected_yaws =
            utils::normalize_yaws_between_points(last_yaws, yaws_between_points);
    } else {
        corrected_yaws = yaws_between_points;
    }
    auto yaws = utils::shortest_angular_distance(last_yaws, corrected_yaws).abs();
    if (power_ > 1) {
        data.costs += (yaws * weight_).pow(power_);
    } else {
        data.costs += yaws * weight_;
    }
}

}  // namespace critics
}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
