#include "autonomy/control/controller/mppi_controller/critics/cost_critic.hpp"

#include "autonomy/control/controller/mppi_controller/tools/utils.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/layers/inflation_layer.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {
namespace critics {

void CostCritic::Configure(
    const proto::MPPIControllerOptions& options,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
    costmap_wrapper_ = std::move(costmap_wrapper);
    const auto& cfg = options.cost_critic();
    enabled_ = cfg.enabled();
    power_ = cfg.cost_power() > 0 ? cfg.cost_power() : 1;
    weight_ = static_cast<float>(cfg.cost_weight() > 0.0 ? cfg.cost_weight() : 3.81);
    weight_ /= 254.0f;
    critical_cost_ =
        static_cast<float>(cfg.critical_cost() > 0.0 ? cfg.critical_cost() : 300.0);
    collision_cost_ = static_cast<float>(cfg.collision_cost() > 0.0
                                             ? cfg.collision_cost()
                                             : 1e6);
    near_goal_distance_ = static_cast<float>(cfg.near_goal_distance() > 0.0
                                                 ? cfg.near_goal_distance()
                                                 : 1.0);
    trajectory_point_step_ = cfg.trajectory_point_step() > 0
                                 ? cfg.trajectory_point_step()
                                 : 2;
    consider_footprint_ = cfg.consider_footprint();

    collision_checker_.setCostmap(costmap_wrapper_->getCostmap());
    possible_collision_cost_ = findCircumscribedCost();
}

float CostCritic::findCircumscribedCost() {
    const double circum_radius =
        costmap_wrapper_->getLayeredCostmap()->getCircumscribedRadius();
    if (static_cast<float>(circum_radius) == circumscribed_radius_) {
        return circumscribed_cost_;
    }

    float result =
        static_cast<float>(map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    auto inflation_layer =
        map::costmap_2d::InflationLayer::getInflationLayer(costmap_wrapper_);
    if (inflation_layer) {
        const double resolution = costmap_wrapper_->getCostmap()->getResolution();
        const double inflation_radius = inflation_layer->getInflationRadius();
        if (inflation_radius < circum_radius) {
            result = 0.0f;
        } else {
            result = static_cast<float>(
                inflation_layer->computeCost(circum_radius / resolution));
        }
    }

    circumscribed_radius_ = static_cast<float>(circum_radius);
    circumscribed_cost_ = result;
    return result;
}

void CostCritic::score(CriticData& data) {
    if (!enabled_) {
        return;
    }

    const auto goal = utils::getCriticGoal(data, enforce_path_inversion_);
    auto* costmap = collision_checker_.getCostmap();
    origin_x_ = static_cast<float>(costmap->getOriginX());
    origin_y_ = static_cast<float>(costmap->getOriginY());
    resolution_ = static_cast<float>(costmap->getResolution());
    size_x_ = costmap->getSizeInCellsX();
    size_y_ = costmap->getSizeInCellsY();

    if (consider_footprint_) {
        possible_collision_cost_ = findCircumscribedCost();
    }

    const bool near_goal = utils::withinPositionGoalTolerance(
        near_goal_distance_, data.state.pose.pose, goal);

    Eigen::ArrayXf repulsive_cost(data.costs.rows());
    repulsive_cost.setZero();
    bool all_trajectories_collide = true;

    const int strided_traj_cols =
        static_cast<int>(std::floor((data.trajectories.x.cols() - 1) /
                                    trajectory_point_step_)) +
        1;
    const int strided_traj_rows = data.trajectories.x.rows();
    const int outer_stride = strided_traj_rows * trajectory_point_step_;

    const auto traj_x = Eigen::Map<const Eigen::ArrayXXf, 0, Eigen::Stride<-1, -1>>(
        data.trajectories.x.data(), strided_traj_rows, strided_traj_cols,
        Eigen::Stride<-1, -1>(outer_stride, 1));
    const auto traj_y = Eigen::Map<const Eigen::ArrayXXf, 0, Eigen::Stride<-1, -1>>(
        data.trajectories.y.data(), strided_traj_rows, strided_traj_cols,
        Eigen::Stride<-1, -1>(outer_stride, 1));
    const auto traj_yaw = Eigen::Map<const Eigen::ArrayXXf, 0, Eigen::Stride<-1, -1>>(
        data.trajectories.yaws.data(), strided_traj_rows, strided_traj_cols,
        Eigen::Stride<-1, -1>(outer_stride, 1));

    const auto footprint = costmap_wrapper_->getRobotFootprint();
    const bool use_radius = costmap_wrapper_->getUseRadius();

    for (int i = 0; i < strided_traj_rows; ++i) {
        bool trajectory_collide = false;
        float& traj_cost = repulsive_cost(i);

        for (int j = 0; j < strided_traj_cols; ++j) {
            const float Tx = traj_x(i, j);
            const float Ty = traj_y(i, j);
            const float Tyaw = traj_yaw(i, j);
            unsigned int x_i = 0u;
            unsigned int y_i = 0u;
            float pose_cost = 0.0f;

            if (!costmap->worldToMap(Tx, Ty, x_i, y_i)) {
                pose_cost = 255.0f;
            } else {
                pose_cost = static_cast<float>(costmap->getCost(x_i, y_i));
                if (pose_cost < 1.0f) {
                    continue;
                }
            }

            bool in_collision = false;
            if (pose_cost >=
                static_cast<float>(map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) {
                in_collision = true;
            } else if (consider_footprint_ && !use_radius &&
                       pose_cost >= possible_collision_cost_) {
                const double footprint_cost =
                    collision_checker_.footprintCostAtPose(Tx, Ty, Tyaw, footprint);
                if (footprint_cost < 0.0 ||
                    footprint_cost ==
                        static_cast<double>(
                            map::costmap_2d::LETHAL_OBSTACLE)) {
                    in_collision = true;
                }
            }

            if (in_collision) {
                traj_cost = collision_cost_;
                trajectory_collide = true;
                break;
            }

            if (pose_cost >= possible_collision_cost_) {
                traj_cost += critical_cost_;
            } else if (!near_goal) {
                traj_cost += pose_cost;
            }
        }
        all_trajectories_collide &= trajectory_collide;
    }

    if (power_ > 1) {
        data.costs += (repulsive_cost * (weight_ / static_cast<float>(strided_traj_cols)))
                          .pow(power_);
    } else {
        data.costs +=
            repulsive_cost * (weight_ / static_cast<float>(strided_traj_cols));
    }
    data.fail_flag = all_trajectories_collide;
}

}  // namespace critics
}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
