#pragma once

#include "autonomy/control/controller/mppi_controller/critic_function.hpp"
#include "autonomy/map/costmap_2d/footprint_collision_checker.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {
namespace critics {

class CostCritic : public CriticFunction
{
public:
    void Configure(const proto::MPPIControllerOptions& options,
                   std::shared_ptr<map::costmap_2d::Costmap2DWrapper>
                       costmap_wrapper) override;
    void score(CriticData& data) override;
    std::string name() const override { return "CostCritic"; }

private:
    float findCircumscribedCost();

    int power_{1};
    float weight_{3.81f};
    float critical_cost_{300.0f};
    float collision_cost_{1e6f};
    float near_goal_distance_{1.0f};
    int trajectory_point_step_{2};
    bool consider_footprint_{true};
    bool enforce_path_inversion_{false};

    float possible_collision_cost_{253.0f};
    float circumscribed_radius_{0.0f};
    float circumscribed_cost_{0.0f};

    float origin_x_{0.0f};
    float origin_y_{0.0f};
    float resolution_{0.05f};
    unsigned int size_x_{0};
    unsigned int size_y_{0};

    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
    map::costmap_2d::FootprintCollisionChecker<map::costmap_2d::Costmap2D*>
        collision_checker_;
};

}  // namespace critics
}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
