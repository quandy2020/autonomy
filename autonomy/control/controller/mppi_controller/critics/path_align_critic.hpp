#pragma once

#include "autonomy/control/controller/mppi_controller/critic_function.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {
namespace critics {

class PathAlignCritic : public CriticFunction
{
public:
    void Configure(const proto::MPPIControllerOptions& options,
                   std::shared_ptr<map::costmap_2d::Costmap2DWrapper>
                       costmap_wrapper) override;
    void score(CriticData& data) override;
    std::string name() const override { return "PathAlignCritic"; }

private:
    int power_{1};
    float weight_{14.0f};
    float threshold_{0.5f};
    float max_path_occupancy_ratio_{0.05f};
    size_t offset_{20};
    int trajectory_point_step_{4};
    bool use_path_orientations_{false};
    bool enforce_path_inversion_{false};
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper_;
};

}  // namespace critics
}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
