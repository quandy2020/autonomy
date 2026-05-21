#pragma once

#include "autonomy/control/controller/mppi_controller/critic_function.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {
namespace critics {

class PreferForwardCritic : public CriticFunction
{
public:
    void Configure(const proto::MPPIControllerOptions& options,
                   std::shared_ptr<map::costmap_2d::Costmap2DWrapper>
                       costmap_wrapper) override;
    void score(CriticData& data) override;
    std::string name() const override { return "PreferForwardCritic"; }

private:
    int power_{1};
    float weight_{5.0f};
    float threshold_{0.5f};
    bool enforce_path_inversion_{false};
};

}  // namespace critics
}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
