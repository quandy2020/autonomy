/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#pragma once

#include "autonomy/control/controller/mppi_controller/critic_function.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {
namespace critics {

class ConstraintCritic : public CriticFunction
{
public:
    void Configure(const proto::MPPIControllerOptions& options,
                   std::shared_ptr<map::costmap_2d::Costmap2DWrapper>
                       costmap_wrapper) override;
    void score(CriticData& data) override;
    std::string name() const override { return "ConstraintCritic"; }

private:
    int power_{1};
    float weight_{4.0f};
    float max_vel_{0.0f};
    float min_vel_{0.0f};
};

}  // namespace critics
}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
