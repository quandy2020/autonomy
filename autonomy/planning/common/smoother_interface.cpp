/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/planning/common/smoother_interface.hpp"

#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"

namespace autonomy {
namespace planning {
namespace common {

Smoother::Smoother(
    std::string name,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper)
    : name_(std::move(name)), costmap_wrapper_(std::move(costmap_wrapper)) {}

}  // namespace common
}  // namespace planning
}  // namespace autonomy
