/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/common/controller_interface.hpp"

#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace control {
namespace common {

ControllerInterface::ControllerInterface(
    const proto::ControllerOptions& options, std::string name,
    std::shared_ptr<TfBuffer> tf_buffer,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper)
    : options_(options),
      name_(std::move(name)),
      tf_buffer_(std::move(tf_buffer)),
      costmap_wrapper_(std::move(costmap_wrapper)) {}

}  // namespace common
}  // namespace control
}  // namespace autonomy
