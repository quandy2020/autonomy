/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/perception/exploration/core/explorer.hpp"

#include "autonomy/perception/exploration/far3d/far3d_explorer.hpp"
#include "autonomy/perception/exploration/far2d/far_explorer.hpp"
#include "autonomy/perception/exploration/tare/tare_explorer.hpp"

namespace autonomy::perception::exploration {

std::unique_ptr<ExplorerInterface> ExplorerFactory::Create(
    const std::string& backend, const proto::ExplorationOptions& options) {
  if (backend == "far3d" || backend == "rgbd_far3d") {
    return std::make_unique<far3d::Far3dExplorer>(options);
  }
  if (backend == "far" || backend == "rgbd_far") {
    return std::make_unique<FarExplorer>(options);
  }
  if (backend == "lidar_tare") {
    proto::ExplorationOptions lidar_options = options;
    lidar_options.set_use_lidar_primary(true);
    return std::make_unique<TareExplorer>(lidar_options);
  }
  return std::make_unique<TareExplorer>(options);
}

}  // namespace autonomy::perception::exploration
