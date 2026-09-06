/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <string>

#include "autonomy/perception/proto/exploration_options.pb.h"

namespace autonomy::perception::exploration {

proto::ExplorationOptions DefaultOptions();
proto::ExplorationOptions LoadOptions(
    const std::string& config_directory,
    const std::string& relative_path =
        "perception/exploration_rgbd_tare.lua");

}  // namespace autonomy::perception::exploration
