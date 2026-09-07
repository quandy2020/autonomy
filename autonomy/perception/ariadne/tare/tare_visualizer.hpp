/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <string>

#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>

#include "autonomy/perception/ariadne/tare/grid_world.hpp"
#include "autonomy/perception/ariadne/tare/keypose_graph.hpp"

namespace autonomy::perception::exploration {

automsgs::msgs::visualization_msgs::MarkerArray BuildTareGraphMarkers(
    const GridWorld& grid_world, const KeyposeGraph& keypose_graph,
    const std::string& frame_id);

}  // namespace autonomy::perception::exploration
