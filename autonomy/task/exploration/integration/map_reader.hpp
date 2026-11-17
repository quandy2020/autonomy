/* Copyright 2026 The Openbot Authors. */
#pragma once
#include "autonomy/perception/data/map_reader.hpp"

namespace autonomy::task::exploration {
// Task integration names refer to the shared, consumer-owned geometry contract.
// Planning depends on perception/data, never on task implementations or map writes.
using Pose2d = opt::navigation::Pose2d;
using Bounds2d = perception::Bounds2d;
using MapReadView = perception::MapReadView;
using MapReader = perception::MapReader;
}  // namespace autonomy::task::exploration
