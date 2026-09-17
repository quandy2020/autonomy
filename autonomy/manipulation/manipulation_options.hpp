/*
 * Copyright 2026 The Openbot Authors
 *
 * Thin options facade (planning planner_options analogue).
 */

#pragma once

#include "autonomy/manipulation/proto/manipulation_options.pb.h"

namespace autonomy {
namespace manipulation {

/** @brief Alias for proto ManipulationOptions (conf / Init input). */
using ManipulationOptions = proto::ManipulationOptions;

}  // namespace manipulation
}  // namespace autonomy
