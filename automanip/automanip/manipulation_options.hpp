/*
 * Copyright 2026 The Openbot Authors
 *
 * Thin options facade (planning planner_options analogue).
 */

#pragma once

#include "automanip/proto/manipulation_options.pb.h"

namespace automanip {

/** @brief Alias for proto ManipulationOptions (conf / Init input). */
using ManipulationOptions = proto::ManipulationOptions;

}  // namespace automanip
