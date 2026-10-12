/*
 * Copyright 2026 The Openbot Authors
 *
 * Umbrella include for all move_group capability plugins.
 */

#pragma once

#include "automanip/dispatch/capability/capability.hpp"
#include "automanip/dispatch/capability/plan_capability.hpp"
#include "automanip/dispatch/capability/execute_capability.hpp"
#include "automanip/dispatch/capability/plan_and_execute_capability.hpp"
#include "automanip/dispatch/capability/cartesian_capability.hpp"
#include "automanip/dispatch/capability/fk_ik_capability.hpp"
#include "automanip/dispatch/capability/get_planning_scene_capability.hpp"
#include "automanip/dispatch/capability/apply_planning_scene_capability.hpp"
#include "automanip/dispatch/capability/state_validation_capability.hpp"
#include "automanip/dispatch/capability/query_planners_capability.hpp"
#include "automanip/dispatch/capability/clear_octomap_capability.hpp"
#include "automanip/dispatch/capability/clear_scene_capability.hpp"
#include "automanip/dispatch/capability/validate_trajectory_capability.hpp"
#include "automanip/dispatch/capability/get_urdf_capability.hpp"
#include "automanip/dispatch/capability/save_load_geometry_capability.hpp"
#include "automanip/dispatch/capability/get_dynamics_capability.hpp"
