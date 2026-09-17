/*
 * Copyright 2026 The Openbot Authors
 *
 * Umbrella include for all move_group capability plugins.
 */

#pragma once

#include "autonomy/manipulation/dispatch/capability/capability.hpp"
#include "autonomy/manipulation/dispatch/capability/plan_capability.hpp"
#include "autonomy/manipulation/dispatch/capability/execute_capability.hpp"
#include "autonomy/manipulation/dispatch/capability/plan_and_execute_capability.hpp"
#include "autonomy/manipulation/dispatch/capability/cartesian_capability.hpp"
#include "autonomy/manipulation/dispatch/capability/fk_ik_capability.hpp"
#include "autonomy/manipulation/dispatch/capability/get_planning_scene_capability.hpp"
#include "autonomy/manipulation/dispatch/capability/apply_planning_scene_capability.hpp"
#include "autonomy/manipulation/dispatch/capability/state_validation_capability.hpp"
#include "autonomy/manipulation/dispatch/capability/query_planners_capability.hpp"
#include "autonomy/manipulation/dispatch/capability/clear_octomap_capability.hpp"
#include "autonomy/manipulation/dispatch/capability/clear_scene_capability.hpp"
#include "autonomy/manipulation/dispatch/capability/validate_trajectory_capability.hpp"
#include "autonomy/manipulation/dispatch/capability/get_urdf_capability.hpp"
#include "autonomy/manipulation/dispatch/capability/save_load_geometry_capability.hpp"
#include "autonomy/manipulation/dispatch/capability/get_dynamics_capability.hpp"
