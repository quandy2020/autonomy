-- Copyright 2025 The Openbot Authors(duyongquan)
--
-- Full autonomy config with exploration perception enabled.
-- Use as --configuration_basename=exploration_autonomy.lua for closed-loop stacks.

include "common.lua"
include "bridge/bridge.lua"
include "control/controller.lua"
include "map/map.lua"
include "prediction/prediction.lua"
include "perception/perception_exploration.lua"
include "localization/localization.lua"
include "planner/planner.lua"
include "transform/transform.lua"
include "navigator/navigator.lua"

AUTONOMY = {
  map = AUTONOMY_MAP,
  planning = AUTONOMY_PLANNER,
  controller = AUTONOMY_CONTROLLER,
  navigator = navigator,
  transform = AUTONOMY_TRANSFORM,
  perception = AUTONOMY_PERCEPTION,
}

return AUTONOMY
