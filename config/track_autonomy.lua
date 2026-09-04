-- Copyright 2026 The Openbot Authors
--
-- Autonomy stack config for YOPO human-following with autosim.

include "common.lua"
include "bridge/bridge.lua"
include "control/controller.lua"
include "map/map.lua"
include "prediction/prediction.lua"
include "perception/perception_track.lua"
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
