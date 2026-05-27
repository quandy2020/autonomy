-- Copyright 2025 The Openbot Authors(duyongquan)
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "common.lua"
include "bridge/bridge.lua"
include "control/controller.lua"
include "map/map.lua"
include "prediction/prediction.lua"
include "perception/perception.lua"
include "localization/localization.lua"
include "planner/planner.lua"
include "transform/transform.lua"
include "tasks/tasks.lua"

-- Root table consumed by system::CreateOptions (see system/options.cpp).
AUTONOMY = {
  map = AUTONOMY_MAP,
  planning = AUTONOMY_PLANNER,
  controller = AUTONOMY_CONTROLLER,
  tasks = tasks,
  transform = AUTONOMY_TRANSFORM,
}

return AUTONOMY
