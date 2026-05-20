-- Copyright 2025 The Openbot Authors(duyongquan)
--
-- Deprecated: TaskScheduler loads config/tasks/tasks.lua directly.
-- This file remains for launchers that expect config/task_options.lua.

include "tasks/tasks.lua"

return { tasks = tasks }
