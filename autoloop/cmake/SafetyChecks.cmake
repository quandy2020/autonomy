# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.

# Compile-time reminders: data_agent must not publish /cmd_vel;
# cloud PublishGate must not skip version-tuple admission.
add_compile_definitions(AUTOLOOP_LATCH_ONLY=1)
