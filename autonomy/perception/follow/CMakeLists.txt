# Copyright 2025 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Follow is a loadable autolink component. It consumes base tracks + depth and
# publishes target / path / grid for TrackingClient + MPPI GridObstaclesCritic.
include(autonomy_module)
autonomy_component(follow_component
  SOURCES
    follow_component.cpp
    options.cpp
    localizer.cpp
    grid.cpp
    planner.cpp)
