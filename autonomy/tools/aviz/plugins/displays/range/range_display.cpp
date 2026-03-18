/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "autonomy/tools/aviz/plugins/displays/range/range_display.hpp"

#include "autonomy/tools/aviz/common/properties/status_property.hpp"

namespace aviz {
namespace plugins {
namespace displays {

RangeDisplay::RangeDisplay(const QString& name) : aviz::common::Display() {
  setClassId("aviz/Range");
  setName(name);
}

RangeDisplay::~RangeDisplay() = default;

void RangeDisplay::onInitialize() { aviz::common::Display::onInitialize(); }

void RangeDisplay::onEnable() { aviz::common::Display::onEnable(); }

void RangeDisplay::onDisable() { aviz::common::Display::onDisable(); }

void RangeDisplay::update(float wall_dt, float ros_dt) {
  (void)wall_dt;
  (void)ros_dt;
}

void RangeDisplay::reset() { aviz::common::Display::reset(); }

}  // namespace displays
}  // namespace plugins
}  // namespace aviz
