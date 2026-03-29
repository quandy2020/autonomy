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

#include "autonomy/tools/aviz/plugins/displays/polygon/polygon_display.hpp"

#include "autonomy/tools/aviz/common/properties/status_property.hpp"

namespace aviz {
namespace plugins {
namespace displays {

PolygonDisplay::PolygonDisplay(const QString& name) : aviz::common::Display() {
    setClassId("aviz/Polygon");
    setName(name);
}

PolygonDisplay::~PolygonDisplay() = default;

void PolygonDisplay::onInitialize() {
    aviz::common::Display::onInitialize();
}

void PolygonDisplay::onEnable() {
    aviz::common::Display::onEnable();
}

void PolygonDisplay::onDisable() {
    aviz::common::Display::onDisable();
}

void PolygonDisplay::update(float wall_dt, float ros_dt) {
    (void)wall_dt;
    (void)ros_dt;
}

void PolygonDisplay::reset() {
    aviz::common::Display::reset();
}

}  // namespace displays
}  // namespace plugins
}  // namespace aviz
