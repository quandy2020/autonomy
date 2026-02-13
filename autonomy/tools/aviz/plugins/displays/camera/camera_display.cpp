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

#include "autonomy/tools/aviz/plugins/displays/camera/camera_display.hpp"

#include "autonomy/tools/aviz/common/properties/status_property.hpp"

namespace aviz {
namespace plugins {
namespace displays {

CameraDisplay::CameraDisplay(const QString& name)
    : AutolinkTopicDisplay<autonomy::commsgs::sensor_msgs::Image>(QString("aviz/Camera")) {
    setName(name);
}

CameraDisplay::~CameraDisplay() = default;

void CameraDisplay::onInitialize() {
    AutolinkTopicDisplay::onInitialize();
}

void CameraDisplay::reset() {
    AutolinkTopicDisplay::reset();
}

void CameraDisplay::processMessage(const std::shared_ptr<autonomy::commsgs::sensor_msgs::Image>& msg) {
    if (!msg) {
        return;
    }

    // TODO: Implement camera image rendering with camera info
    setTransformOk();
    queueRender();
}

}  // namespace displays
}  // namespace plugins
}  // namespace aviz
