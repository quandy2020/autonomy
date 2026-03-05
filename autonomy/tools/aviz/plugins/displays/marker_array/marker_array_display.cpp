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

#include "autonomy/tools/aviz/plugins/displays/marker_array/marker_array_display.hpp"

#include "autonomy/tools/aviz/common/properties/status_property.hpp"
#include "autonomy/tools/aviz/plugins/displays/marker/marker_display.hpp"

namespace aviz {
namespace plugins {
namespace displays {

MarkerArrayDisplay::MarkerArrayDisplay(const QString& name)
    : AutolinkTopicDisplay<autonomy::commsgs::visualization_msgs::MarkerArray>(QString("aviz/MarkerArray")),
      marker_display_(nullptr) {
  setName(name);
}

MarkerArrayDisplay::~MarkerArrayDisplay() { delete marker_display_; }

void MarkerArrayDisplay::onInitialize() {
  AutolinkTopicDisplay::onInitialize();
  if (!marker_display_) {
    marker_display_ = new MarkerDisplay("InternalMarkerDisplay");
    marker_display_->initialize(context_);
  }
}

void MarkerArrayDisplay::reset() {
  AutolinkTopicDisplay::reset();
  if (marker_display_) {
    marker_display_->reset();
  }
}

void MarkerArrayDisplay::processMessage(
    const std::shared_ptr<autonomy::commsgs::visualization_msgs::MarkerArray>& msg) {
  if (!msg || !marker_display_) {
    return;
  }

  // Process each marker in the array
  for (const auto& marker : msg->markers) {
    auto marker_ptr = std::make_shared<autonomy::commsgs::visualization_msgs::Marker>(marker);
    marker_display_->processMarkerMessage(marker_ptr);
  }

  setTransformOk();
  queueRender();
}

}  // namespace displays
}  // namespace plugins
}  // namespace aviz
