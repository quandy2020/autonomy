/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/plugins/displays/util/rviz_scene_publisher.hpp"

#include <utility>

namespace autoviz {
namespace display {

ScenePublisherDisplay::ScenePublisherDisplay(std::string image_channel)
    : image_channel_(std::move(image_channel)) {
  setProperties({});
}

void ScenePublisherDisplay::setChannel(const std::string& channel) {
  image_channel_ = channel;
  refreshStatus();
}

std::vector<common::DisplayPropertySpec>
ScenePublisherDisplay::propertySpecs() const {
  return {{"image_channel", "Image Channel", ""},
          {"publish_rate", "Publish Rate (Hz)", "1.0"},
          {"enabled", "Publish Enabled", "false"}};
}

void ScenePublisherDisplay::onEnable() {
  Display::onEnable();
  if (image_channel_.empty()) {
    image_channel_ = propertyValue("image_channel", "");
  }
  refreshStatus();
}

void ScenePublisherDisplay::onDisable() {
  Display::onDisable();
  setStatusOk("Scene publisher disabled");
}

void ScenePublisherDisplay::onUpdate() {
  Display::onUpdate();
  if (image_channel_.empty()) {
    image_channel_ = propertyValue("image_channel", "");
  }
  refreshStatus();
}

void ScenePublisherDisplay::refreshStatus() {
  const bool publish =
      enabled() && propertyValue("enabled", "false") == "true";
  if (!publish) {
    setStatusOk("Publishing disabled");
    return;
  }
  if (image_channel_.empty()) {
    setStatusWarn("Set Image Channel to publish viewport snapshots");
    return;
  }
  setStatusWarn("Scene snapshot publishing is not implemented yet (channel=" +
                image_channel_ + ")");
}

void ScenePublisherDisplay::onDraw(rendering::SceneOverlay& /*scene*/) {}

}  // namespace display
}  // namespace autoviz
