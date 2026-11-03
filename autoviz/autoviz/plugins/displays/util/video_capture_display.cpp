/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Ported from RViz visualization plugins (BSD).
 *****************************************************************************/

#include "autoviz/plugins/displays/util/video_capture_display.hpp"

namespace autoviz {
namespace display {

VideoCaptureDisplay::VideoCaptureDisplay() {
  setProperties({});
}

std::vector<common::DisplayPropertySpec>
VideoCaptureDisplay::propertySpecs() const {
  return {{"enable", "Enable Capture", "false"},
          {"save_path", "Save Path", "/tmp/autoviz_capture.avi"},
          {"fps", "Frames Per Second", "30"}};
}

void VideoCaptureDisplay::onEnable() {
  Display::onEnable();
  onUpdate();
}

void VideoCaptureDisplay::onDisable() {
  Display::onDisable();
  setStatusOk("Capture disabled");
}

void VideoCaptureDisplay::onUpdate() {
  Display::onUpdate();
  const bool enable = propertyValue("enable", "false") == "true";
  if (!enable) {
    setStatusOk("Capture disabled (enable property is false)");
    return;
  }
  const std::string path = propertyValue("save_path", "/tmp/autoviz_capture.avi");
  setStatusWarn("Viewport video capture is not available in Autoviz yet "
                  "(save_path=" +
              path + ")");
}

void VideoCaptureDisplay::onDraw(rendering::SceneOverlay& /*scene*/) {
  // No-op until a viewport grab / encoder API is exposed on DisplayContext.
}

}  // namespace display
}  // namespace autoviz
