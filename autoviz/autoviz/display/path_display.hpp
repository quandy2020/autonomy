/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QQuaternion>
#include <QVector3D>
#include <vector>

#include <automsgs/msgs/nav_msgs/path.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace display {

class PathDisplay
    : public ChannelDisplay<automsgs::msgs::nav_msgs::Path> {
 public:
  explicit PathDisplay(std::string channel);

  std::string typeId() const override { return "Path"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::nav_msgs::Path& message) override;
  void onDraw(rendering::SceneOverlay& scene) override;
  void clearReceivedData() override;

 private:
  struct PathPose {
    QVector3D position;
    QQuaternion orientation;
  };

  struct PathSnapshot {
    std::vector<PathPose> poses;
  };

  void resizeBuffers(std::size_t buffer_length);
  bool useBillboardLineStyle() const;
  void drawPathPolyline(rendering::SceneOverlay& scene,
                        const std::string& suffix,
                        const std::vector<PathPose>& poses,
                        const QColor& color, float line_width) const;
  void drawPoseMarkers(rendering::SceneOverlay& scene,
                       const std::string& suffix,
                       const std::vector<PathPose>& poses) const;

  std::vector<PathSnapshot> path_buffers_;
  std::size_t messages_received_ = 0;
};

}  // namespace display
}  // namespace autoviz
