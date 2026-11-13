/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file path_display.hpp
 * @brief Channel display for @c nav_msgs/Path polylines and pose markers.
 *
 * Supports Solid / Dashed / FlowDashed patterns — FlowDashed animates dashes
 * and chevrons along the path for a sci-fi guide look.
 */

#pragma once

#include <QQuaternion>
#include <QVector3D>
#include <string>
#include <vector>

#include <automsgs/msgs/nav_msgs/path.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace display {

class PathDisplay : public ChannelDisplay<automsgs::msgs::nav_msgs::Path> {
 public:
  explicit PathDisplay(std::string channel);

  std::string typeId() const override { return "Path"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::nav_msgs::Path& message) override;
  void onUpdate() override;
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
  std::string pathPattern() const;

  void drawPathPolyline(rendering::SceneOverlay& scene,
                        const std::string& suffix,
                        const std::vector<PathPose>& poses,
                        const QColor& color, float line_width) const;

  /** Dashed / flowing dashes sampled along arc length. */
  void drawDashedPath(rendering::SceneOverlay& scene,
                      const std::string& suffix,
                      const std::vector<PathPose>& poses, const QColor& color,
                      float line_width, bool animate) const;

  void drawFlowChevrons(rendering::SceneOverlay& scene,
                        const std::string& suffix,
                        const std::vector<PathPose>& poses,
                        const QColor& color) const;

  void drawPoseMarkers(rendering::SceneOverlay& scene,
                       const std::string& suffix,
                       const std::vector<PathPose>& poses) const;

  /** Clears latched dash / arrow / underlay meshes for one buffer slot. */
  void clearPathGeometry(rendering::SceneOverlay& scene,
                         const std::string& suffix) const;

  /** True when robot TF is within arrive_distance of the path terminal. */
  bool robotArrivedAtPathEnd(const std::vector<PathPose>& poses) const;

  /** Drop poses behind the robot so residual trail does not linger. */
  std::vector<PathPose> prunePosesBehindRobot(
      const std::vector<PathPose>& poses) const;

  bool lookupRobotPosition(QVector3D* out) const;

  std::vector<PathSnapshot> path_buffers_;
  std::size_t messages_received_ = 0;
};

}  // namespace display
}  // namespace autoviz
