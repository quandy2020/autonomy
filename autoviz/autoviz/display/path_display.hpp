/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file path_display.hpp
 * @brief Channel display for @c nav_msgs/Path polylines and pose markers.
 *
 * Renders one or more buffered paths as Lines or Billboards, with optional
 * Axes / Arrows at each pose — aligned with RViz2 PathDisplay.
 *
 * ## Properties
 *
 * - @c line_style — @c Lines or @c Billboards
 * - @c line_width / @c color / @c alpha — polyline appearance
 * - @c buffer_length — number of historical path snapshots kept
 * - @c offset — XYZ offset applied in the fixed frame
 * - @c pose_style — @c None, @c Axes, or @c Arrows at each pose
 * - pose axes / arrow size and color properties
 *
 * @see ChannelDisplay
 * @see drawBillboardStripOgreOrGl()
 * @see PoseArrayDisplay
 */

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

/**
 * @class PathDisplay
 * @brief Subscribes to @c nav_msgs/Path and draws buffered trajectories.
 *
 * ## Data flow
 *
 * - **In:** @ref processMessage() TF-transforms each pose into the fixed
 *   frame and appends a @ref PathSnapshot into a ring of @c path_buffers_.
 * - **Out:** @ref onDraw() draws polylines (@ref drawPathPolyline) and
 *   optional pose markers (@ref drawPoseMarkers).
 */
class PathDisplay
    : public ChannelDisplay<automsgs::msgs::nav_msgs::Path> {
 public:
  /**
   * @brief Constructs a Path display bound to @p channel.
   *
   * @param channel Autolink / topic name for @c nav_msgs/Path.
   */
  explicit PathDisplay(std::string channel);

  /**
   * @brief Catalog type id used by the display registry.
   * @return Always @c "Path".
   */
  std::string typeId() const override { return "Path"; }

  /**
   * @brief Declares line style, buffer, offset, and pose-marker properties.
   * @return Property specification list for the Displays tree.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Transforms and buffers a new path message.
   *
   * @param message Incoming @c nav_msgs/Path.
   */
  void processMessage(
      const automsgs::msgs::nav_msgs::Path& message) override;

  /**
   * @brief Draws all buffered path snapshots into @p scene.
   *
   * @param scene Scene overlay / Ogre host for the current frame.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

  /**
   * @brief Clears path buffers (Time-panel Reset).
   */
  void clearReceivedData() override;

 private:
  /**
   * @struct PathPose
   * @brief One path waypoint in the fixed frame.
   */
  struct PathPose {
    QVector3D position;       /**< Waypoint position. */
    QQuaternion orientation;  /**< Waypoint orientation. */
  };

  /**
   * @struct PathSnapshot
   * @brief One complete path message after TF transform.
   */
  struct PathSnapshot {
    std::vector<PathPose> poses; /**< Ordered poses along the path. */
  };

  /**
   * @brief Resizes @c path_buffers_ to @p buffer_length (keeps newest).
   *
   * @param buffer_length Desired history length (≥ 1).
   */
  void resizeBuffers(std::size_t buffer_length);

  /**
   * @brief Whether the current Line Style property selects Billboards.
   * @return @c true when billboard strip drawing should be used.
   */
  bool useBillboardLineStyle() const;

  /**
   * @brief Draws a single path polyline (Lines or Billboards).
   *
   * @param scene Target overlay.
   * @param suffix Unique suffix for Ogre object naming within the display.
   * @param poses Path poses to connect.
   * @param color Polyline color.
   * @param line_width Width for billboard style (meters).
   */
  void drawPathPolyline(rendering::SceneOverlay& scene,
                        const std::string& suffix,
                        const std::vector<PathPose>& poses,
                        const QColor& color, float line_width) const;

  /**
   * @brief Draws Axes or Arrows at each pose according to Pose Style.
   *
   * @param scene Target overlay.
   * @param suffix Unique suffix for Ogre object naming.
   * @param poses Path poses to annotate.
   */
  void drawPoseMarkers(rendering::SceneOverlay& scene,
                       const std::string& suffix,
                       const std::vector<PathPose>& poses) const;

  std::vector<PathSnapshot> path_buffers_; /**< Ring of historical paths. */
  std::size_t messages_received_ = 0;      /**< Count of accepted messages. */
};

}  // namespace display
}  // namespace autoviz
