/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file effort_display.hpp
 * @brief Visualizes @c sensor_msgs/JointState effort/torque on URDF joint
 *        axes (RViz Effort display).
 *
 * Loads a robot description (URDF path or @c /robot_description channel),
 * listens to joint states, and draws signed effort arrows along each joint
 * axis using @ref appendSolidArrowMeshes.
 *
 * @see UrdfModel
 * @see arrow_mesh_utils.hpp
 * @see RobotModelDisplay
 */

#pragma once

#include <string>
#include <unordered_map>

#include "autoviz/common/display_property.hpp"
#include "autoviz/display/display.hpp"
#include "autoviz/display/proto_payload_utils.hpp"
#include "autoviz/display/urdf_model.hpp"
#include "autoviz/integration/message_queue.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace display {

/**
 * @class EffortDisplay
 * @brief Joint effort arrows overlaid on a URDF kinematic tree.
 *
 * Properties include URDF path / description channel, effort scale, minimum
 * arrow length, threshold, and positive/negative colors.
 *
 * @see UrdfModel
 * @see Display
 */
class EffortDisplay : public Display {
 public:
  /**
   * @brief Constructs the display for @p joint_channel.
   *
   * @param joint_channel Autolink channel for JointState messages.
   */
  explicit EffortDisplay(std::string joint_channel);

  /**
   * @brief Catalog type id.
   *
   * @return Always @c "Effort".
   */
  std::string typeId() const override { return "Effort"; }

  /**
   * @brief JointState channel name.
   *
   * @return @c joint_channel_.
   */
  std::string channel() const override { return joint_channel_; }

  /**
   * @brief Changes the JointState channel and rebinds when enabled.
   *
   * @param channel New joint state channel.
   */
  void setChannel(const std::string& channel) override;

  /**
   * @brief Property schema (URDF, colors, scales, thresholds, …).
   *
   * @return Specs for the Displays property editor.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Subscribes to joint state and optional description channels.
   */
  void onEnable() override;

  /**
   * @brief Unsubscribes and clears queues.
   */
  void onDisable() override;

  /**
   * @brief Clears joint position/effort caches.
   */
  void reset() override;

  /**
   * @brief Drains joint / description queues and updates status.
   */
  void onUpdate() override;

  /**
   * @brief Reloads URDF when path/description properties change.
   *
   * @param key Changed property key.
   */
  void onPropertyChanged(const std::string& key) override;

  /**
   * @brief Draws effort arrows for joints above the threshold.
   *
   * @param scene Scene overlay destination.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  /**
   * @brief Reloads @c model_ from @c urdf_path or description channel data.
   */
  void reloadUrdf();

  /**
   * @brief Updates @c joint_positions_ / @c joint_efforts_ from a parsed
   *        JointState.
   *
   * @param message Parsed joint state wire struct.
   */
  void processJointState(const proto_wire::ParsedJointState& message);

  /**
   * @brief Appends a signed effort arrow along @p axis at @p origin.
   *
   * @param scene Overlay destination.
   * @param origin Joint origin in fixed frame.
   * @param axis Joint axis direction (unit-ish).
   * @param effort Signed effort/torque value.
   * @param positive_color Color for effort ≥ 0.
   * @param negative_color Color for effort < 0.
   * @param min_length Minimum arrow length in meters.
   * @param scale Meters per unit effort.
   */
  void drawEffortArrow(rendering::SceneOverlay& scene, const QVector3D& origin,
                       const QVector3D& axis, double effort,
                       const QColor& positive_color,
                       const QColor& negative_color, float min_length,
                       float scale) const;

  /** JointState Autolink channel. */
  std::string joint_channel_;

  /** Parsed URDF kinematic / visual model. */
  UrdfModel model_;

  /** Latest joint name → position (rad or m). */
  std::unordered_map<std::string, double> joint_positions_;

  /** Latest joint name → effort/torque. */
  std::unordered_map<std::string, double> joint_efforts_;

  /** Incoming JointState payload queue. */
  integration::MessageQueue joint_queue_;

  /** Autolink reader for joint states. */
  std::shared_ptr<autolink::Reader<autolink::message::RawMessage>>
      joint_reader_;

  /** Autolink reader for robot_description (optional). */
  std::shared_ptr<autolink::Reader<autolink::message::RawMessage>>
      description_reader_;
};

}  // namespace display
}  // namespace autoviz
