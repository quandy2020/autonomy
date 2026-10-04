/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file interactive_marker_registry.hpp
 * @brief Shared InteractiveMarker state store, picking, and drag helpers for
 *        @ref InteractiveMarkerDisplay and the Interact tool.
 *
 * Displays publish marker maps by source id; the Interact tool queries
 * @ref InteractiveMarkerRegistry::pickMarker and applies
 * @ref InteractiveMarkerRegistry::draggedPose / @ref updatePose during
 * manipulation, then publishes feedback on the marker's feedback channel.
 *
 * Free functions apply updates, draw a single marker, and resolve poses in
 * the fixed frame.
 *
 * @see InteractiveMarkerDisplay
 * @see InteractiveMarkerState
 * @see InteractiveMarkerPick
 */

#pragma once

#include <map>
#include <string>
#include <vector>

#include <QMatrix4x4>
#include <QVector3D>

#include <automsgs/msgs/visualization_msgs/interactive_marker.pb.h>
#include <automsgs/msgs/visualization_msgs/interactive_marker_update.pb.h>
#include "autoviz/display/display.hpp"
#include "autoviz/rendering/view_controller.hpp"

namespace autoviz {
namespace rendering {
class SceneOverlay;
}

namespace display {

/**
 * @struct InteractiveMarkerState
 * @brief One interactive marker plus routing metadata for feedback.
 *
 * @see InteractiveMarkerRegistry
 * @see InteractiveMarkerDisplay
 */
struct InteractiveMarkerState {
  std::string server_id;         /**< Server / update identity string. */
  std::string feedback_channel;  /**< Topic for InteractiveMarkerFeedback. */
  std::string source_id;         /**< Owning display source key in the registry. */
  /** Full visualization_msgs/InteractiveMarker definition. */
  automsgs::msgs::visualization_msgs::InteractiveMarker marker;
};

/**
 * @struct InteractiveMarkerPick
 * @brief Result of a pixel pick against interactive marker controls.
 *
 * @c hit is @c false when no control is within @c max_pixel_distance.
 *
 * @see InteractiveMarkerRegistry::pickMarker
 */
struct InteractiveMarkerPick {
  bool hit = false;                 /**< @c true when a control was selected. */
  std::string marker_name;          /**< Picked interactive marker name. */
  std::string control_name;         /**< Picked control name. */
  std::string feedback_channel;     /**< Feedback topic for this marker. */
  std::string frame_id;             /**< Marker header frame. */
  QVector3D position;               /**< Pick position in fixed frame. */
  QMatrix4x4 control_transform;     /**< Control pose used for drag constraints. */
  uint32_t interaction_mode = 0;    /**< visualization_msgs interaction mode. */
  int control_index = -1;           /**< Index into marker.controls. */
  float pixel_distance = 0.f;       /**< Screen distance from cursor to hit. */
};

/**
 * @class InteractiveMarkerRegistry
 * @brief Process-wide (per manager) store bridging displays and Interact.
 *
 * ## Ownership
 *
 * Displays call @ref setSourceMarkers / @ref removeSource. The Interact tool
 * reads @ref markers, @ref pickMarker, @ref updatePose, and
 * @ref draggedPose. The registry does not own displays.
 *
 * @see InteractiveMarkerDisplay
 * @see InteractiveMarkerPick
 */
class InteractiveMarkerRegistry {
 public:
  /**
   * @brief Replaces all markers published by @p source_id.
   *
   * Updates @c source_by_marker_ so later pose edits can attribute ownership.
   *
   * @param source_id Display source key (@ref InteractiveMarkerDisplay).
   * @param markers Name → state map for that source.
   * @see removeSource()
   */
  void setSourceMarkers(
      const std::string& source_id,
      const std::map<std::string, InteractiveMarkerState>& markers);

  /**
   * @brief Removes every marker belonging to @p source_id.
   *
   * @param source_id Display source key to clear.
   */
  void removeSource(const std::string& source_id);

  /**
   * @brief All interactive markers currently registered (all sources).
   *
   * @return Const reference to the aggregated marker map.
   */
  const std::map<std::string, InteractiveMarkerState>& markers() const {
    return markers_;
  }

  /**
   * @brief Writes a new pose into the named marker if present.
   *
   * @param marker_name Interactive marker name key.
   * @param pose New pose in the marker's frame.
   * @return @c true when the marker existed and was updated.
   */
  bool updatePose(const std::string& marker_name,
                  const automsgs::msgs::geometry_msgs::Pose& pose);

  /**
   * @brief Picks the closest interactive control under a viewport pixel.
   *
   * @param view_controller Active camera controller (ray casting).
   * @param viewport_width Viewport width in pixels.
   * @param viewport_height Viewport height in pixels.
   * @param pixel_x Cursor X in viewport pixels.
   * @param pixel_y Cursor Y in viewport pixels.
   * @param context Display context for TF / frames; may be @c nullptr.
   * @param max_pixel_distance Maximum screen-space hit distance (default 18).
   * @return Populated @ref InteractiveMarkerPick (@c hit may be false).
   */
  InteractiveMarkerPick pickMarker(rendering::ViewController* view_controller,
                                   int viewport_width, int viewport_height,
                                   int pixel_x, int pixel_y,
                                   autoviz::common::DisplayContext* context,
                                   float max_pixel_distance = 18.f) const;

  /**
   * @brief Applies constrained drag from an initial pose using the control
   *        interaction mode.
   *
   * Ground-plane (or mode-specific) motion is derived from
   * @p initial_ground → @p current_ground; @p shift_held may switch to
   * alternate axes (RViz Interact parity).
   *
   * @param initial_pose Pose at mouse-down.
   * @param control_transform Control frame used for axis constraints.
   * @param interaction_mode visualization_msgs interaction mode enum value.
   * @param initial_ground Ground-plane hit at mouse-down.
   * @param current_ground Ground-plane hit at current cursor.
   * @param shift_held Whether Shift modifies the drag axes.
   * @return Updated pose suitable for @ref updatePose / feedback.
   */
  static automsgs::msgs::geometry_msgs::Pose draggedPose(
      const automsgs::msgs::geometry_msgs::Pose& initial_pose,
      const QMatrix4x4& control_transform, uint32_t interaction_mode,
      const QVector3D& initial_ground, const QVector3D& current_ground,
      bool shift_held);

 private:
  /** Aggregated marker name → state (all sources). */
  std::map<std::string, InteractiveMarkerState> markers_;

  /** Marker name → owning source_id for remove/replace. */
  std::map<std::string, std::string> source_by_marker_;
};

/**
 * @brief Applies an InteractiveMarkerUpdate to a local marker map.
 *
 * Handles keep-alive, erase, and upsert of markers with routing metadata.
 *
 * @param update Update message from the server.
 * @param server_id Server identity stored on each state.
 * @param feedback_channel Feedback topic stored on each state.
 * @param source_id Owning display source id.
 * @param[in,out] markers Mutable name → state map.
 *
 * @see InteractiveMarkerDisplay::processMessage
 */
void applyInteractiveMarkerUpdate(
    const automsgs::msgs::visualization_msgs::InteractiveMarkerUpdate&
        update,
    const std::string& server_id, const std::string& feedback_channel,
    const std::string& source_id,
    std::map<std::string, InteractiveMarkerState>* markers);

/**
 * @brief Draws one interactive marker (controls + visuals) into @p scene.
 *
 * @param scene Overlay destination.
 * @param context Context for TF; may be @c nullptr.
 * @param properties Display property overrides (alpha, show axes, …).
 * @param state Marker + routing metadata to draw.
 *
 * @see markerPositionInFixedFrame
 */
void drawInteractiveMarker(
    rendering::SceneOverlay& scene, autoviz::common::DisplayContext* context,
    const autoviz::common::DisplayPropertyMap& properties,
    const InteractiveMarkerState& state);

/**
 * @brief Resolves the interactive marker pose origin in the fixed frame.
 *
 * @param marker Interactive marker definition (header + pose).
 * @param context Context providing TF; may be @c nullptr.
 * @return Position in fixed frame (origin on TF failure — see .cpp).
 */
QVector3D markerPositionInFixedFrame(
    const automsgs::msgs::visualization_msgs::InteractiveMarker&
        marker,
    autoviz::common::DisplayContext* context);

/**
 * @brief Derives the default feedback channel from an update channel name.
 *
 * Typical convention: replace trailing @c /update with @c /feedback.
 *
 * @param update_channel InteractiveMarker update topic.
 * @return Suggested feedback topic string.
 */
std::string defaultFeedbackChannel(const std::string& update_channel);

}  // namespace display
}  // namespace autoviz
