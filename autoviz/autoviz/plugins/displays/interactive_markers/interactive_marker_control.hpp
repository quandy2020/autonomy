/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins InteractiveMarkerControl
 * (Willow Garage / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file interactive_marker_control.hpp
 * @brief One InteractiveMarker control element (RViz InteractiveMarkerControl).
 *
 * Autoviz draws into @ref rendering::SceneOverlay each frame instead of owning
 * persistent Ogre marker entities. Interaction (pick / drag) is driven by
 * @ref InteractiveMarkerRegistry and @ref InteractTool using the same modes.
 *
 * @see InteractiveMarker
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <QColor>
#include <QMatrix4x4>
#include <QQuaternion>
#include <QString>
#include <QVector3D>

#include <automsgs/msgs/visualization_msgs/interactive_marker.pb.h>
#include <automsgs/msgs/visualization_msgs/interactive_marker_control.pb.h>
#include <automsgs/msgs/visualization_msgs/marker.pb.h>
#include "autoviz/common/display_property.hpp"

namespace autoviz {

namespace common {
class DisplayContext;
}

namespace rendering {
class SceneOverlay;
}

namespace display {

class InteractiveMarker;

/**
 * @brief Fills default control handles (RViz / interactive_markers::autoComplete).
 *
 * Empty MOVE_AXIS → dual ARROW; ROTATE_AXIS / MOVE_PLANE / MOVE_ROTATE → disc.
 * Mutates @p marker in place so display and registry share the same geometry.
 */
void autoCompleteInteractiveMarker(
    automsgs::msgs::visualization_msgs::InteractiveMarker* marker);

/**
 * @class InteractiveMarkerControl
 * @brief Single control of an @ref InteractiveMarker.
 */
class InteractiveMarkerControl
    : public std::enable_shared_from_this<InteractiveMarkerControl> {
 public:
  using SharedPtr = std::shared_ptr<InteractiveMarkerControl>;

  /** Highlight intensity (matches RViz ControlHighlight). */
  enum ControlHighlight {
    kNoHighlight = 0,
    kHoverHighlight = 3,
    kActiveHighlight = 5,
  };

  /**
   * @brief Visual cue family while highlighted (translate vs rotate).
   */
  enum class HighlightKind {
    kAuto = 0,       /**< Derive from interaction_mode_. */
    kTranslate = 1,  /**< Cyan tint + axis aid. */
    kRotate = 2,     /**< Amber tint + ring aid. */
    kBoth = 3,
  };

  /**
   * @brief Constructs a control owned by @p parent.
   *
   * @param context Display context (TF / fixed frame); non-owning.
   * @param parent Owning interactive marker; non-owning.
   */
  InteractiveMarkerControl(common::DisplayContext* context,
                           InteractiveMarker* parent);

  ~InteractiveMarkerControl() = default;

  /**
   * @brief Rebuilds this control from a message (RViz processMessage).
   */
  void processMessage(
      const automsgs::msgs::visualization_msgs::InteractiveMarkerControl&
          message);

  /**
   * @brief Updates local frame when the parent marker pose changes.
   */
  void interactiveMarkerPoseChanged(const QVector3D& int_marker_position,
                                    const QQuaternion& int_marker_orientation);

  /** @return @c true when interaction_mode is not NONE. */
  bool isInteractive() const;

  /** Per-frame update (view-facing orientation refresh). */
  void update();

  void setVisible(bool visible);
  bool getVisible() const { return visible_; }

  void setHighlight(ControlHighlight highlight,
                    HighlightKind kind = HighlightKind::kAuto);

  HighlightKind resolvedHighlightKind() const;

  InteractiveMarker* getParent() const { return parent_; }
  const std::string& getName() const { return name_; }
  const QString& getDescription() const { return description_; }
  int getInteractionMode() const { return interaction_mode_; }
  int getOrientationMode() const { return orientation_mode_; }

  void setShowVisualAids(bool show) { show_visual_aids_ = show; }

  /**
   * @brief Control transform in the fixed frame (marker pose × control ori).
   */
  QMatrix4x4 controlTransform() const { return control_transform_; }

  /**
   * @brief Draws markers / default handles into @p scene.
   */
  void draw(rendering::SceneOverlay& scene,
            const common::DisplayPropertyMap& properties,
            const std::string& draw_prefix) const;

  /**
   * @brief World-space pick samples for screen-space hit testing.
   */
  void collectPickSamples(float marker_scale,
                          std::vector<QVector3D>* samples) const;

 private:
  void rebuildControlTransform();
  void drawDefaultHandle(rendering::SceneOverlay& scene, float scale,
                         const QColor& color) const;

  common::DisplayContext* context_ = nullptr;
  InteractiveMarker* parent_ = nullptr;

  automsgs::msgs::visualization_msgs::InteractiveMarkerControl message_;
  std::vector<automsgs::msgs::visualization_msgs::Marker> markers_;

  std::string name_;
  QString description_;
  int interaction_mode_ = 0;
  int orientation_mode_ = 0;
  bool independent_marker_orientation_ = false;
  bool always_visible_ = false;
  bool visible_ = true;
  bool show_visual_aids_ = false;
  float highlight_ = 0.f;
  HighlightKind highlight_kind_ = HighlightKind::kAuto;

  QQuaternion control_orientation_;
  QVector3D parent_position_;
  QQuaternion parent_orientation_;
  /** Interaction frame (RViz control_frame_node_ × control orientation). */
  QMatrix4x4 control_transform_;
  /** Visual markers frame (RViz markers_node_). */
  QMatrix4x4 markers_transform_;
};

}  // namespace display
}  // namespace autoviz
