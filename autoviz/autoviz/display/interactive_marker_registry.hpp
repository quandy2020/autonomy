/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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

struct InteractiveMarkerState {
  std::string server_id;
  std::string feedback_channel;
  std::string source_id;
  automsgs::msgs::visualization_msgs::InteractiveMarker marker;
};

struct InteractiveMarkerPick {
  bool hit = false;
  std::string marker_name;
  std::string control_name;
  std::string feedback_channel;
  std::string frame_id;
  QVector3D position;
  QMatrix4x4 control_transform;
  uint32_t interaction_mode = 0;
  int control_index = -1;
  float pixel_distance = 0.f;
};

/** Shared store for InteractiveMarkerDisplay + Interact tool. */
class InteractiveMarkerRegistry {
 public:
  void setSourceMarkers(
      const std::string& source_id,
      const std::map<std::string, InteractiveMarkerState>& markers);
  void removeSource(const std::string& source_id);

  const std::map<std::string, InteractiveMarkerState>& markers() const {
    return markers_;
  }

  bool updatePose(const std::string& marker_name,
                  const automsgs::msgs::geometry_msgs::Pose& pose);

  InteractiveMarkerPick pickMarker(rendering::ViewController* view_controller,
                                   int viewport_width, int viewport_height,
                                   int pixel_x, int pixel_y,
                                   autoviz::common::DisplayContext* context,
                                   float max_pixel_distance = 18.f) const;

  /** Apply constrained drag from initial pose using control interaction mode. */
  static automsgs::msgs::geometry_msgs::Pose draggedPose(
      const automsgs::msgs::geometry_msgs::Pose& initial_pose,
      const QMatrix4x4& control_transform, uint32_t interaction_mode,
      const QVector3D& initial_ground, const QVector3D& current_ground,
      bool shift_held);

 private:
  std::map<std::string, InteractiveMarkerState> markers_;
  std::map<std::string, std::string> source_by_marker_;
};

void applyInteractiveMarkerUpdate(
    const automsgs::msgs::visualization_msgs::InteractiveMarkerUpdate&
        update,
    const std::string& server_id, const std::string& feedback_channel,
    const std::string& source_id,
    std::map<std::string, InteractiveMarkerState>* markers);

void drawInteractiveMarker(
    rendering::SceneOverlay& scene, autoviz::common::DisplayContext* context,
    const autoviz::common::DisplayPropertyMap& properties,
    const InteractiveMarkerState& state);

QVector3D markerPositionInFixedFrame(
    const automsgs::msgs::visualization_msgs::InteractiveMarker&
        marker,
    autoviz::common::DisplayContext* context);

std::string defaultFeedbackChannel(const std::string& update_channel);

}  // namespace display
}  // namespace autoviz
