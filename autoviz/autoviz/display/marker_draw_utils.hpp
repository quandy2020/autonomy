/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <map>

#include <automsgs/msgs/visualization_msgs/marker.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/display.hpp"
#include "autoviz/display/obj_mesh.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace display {

struct MarkerKey {
  std::string ns;
  int id = 0;
  bool operator<(const MarkerKey& other) const {
    if (ns != other.ns) {
      return ns < other.ns;
    }
    return id < other.id;
  }
};

struct StoredMarker {
  automsgs::msgs::visualization_msgs::Marker marker;
  ObjMesh mesh;
  bool has_mesh = false;
};

void drawStoredMarkers(
    rendering::SceneOverlay& scene, common::DisplayContext* context,
    const common::DisplayPropertyMap& properties,
    const std::map<MarkerKey, StoredMarker>& markers,
    const std::string& display_prefix);

void upsertMarker(
    const automsgs::msgs::visualization_msgs::Marker& marker,
    std::map<MarkerKey, StoredMarker>* markers);

QMatrix4x4 markerTransformInFixedFrame(
    const automsgs::msgs::visualization_msgs::Marker& marker,
    common::DisplayContext* context);

}  // namespace display
}  // namespace autoviz
