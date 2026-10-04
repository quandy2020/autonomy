/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file marker_draw_utils.hpp
 * @brief Shared Marker / MarkerArray storage keys, mesh cache, and draw /
 *        upsert helpers.
 *
 * Both @ref MarkerDisplay and @ref MarkerArrayDisplay keep a
 * @c map<MarkerKey, StoredMarker> and call @ref upsertMarker /
 * @ref drawStoredMarkers so ADD/MODIFY/DELETE semantics and TF lookup stay
 * consistent.
 *
 * @see MarkerDisplay
 * @see MarkerArrayDisplay
 * @see drawStoredMarkers
 */

#pragma once

#include <map>

#include <automsgs/msgs/visualization_msgs/marker.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/display.hpp"
#include "autoviz/display/obj_mesh.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace display {

/**
 * @struct MarkerKey
 * @brief Unique key for a visualization_msgs marker: namespace + id.
 *
 * Ordering enables use as @c std::map key (ns primary, id secondary).
 *
 * @see StoredMarker
 * @see upsertMarker
 */
struct MarkerKey {
  std::string ns;  /**< Marker namespace (@c marker.ns). */
  int id = 0;      /**< Marker id (@c marker.id). */

  /**
   * @brief Strict weak ordering for @c std::map.
   *
   * @param other Other key.
   * @return @c true when @c this sorts before @p other.
   */
  bool operator<(const MarkerKey& other) const {
    if (ns != other.ns) {
      return ns < other.ns;
    }
    return id < other.id;
  }
};

/**
 * @struct StoredMarker
 * @brief Cached marker message plus optional loaded mesh resource.
 *
 * @c mesh / @c has_mesh support @c MESH_RESOURCE markers so OBJ data is not
 * reloaded every frame.
 *
 * @see MarkerKey
 * @see drawStoredMarkers
 */
struct StoredMarker {
  /** Latest visualization_msgs/Marker protobuf. */
  automsgs::msgs::visualization_msgs::Marker marker;

  /** Cached OBJ mesh for MESH_RESOURCE markers. */
  ObjMesh mesh;

  /** @c true when @c mesh was loaded successfully for the current resource. */
  bool has_mesh = false;
};

/**
 * @brief Draws all stored markers into @p scene with TF and property overrides.
 *
 * Applies lifetime expiry, fixed-frame transforms, and display-level color
 * overrides. @p display_prefix namespaces overlay object ids to avoid
 * collisions between Marker and MarkerArray instances.
 *
 * @param scene Overlay destination.
 * @param context Display context for TF / frame lookup; may be @c nullptr
 *        (markers in fixed frame only).
 * @param properties Display property map (color, alpha, …).
 * @param markers Namespace+id → stored marker map.
 * @param display_prefix Unique prefix for overlay entity names.
 *
 * @see upsertMarker
 * @see markerTransformInFixedFrame
 */
void drawStoredMarkers(
    rendering::SceneOverlay& scene, common::DisplayContext* context,
    const common::DisplayPropertyMap& properties,
    const std::map<MarkerKey, StoredMarker>& markers,
    const std::string& display_prefix);

/**
 * @brief Inserts or updates a marker; handles DELETE / DELETEALL actions.
 *
 * @param marker Incoming visualization_msgs/Marker.
 * @param[in,out] markers Mutable storage map.
 *
 * @see drawStoredMarkers
 * @see MarkerKey
 */
void upsertMarker(
    const automsgs::msgs::visualization_msgs::Marker& marker,
    std::map<MarkerKey, StoredMarker>* markers);

/**
 * @brief Computes the marker pose transform in the fixed frame.
 *
 * @param marker Source marker (frame_id + pose).
 * @param context Context providing FrameManager / TF buffer.
 * @return Transform matrix; identity or last-known on TF failure (see .cpp).
 *
 * @see drawStoredMarkers
 */
QMatrix4x4 markerTransformInFixedFrame(
    const automsgs::msgs::visualization_msgs::Marker& marker,
    common::DisplayContext* context);

}  // namespace display
}  // namespace autoviz
