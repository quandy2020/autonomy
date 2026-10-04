/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file marker_array_display.hpp
 * @brief Displays @c visualization_msgs/MarkerArray by upserting each marker
 *        into a shared @c MarkerKey map (RViz MarkerArray).
 *
 * Reuses @ref upsertMarker / @ref drawStoredMarkers from
 * @ref marker_draw_utils.hpp so semantics match @ref MarkerDisplay.
 *
 * @see MarkerDisplay
 * @see marker_draw_utils.hpp
 * @see ChannelDisplay
 */

#pragma once

#include <map>

#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"
#include "autoviz/display/marker_draw_utils.hpp"

namespace autoviz {
namespace display {

/**
 * @class MarkerArrayDisplay
 * @brief Channel display for MarkerArray topics.
 *
 * Each array message upserts (or deletes) entries in @c markers_; property
 * changes may force a redraw without clearing the map.
 *
 * @see MarkerDisplay
 * @see drawStoredMarkers
 */
class MarkerArrayDisplay
    : public ChannelDisplay<
          automsgs::msgs::visualization_msgs::MarkerArray> {
 public:
  /**
   * @brief Constructs the display bound to @p channel.
   *
   * @param channel Autolink channel for MarkerArray messages.
   */
  explicit MarkerArrayDisplay(std::string channel);

  /**
   * @brief Catalog type id.
   *
   * @return Always @c "MarkerArray".
   */
  std::string typeId() const override { return "MarkerArray"; }

  /**
   * @brief Property schema (color override, alpha, …).
   *
   * @return Specs for the Displays property editor.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

  /**
   * @brief Reacts to property edits (e.g. color override).
   *
   * @param key Changed property key.
   */
  void onPropertyChanged(const std::string& key) override;

 protected:
  /**
   * @brief Upserts every marker in @p message into @c markers_.
   *
   * @param message visualization_msgs/MarkerArray protobuf.
   */
  void processMessage(
      const automsgs::msgs::visualization_msgs::MarkerArray& message)
      override;

  /**
   * @brief Draws all stored markers via @ref drawStoredMarkers.
   *
   * @param scene Scene overlay destination.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

  /**
   * @brief Clears @c markers_.
   */
  void clearReceivedData() override;

 private:
  /** Namespace+id → cached marker / mesh. */
  std::map<MarkerKey, StoredMarker> markers_;
};

}  // namespace display
}  // namespace autoviz
