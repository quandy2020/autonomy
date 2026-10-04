/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file marker_display.hpp
 * @brief Displays individual @c visualization_msgs/Marker messages with
 *        ADD/MODIFY/DELETE lifetime semantics (RViz Marker).
 *
 * Maintains a @c map<MarkerKey, StoredMarker> updated by @ref upsertMarker
 * and rendered by @ref drawStoredMarkers.
 *
 * @see MarkerArrayDisplay
 * @see marker_draw_utils.hpp
 * @see ChannelDisplay
 */

#pragma once

#include <map>

#include <automsgs/msgs/visualization_msgs/marker.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"
#include "autoviz/display/marker_draw_utils.hpp"

namespace autoviz {
namespace display {

/**
 * @class MarkerDisplay
 * @brief Channel display for a stream of visualization_msgs/Marker.
 *
 * Unlike MarkerArray, each message is a single marker action. DELETEALL and
 * lifetime expiry are handled inside the shared draw/upsert helpers.
 *
 * @see MarkerArrayDisplay
 * @see upsertMarker
 * @see drawStoredMarkers
 */
class MarkerDisplay
    : public ChannelDisplay<
          automsgs::msgs::visualization_msgs::Marker> {
 public:
  /**
   * @brief Constructs the display bound to @p channel.
   *
   * @param channel Autolink channel for Marker messages.
   */
  explicit MarkerDisplay(std::string channel);

  /**
   * @brief Catalog type id.
   *
   * @return Always @c "Marker".
   */
  std::string typeId() const override { return "Marker"; }

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
   * @brief Upserts or deletes @p message in @c markers_.
   *
   * @param message visualization_msgs/Marker protobuf.
   */
  void processMessage(
      const automsgs::msgs::visualization_msgs::Marker& message)
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
