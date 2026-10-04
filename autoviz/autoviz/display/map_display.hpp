/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file map_display.hpp
 * @brief Displays @c map_msgs/OccupancyGrid (and optional updates) as a
 *        textured ground quad (RViz Map).
 *
 * Subscribes to the full map via @ref ChannelDisplay and optionally to an
 * OccupancyGridUpdate channel for incremental patches. Rebuilds a @c QImage
 * colorized by the selected scheme (map/costmap/raw) and draws it as a
 * textured quad in the map frame.
 *
 * @see ChannelDisplay
 * @see GridCellsDisplay
 * @see GridMapDisplay
 */

#pragma once

#include <QColor>
#include <QImage>
#include <QVector3D>
#include <vector>

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/map_msgs/occupancy_grid_update.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/integration/message_queue.hpp"

namespace autoviz {
namespace display {

/**
 * @class MapDisplay
 * @brief Occupancy grid map texture display with optional update topic.
 *
 * Properties include alpha, color scheme, decimation, draw-behind, and
 * timestamp usage for TF.
 *
 * @see ChannelDisplay
 * @see GridMapDisplay
 */
class MapDisplay
    : public ChannelDisplay<automsgs::msgs::map_msgs::OccupancyGrid> {
 public:
  /**
   * @brief Constructs the display bound to the full-map @p channel.
   *
   * @param channel Autolink channel for OccupancyGrid messages.
   */
  explicit MapDisplay(std::string channel);

  /**
   * @brief Catalog type id.
   *
   * @return Always @c "Map".
   */
  std::string typeId() const override { return "Map"; }

  /**
   * @brief Property schema (alpha, scheme, update topic, …).
   *
   * @return Specs for the Displays property editor.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Enables base channel subscription plus optional update channel.
   */
  void onEnable() override;

  /**
   * @brief Unsubscribes update channel and clears base subscription.
   */
  void onDisable() override;

  /**
   * @brief Drains update queue then base ChannelDisplay update.
   */
  void onUpdate() override;

  /**
   * @brief Rebuilds geometry when scheme/alpha/etc. change.
   *
   * @param key Changed property key.
   */
  void onPropertyChanged(const std::string& key) override;

  /**
   * @brief Stores a full OccupancyGrid and rebuilds the texture.
   *
   * @param message Full map_msgs/OccupancyGrid.
   */
  void processMessage(
      const automsgs::msgs::map_msgs::OccupancyGrid& message)
      override;

  /**
   * @brief Draws the textured map quad when @c has_geometry_ is set.
   *
   * @param scene Scene overlay destination.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

  /**
   * @brief Clears map message, update state, and geometry.
   */
  void clearReceivedData() override;

 private:
  /**
   * @brief Rebuilds @c image_ and corner positions from @c current_message_.
   */
  void rebuildGeometry();

  /**
   * @brief Patches @c current_message_ with an OccupancyGridUpdate and
   *        rebuilds.
   *
   * @param update Incremental occupancy update protobuf.
   */
  void applyUpdate(const automsgs::msgs::map_msgs::OccupancyGridUpdate& update);

  /**
   * @brief Clears texture, corners, and @c has_geometry_.
   */
  void clearGeometry();

  /** Colorized occupancy texture. */
  QImage image_;

  QVector3D top_left_;      /**< Map quad corner in fixed/map frame. */
  QVector3D top_right_;     /**< Map quad corner. */
  QVector3D bottom_right_;  /**< Map quad corner. */
  QVector3D bottom_left_;   /**< Map quad corner. */

  /** @c true when texture + corners are ready to draw. */
  bool has_geometry_ = false;

  /** Latest full map (also target of incremental updates). */
  automsgs::msgs::map_msgs::OccupancyGrid current_message_;

  /** @c true after at least one full OccupancyGrid was received. */
  bool has_full_map_ = false;

  /** Queue for OccupancyGridUpdate payloads. */
  integration::MessageQueue update_queue_;

  /** Registry subscription for the update channel; @c 0 if none. */
  integration::ChannelReaderRegistry::SubscriptionId update_subscription_id_ = 0;
};

}  // namespace display
}  // namespace autoviz
