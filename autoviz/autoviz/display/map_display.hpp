/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

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

class MapDisplay
    : public ChannelDisplay<automsgs::msgs::map_msgs::OccupancyGrid> {
 public:
  explicit MapDisplay(std::string channel);

  std::string typeId() const override { return "Map"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void onEnable() override;
  void onDisable() override;
  void onUpdate() override;
  void onPropertyChanged(const std::string& key) override;
  void processMessage(
      const automsgs::msgs::map_msgs::OccupancyGrid& message)
      override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  void rebuildGeometry();
  void applyUpdate(const automsgs::msgs::map_msgs::OccupancyGridUpdate& update);
  void clearGeometry();

  QImage image_;
  QVector3D top_left_;
  QVector3D top_right_;
  QVector3D bottom_right_;
  QVector3D bottom_left_;
  bool has_geometry_ = false;
  automsgs::msgs::map_msgs::OccupancyGrid current_message_;
  bool has_full_map_ = false;
  integration::MessageQueue update_queue_;
  integration::ChannelReaderRegistry::SubscriptionId update_subscription_id_ = 0;
};

}  // namespace display
}  // namespace autoviz
