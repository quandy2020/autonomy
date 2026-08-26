/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QVector3D>
#include <unordered_map>
#include <vector>

#include <automsgs/msgs/strata_msgs/road_graph.pb.h>
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

class StrataRoadGraphDisplay
    : public ChannelDisplay<automsgs::msgs::strata_msgs::RoadGraph> {
 public:
  explicit StrataRoadGraphDisplay(std::string channel);

  std::string typeId() const override { return "StrataRoadGraph"; }
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;
  void onPropertyChanged(const std::string& key) override;

 protected:
  void processMessage(
      const automsgs::msgs::strata_msgs::RoadGraph& message) override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct StoredEdge {
    QVector3D from;
    QVector3D to;
  };

  std::vector<QVector3D> nodes_;
  std::vector<StoredEdge> edges_;
  QColor node_color_{51, 230, 77};
  QColor edge_color_{51, 204, 102};
};

}  // namespace display
}  // namespace autoviz
