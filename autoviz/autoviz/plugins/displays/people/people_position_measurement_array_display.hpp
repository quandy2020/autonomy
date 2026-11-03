/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include <QColor>
#include <QVector3D>

#include <automsgs/msgs/people_msgs/position_measurement_array.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"

namespace autoviz {
namespace display {

class PeoplePositionMeasurementArrayDisplay
    : public ChannelDisplay<
          automsgs::msgs::people_msgs::PositionMeasurementArray> {
 public:
  explicit PeoplePositionMeasurementArrayDisplay(std::string channel);

  std::string typeId() const override {
    return "PeoplePositionMeasurementArray";
  }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void processMessage(
      const automsgs::msgs::people_msgs::PositionMeasurementArray& message)
      override;
  void clearReceivedData() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct PersonVisual {
    QVector3D position;
    QColor color;
    float radius;
  };

  std::vector<PersonVisual> people_;
};

}  // namespace display
}  // namespace autoviz
