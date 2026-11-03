/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <chrono>
#include <deque>
#include <string>
#include <vector>

#include <QColor>
#include <QVector3D>

#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/display.hpp"

namespace autoviz {
namespace display {

class TFTrajectoryDisplay : public Display {
 public:
  TFTrajectoryDisplay();

  std::string typeId() const override { return "TFTrajectory"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void onUpdate() override;
  void onDraw(rendering::SceneOverlay& scene) override;
  void reset() override;

 private:
  struct TrailSample {
    QVector3D position;
    std::chrono::steady_clock::time_point stamp;
  };

  std::deque<TrailSample> trail_;
};

}  // namespace display
}  // namespace autoviz
