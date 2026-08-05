/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QString>

namespace autoviz {
namespace gauge {

enum class GaugeColorMode {
  kSolid = 0,
  kGradient = 1,
  kColorMap = 2,
};

enum class GaugeColorMap {
  kRedYellowGreen = 0,
  kRainbow = 1,
  kTurbo = 2,
};

struct GaugePanelConfig {
  QString title;
  QString channel;
  QString field_path;
  double min_value = 0.0;
  double max_value = 100.0;
  GaugeColorMode color_mode = GaugeColorMode::kGradient;
  GaugeColorMap color_map = GaugeColorMap::kRedYellowGreen;
  QColor gradient_start = QColor(200, 70, 70);
  QColor gradient_end = QColor(70, 170, 95);
  bool reverse_color = false;
  bool reverse_direction = false;
};

GaugePanelConfig DefaultGaugePanelConfig();

}  // namespace gauge
}  // namespace autoviz
