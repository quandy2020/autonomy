/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/scalar_sensor_utils.hpp"

#include <algorithm>

namespace autoviz {
namespace display {

QColor colorFromScalar(double value, double min_value, double max_value) {
  if (max_value <= min_value) {
    return QColor(200, 200, 200);
  }
  const double t =
      std::clamp((value - min_value) / (max_value - min_value), 0.0, 1.0);
  const int r = static_cast<int>(255 * std::min(1.0, std::max(0.0, 1.5 - std::abs(4 * t - 3))));
  const int g = static_cast<int>(255 * std::min(1.0, std::max(0.0, 1.5 - std::abs(4 * t - 2))));
  const int b = static_cast<int>(255 * std::min(1.0, std::max(0.0, 1.5 - std::abs(4 * t - 1))));
  return QColor(r, g, b);
}

}  // namespace display
}  // namespace autoviz
