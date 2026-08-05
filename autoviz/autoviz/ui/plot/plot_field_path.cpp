/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/plot_field_path.hpp"

#include <cmath>

namespace autoviz {
namespace plot {

ParsedFieldPath ParseFieldPath(const QString& field_path) {
  ParsedFieldPath parsed;
  const int marker = field_path.indexOf(QStringLiteral(".@"));
  if (marker < 0) {
    parsed.base_path = field_path;
    return parsed;
  }
  parsed.base_path = field_path.left(marker);
  QString rest = field_path.mid(marker + 2);
  while (!rest.isEmpty()) {
    const int next = rest.indexOf(QStringLiteral(".@"));
    if (next < 0) {
      parsed.modifiers.push_back(rest);
      break;
    }
    parsed.modifiers.push_back(rest.left(next));
    rest = rest.mid(next + 2);
  }
  return parsed;
}

double ApplyPlotModifiers(double raw_value, double timestamp_sec,
                          const QStringList& modifiers,
                          double* last_raw_value, double* last_timestamp_sec,
                          bool* has_last_sample) {
  double result = raw_value;
  for (const QString& modifier : modifiers) {
    if (modifier == QStringLiteral("abs")) {
      result = std::abs(result);
    } else if (modifier == QStringLiteral("log")) {
      result = result > 0.0 ? std::log(result) : 0.0;
    } else if (modifier == QStringLiteral("derivative")) {
      if (last_raw_value != nullptr && last_timestamp_sec != nullptr &&
          has_last_sample != nullptr && *has_last_sample) {
        const double dt = timestamp_sec - *last_timestamp_sec;
        result = dt > 1e-9 ? (raw_value - *last_raw_value) / dt : 0.0;
      } else {
        result = 0.0;
      }
    }
  }
  if (last_raw_value != nullptr) {
    *last_raw_value = raw_value;
  }
  if (last_timestamp_sec != nullptr) {
    *last_timestamp_sec = timestamp_sec;
  }
  if (has_last_sample != nullptr) {
    *has_last_sample = true;
  }
  return result;
}

}  // namespace plot
}  // namespace autoviz
