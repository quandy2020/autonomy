/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <optional>
#include <string>

namespace autoviz {
namespace plot {

struct PlotSample {
  double timestamp_sec = 0.0;
  double value = 0.0;
};

class PlotFieldExtractor {
 public:
  static PlotFieldExtractor& instance();

  std::optional<PlotSample> extract(const std::string& message_type,
                                    const std::string& payload,
                                    const std::string& field_path,
                                    double fallback_timestamp_sec) const;

  std::optional<double> extractNumeric(const std::string& message_type,
                                       const std::string& payload,
                                       const std::string& field_path) const;

  std::optional<double> extractTimestamp(
      const std::string& message_type, const std::string& payload,
      const std::string& timestamp_path, double fallback_timestamp_sec) const;

 private:
  PlotFieldExtractor() = default;
};

}  // namespace plot
}  // namespace autoviz
