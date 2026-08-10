/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include <QColor>

namespace autoviz {
namespace display {

/** RViz2 TFDisplay default axis length before Marker Scale. */
constexpr float kTfDefaultAxisLength = 0.2f;

/**
 * Filter frame ids like RViz2 whitelist/blacklist regex properties.
 * Empty whitelist → pass-all; empty blacklist → ban-none.
 * Invalid regex → that side treated as empty; optional error_out filled.
 */
std::vector<std::string> FilterTfFrameNames(
    const std::vector<std::string>& frames, const std::string& whitelist_regex,
    const std::string& blacklist_regex, std::string* error_out = nullptr);

struct TfAgeVisual {
  bool visible = true;
  QColor color = QColor(255, 255, 255);
  float alpha = 1.f;
};

/**
 * RViz2 Frame Timeout thirds: first third normal, second grey, third fade out,
 * then invisible. timeout_sec <= 0 disables aging.
 */
TfAgeVisual TfAgeVisualForTimeout(double age_sec, double timeout_sec,
                                  const QColor& base_rgb);

}  // namespace display
}  // namespace autoviz
