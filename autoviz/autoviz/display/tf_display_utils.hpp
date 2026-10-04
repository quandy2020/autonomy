/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file tf_display_utils.hpp
 * @brief Helpers for TF frame filtering and timeout-based aging colors.
 *
 * Shared by @ref TfDisplay (and any UI that mirrors RViz2 TF filter /
 * Frame Timeout behavior).
 *
 * @see TfDisplay
 * @see FilterTfFrameNames()
 * @see TfAgeVisualForTimeout()
 * @see kTfDefaultAxisLength
 */

#pragma once

#include <string>
#include <vector>

#include <QColor>

namespace autoviz {
namespace display {

/**
 * @brief RViz2 TFDisplay default axis length before Marker Scale.
 *
 * Multiplied by the @c marker_scale property when drawing axes.
 */
constexpr float kTfDefaultAxisLength = 0.2f;

/**
 * @brief Filters frame ids like RViz2 whitelist/blacklist regex properties.
 *
 * Empty whitelist → pass-all; empty blacklist → ban-none.
 * Invalid regex → that side treated as empty; optional @p error_out filled.
 *
 * @param frames Candidate frame ids.
 * @param whitelist_regex ECMAScript whitelist; empty disables.
 * @param blacklist_regex ECMAScript blacklist; empty disables.
 * @param error_out Optional destination for regex error text.
 * @return Subset of @p frames that pass both filters (order preserved).
 */
std::vector<std::string> FilterTfFrameNames(
    const std::vector<std::string>& frames, const std::string& whitelist_regex,
    const std::string& blacklist_regex, std::string* error_out = nullptr);

/**
 * @struct TfAgeVisual
 * @brief Visibility / color / alpha for a frame under Frame Timeout aging.
 */
struct TfAgeVisual {
  bool visible = true;                    /**< When @c false, skip drawing. */
  QColor color = QColor(255, 255, 255);   /**< RGB before alpha multiply. */
  float alpha = 1.f;                      /**< Opacity in [0, 1]. */
};

/**
 * @brief Computes RViz2 Frame Timeout aging for a frame of age @p age_sec.
 *
 * First third of @p timeout_sec: normal; second: grey; third: fade out;
 * then invisible. @p timeout_sec ≤ 0 disables aging (always fully visible).
 *
 * @param age_sec Wall age since last transform-to-fixed update.
 * @param timeout_sec Frame Timeout property (seconds).
 * @param base_rgb Base RGB before aging remaps color/alpha.
 * @return Visibility and color/alpha for drawing.
 */
TfAgeVisual TfAgeVisualForTimeout(double age_sec, double timeout_sec,
                                  const QColor& base_rgb);

}  // namespace display
}  // namespace autoviz
