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
 * Used when Auto Scale is off. With Auto Scale, length tracks the TF tree
 * (typical parent→child spacing) so small robots are not buried in 0.2 m shafts.
 */
constexpr float kTfDefaultAxisLength = 0.2f;

/**
 * @brief Radius / length ratio for TF axes (RViz uses 0.02/0.2 = 0.1).
 *
 * Slightly thinner than RViz so dense trees stay readable on small platforms.
 */
constexpr float kTfAxisRadiusRatio = 0.055f;

/** @brief RViz2 @c Axes::getDefaultXColor() — pure red. */
inline QColor TfDefaultXColor() { return QColor::fromRgbF(1.f, 0.f, 0.f); }

/** @brief RViz2 @c Axes::getDefaultYColor() — pure green. */
inline QColor TfDefaultYColor() { return QColor::fromRgbF(0.f, 1.f, 0.f); }

/** @brief RViz2 @c Axes::getDefaultZColor() — pure blue. */
inline QColor TfDefaultZColor() { return QColor::fromRgbF(0.f, 0.f, 1.f); }

/** @brief RViz2 TF parent-arrow shaft (@c FrameInfo::ARROW_SHAFT_COLOR). */
inline QColor TfArrowShaftColor() {
  return QColor::fromRgbF(0.8f, 0.8f, 0.3f);
}

/** @brief RViz2 TF parent-arrow head (@c FrameInfo::ARROW_HEAD_COLOR). */
inline QColor TfArrowHeadColor() {
  return QColor::fromRgbF(1.0f, 0.1f, 0.6f);
}

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
 * Matches @c FrameInfo::updateColorForAge: first third keeps @p base_rgb;
 * second third lerps toward grey (0.7); last third fades grey alpha to 0;
 * past timeout → invisible. @p timeout_sec ≤ 0 disables aging.
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
