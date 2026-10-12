/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file scalar_sensor_utils.hpp
 * @brief Color mapping helper for @ref ScalarSensorDisplay.
 *
 * Maps a scalar into a QColor by linearly interpolating a fixed hue ramp
 * between @p min_value and @p max_value (clamped).
 *
 * @note Distinct from the PointCloud2 @c colorFromScalar overload in
 *       @ref point_cloud_utils.hpp (different signature / behavior).
 *
 * @see ScalarSensorDisplay
 * @see colorFromScalar()
 */

#pragma once

#include <QColor>

namespace autoviz {
namespace display {

/**
 * @brief Maps @p value into a display color between @p min_value and @p max_value.
 *
 * @param value Scalar sample.
 * @param min_value Range minimum (clamped).
 * @param max_value Range maximum (clamped); if ≤ min, treated as a tiny epsilon.
 * @return QColor for the crosshair / point.
 */
QColor colorFromScalar(double value, double min_value, double max_value);

}  // namespace display
}  // namespace autoviz
