/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file plot_field_path.hpp
 * @brief Parse plot field paths and apply math modifiers (@abs, @derivative, …).
 *
 * A stored series path may be @c "twist.linear.x@abs@derivative". Parsing
 * splits the protobuf base path from trailing modifiers; @ref ApplyPlotModifiers()
 * transforms successive raw samples.
 *
 * @see PlotFieldExtractor
 * @see PlotSeriesConfig
 */

#pragma once

#include <QString>
#include <QStringList>

namespace autoviz {
namespace plot {

/**
 * @struct ParsedFieldPath
 * @brief Field path split into protobuf base and ordered modifiers.
 */
struct ParsedFieldPath {
  QString base_path;       /**< Protobuf / message path without modifiers. */
  QStringList modifiers;   /**< Ordered modifier tokens (e.g. @c abs, @c log). */
};

/**
 * @brief Splits @p field_path into base path and @c @modifier suffixes.
 *
 * @param field_path Full path as stored in @ref PlotSeriesConfig::field_path.
 * @return Parsed base + modifiers (modifiers empty when none).
 */
ParsedFieldPath ParseFieldPath(const QString& field_path);

/**
 * @brief Returns @c true when @p field_path contains a @c [:] / @c [] expand.
 *
 * @param field_path Full or base field path.
 */
bool FieldPathHasArrayExpand(const QString& field_path);

/**
 * @brief Returns @c true when @p modifiers include @c norm.
 *
 * @param modifiers Ordered modifier tokens.
 */
bool FieldPathHasNormModifier(const QStringList& modifiers);

/**
 * @brief Returns @p modifiers with @c norm tokens removed.
 *
 * @param modifiers Ordered modifier tokens.
 */
QStringList StripNormModifier(const QStringList& modifiers);

/**
 * @brief Apply math modifiers (@abs, @log, @derivative, …) to a raw sample.
 *
 * Supported unary: abs, log, log2, log10, sqrt, negative, sign, degrees,
 * radians, floor, ceil, round, sin, cos, tan.
 * Binary (arg via @c name:val, @c name(val), or next numeric token):
 * add, sub, mul, div.
 * Stateful: derivative, delta, timedelta.
 * @c norm is ignored here — extract the vector norm before calling.
 *
 * @param raw_value Incoming numeric sample.
 * @param timestamp_sec Sample time (seconds) for derivative / rate.
 * @param modifiers Ordered modifier names from @ref ParsedFieldPath.
 * @param last_raw_value In/out: previous raw value for stateful modifiers.
 * @param last_timestamp_sec In/out: previous sample time.
 * @param has_last_sample In/out: whether a previous sample exists.
 * @return Transformed value to append to the series.
 */
double ApplyPlotModifiers(double raw_value, double timestamp_sec,
                          const QStringList& modifiers,
                          double* last_raw_value, double* last_timestamp_sec,
                          bool* has_last_sample);

}  // namespace plot
}  // namespace autoviz
