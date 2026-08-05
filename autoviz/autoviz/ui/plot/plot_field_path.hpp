/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>
#include <QStringList>

namespace autoviz {
namespace plot {

struct ParsedFieldPath {
  QString base_path;
  QStringList modifiers;
};

ParsedFieldPath ParseFieldPath(const QString& field_path);

/** Apply math modifiers (@abs, @log, @derivative, …) to a raw sample value. */
double ApplyPlotModifiers(double raw_value, double timestamp_sec,
                          const QStringList& modifiers,
                          double* last_raw_value, double* last_timestamp_sec,
                          bool* has_last_sample);

}  // namespace plot
}  // namespace autoviz
