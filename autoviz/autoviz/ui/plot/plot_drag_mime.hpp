/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>

class QMimeData;

namespace autoviz {
namespace plot {

constexpr char kPlotSeriesDragMime[] = "application/x-autoviz-plot-series";

struct PlotSeriesDragPayload {
  QString channel;
  QString field_path;
};

QMimeData* MakePlotSeriesDragPayload(const PlotSeriesDragPayload& payload);
bool ReadPlotSeriesDragPayload(const QMimeData* mime, PlotSeriesDragPayload* out);

}  // namespace plot
}  // namespace autoviz
