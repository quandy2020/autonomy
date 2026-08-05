/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>
#include <QVector>

class QMimeData;

namespace autoviz {
namespace plot {

constexpr char kPlotSeriesDragMime[] = "application/x-autoviz-plot-series";
constexpr char kPlotSeriesListDragMime[] = "application/x-autoviz-plot-series-list";

struct PlotSeriesDragPayload {
  QString channel;
  QString field_path;
};

QMimeData* MakePlotSeriesDragPayload(const PlotSeriesDragPayload& payload);
QMimeData* MakePlotSeriesListDragPayload(
    const QVector<PlotSeriesDragPayload>& payloads);
bool ReadPlotSeriesDragPayload(const QMimeData* mime, PlotSeriesDragPayload* out);
QVector<PlotSeriesDragPayload> ReadPlotSeriesDragPayloads(const QMimeData* mime);

}  // namespace plot
}  // namespace autoviz
