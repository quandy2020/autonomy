/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/plot_drag_mime.hpp"

#include <QMimeData>

namespace autoviz {
namespace plot {

QMimeData* MakePlotSeriesDragPayload(const PlotSeriesDragPayload& payload) {
  auto* mime = new QMimeData();
  const QByteArray bytes =
      payload.channel.toUtf8() + '\n' + payload.field_path.toUtf8();
  mime->setData(QString::fromLatin1(kPlotSeriesDragMime), bytes);
  return mime;
}

bool ReadPlotSeriesDragPayload(const QMimeData* mime,
                               PlotSeriesDragPayload* out) {
  if (mime == nullptr || out == nullptr ||
      !mime->hasFormat(QString::fromLatin1(kPlotSeriesDragMime))) {
    return false;
  }
  const QByteArray bytes =
      mime->data(QString::fromLatin1(kPlotSeriesDragMime));
  const int split = bytes.indexOf('\n');
  if (split < 0) {
    out->channel = QString::fromUtf8(bytes);
    out->field_path.clear();
    return !out->channel.isEmpty();
  }
  out->channel = QString::fromUtf8(bytes.left(split));
  out->field_path = QString::fromUtf8(bytes.mid(split + 1));
  return !out->channel.isEmpty();
}

}  // namespace plot
}  // namespace autoviz
