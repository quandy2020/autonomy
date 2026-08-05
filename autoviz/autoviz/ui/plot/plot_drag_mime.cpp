/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/plot_drag_mime.hpp"

#include <QMimeData>

namespace autoviz {
namespace plot {
namespace {

QByteArray EncodePayload(const PlotSeriesDragPayload& payload) {
  return payload.channel.toUtf8() + '\n' + payload.field_path.toUtf8();
}

PlotSeriesDragPayload DecodePayload(const QByteArray& bytes) {
  PlotSeriesDragPayload payload;
  const int split = bytes.indexOf('\n');
  if (split < 0) {
    payload.channel = QString::fromUtf8(bytes);
    return payload;
  }
  payload.channel = QString::fromUtf8(bytes.left(split));
  payload.field_path = QString::fromUtf8(bytes.mid(split + 1));
  return payload;
}

}  // namespace

QMimeData* MakePlotSeriesDragPayload(const PlotSeriesDragPayload& payload) {
  auto* mime = new QMimeData();
  mime->setData(QString::fromLatin1(kPlotSeriesDragMime), EncodePayload(payload));
  return mime;
}

QMimeData* MakePlotSeriesListDragPayload(
    const QVector<PlotSeriesDragPayload>& payloads) {
  auto* mime = new QMimeData();
  if (payloads.size() == 1) {
    mime->setData(QString::fromLatin1(kPlotSeriesDragMime),
                  EncodePayload(payloads.front()));
    return mime;
  }
  QByteArray encoded;
  encoded.append(QByteArray::number(payloads.size()));
  encoded.append('\n');
  for (const PlotSeriesDragPayload& payload : payloads) {
    encoded.append(EncodePayload(payload));
    encoded.append('\n');
  }
  mime->setData(QString::fromLatin1(kPlotSeriesListDragMime), encoded);
  return mime;
}

bool ReadPlotSeriesDragPayload(const QMimeData* mime,
                               PlotSeriesDragPayload* out) {
  const QVector<PlotSeriesDragPayload> payloads = ReadPlotSeriesDragPayloads(mime);
  if (payloads.isEmpty()) {
    return false;
  }
  if (out != nullptr) {
    *out = payloads.front();
  }
  return true;
}

QVector<PlotSeriesDragPayload> ReadPlotSeriesDragPayloads(const QMimeData* mime) {
  QVector<PlotSeriesDragPayload> payloads;
  if (mime == nullptr) {
    return payloads;
  }
  if (mime->hasFormat(QString::fromLatin1(kPlotSeriesListDragMime))) {
    const QByteArray bytes =
        mime->data(QString::fromLatin1(kPlotSeriesListDragMime));
    const int first_newline = bytes.indexOf('\n');
    if (first_newline < 0) {
      return payloads;
    }
    bool ok = false;
    const int count = bytes.left(first_newline).toInt(&ok);
    if (!ok || count <= 0) {
      return payloads;
    }
    int offset = first_newline + 1;
    for (int i = 0; i < count && offset < bytes.size(); ++i) {
      const int next_newline = bytes.indexOf('\n', offset);
      const QByteArray chunk =
          next_newline < 0 ? bytes.mid(offset) : bytes.mid(offset, next_newline - offset);
      if (chunk.isEmpty()) {
        break;
      }
      PlotSeriesDragPayload payload = DecodePayload(chunk);
      if (!payload.channel.isEmpty()) {
        payloads.push_back(payload);
      }
      if (next_newline < 0) {
        break;
      }
      offset = next_newline + 1;
    }
    return payloads;
  }
  if (mime->hasFormat(QString::fromLatin1(kPlotSeriesDragMime))) {
    const PlotSeriesDragPayload payload = DecodePayload(
        mime->data(QString::fromLatin1(kPlotSeriesDragMime)));
    if (!payload.channel.isEmpty()) {
      payloads.push_back(payload);
    }
  }
  return payloads;
}

}  // namespace plot
}  // namespace autoviz
