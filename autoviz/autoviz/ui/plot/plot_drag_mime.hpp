/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file plot_drag_mime.hpp
 * @brief MIME payload helpers for dragging channel+field paths onto Plot panels.
 *
 * Topics tree and settings editors encode @ref PlotSeriesDragPayload into
 * custom MIME types recognized by @ref PlotPanel drop handlers.
 *
 * @see PlotPanel
 * @see message_field_tree.hpp
 */

#pragma once

#include <QString>
#include <QVector>

class QMimeData;

namespace autoviz {
namespace plot {

/**
 * @brief MIME type for a single plot series drag (one channel + field path).
 */
constexpr char kPlotSeriesDragMime[] = "application/x-autoviz-plot-series";

/**
 * @brief MIME type for a multi-series drag (list of channel + field paths).
 */
constexpr char kPlotSeriesListDragMime[] = "application/x-autoviz-plot-series-list";

/**
 * @struct PlotSeriesDragPayload
 * @brief One draggable series identity: channel and Y field path.
 */
struct PlotSeriesDragPayload {
  QString channel;     /**< Source channel name. */
  QString field_path;  /**< Protobuf / message field path for Y. */
};

/**
 * @brief Builds @c QMimeData carrying a single series payload.
 *
 * @param payload Channel + field path.
 * @return Heap-allocated mime data (caller / Qt drag takes ownership).
 * @see ReadPlotSeriesDragPayload()
 */
QMimeData* MakePlotSeriesDragPayload(const PlotSeriesDragPayload& payload);

/**
 * @brief Builds @c QMimeData carrying multiple series payloads.
 *
 * @param payloads Ordered list of channel + field paths.
 * @return Heap-allocated mime data.
 * @see ReadPlotSeriesDragPayloads()
 */
QMimeData* MakePlotSeriesListDragPayload(
    const QVector<PlotSeriesDragPayload>& payloads);

/**
 * @brief Reads a single-series drag payload from mime data.
 *
 * @param mime Source mime (may also contain the list type).
 * @param out Out-parameter for the first / only payload.
 * @return @c true when a payload was decoded into @p out.
 */
bool ReadPlotSeriesDragPayload(const QMimeData* mime, PlotSeriesDragPayload* out);

/**
 * @brief Reads all series payloads from single- or list-typed mime data.
 *
 * @param mime Source mime.
 * @return Zero or more payloads (empty if unrecognized).
 */
QVector<PlotSeriesDragPayload> ReadPlotSeriesDragPayloads(const QMimeData* mime);

}  // namespace plot
}  // namespace autoviz
