/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file plot_legend_widget.hpp
 * @brief Floating legend overlay listing plot series colors, labels, and values.
 *
 * Positioned by @ref PlotPanel over @ref PlotChartWidget using
 * @ref PlotChartWidget::legendAnchorRect().
 *
 * @see PlotPanel
 * @see PlotValueRow
 */

#pragma once

#include <QColor>
#include <QFrame>
#include <QVector>

#include <vector>

#include "autoviz/ui/plot/plot_types.hpp"

namespace autoviz {
namespace plot {

struct PlotSeriesRuntime;

/**
 * @class PlotLegendWidget
 * @brief Compact legend frame showing series swatches and optional live values.
 *
 * When @ref setShowValues() is true, rows from @ref setValueRows() (typically
 * hover/playback samples) appear beside each label.
 */
class PlotLegendWidget : public QFrame {
  Q_OBJECT

 public:
  /**
   * @brief Constructs an empty legend frame.
   *
   * @param parent Qt parent (usually the plot panel, not the chart).
   */
  explicit PlotLegendWidget(QWidget* parent = nullptr);

  /**
   * @brief Replaces the series list used for labels and colors.
   *
   * @param series Non-owning pointers to runtimes (typically enabled only).
   */
  void setSeries(const std::vector<const PlotSeriesRuntime*>& series);

  /**
   * @brief Enables or disables the live value column.
   *
   * @param show @c true to paint @ref PlotValueRow text.
   */
  void setShowValues(bool show);

  /**
   * @brief Updates the per-series value strings (hover / playhead).
   *
   * @param rows Value rows aligned with the series list where possible.
   */
  void setValueRows(const QVector<PlotValueRow>& rows);

  /**
   * @brief Preferred size for layout / positioning over the chart.
   *
   * @return Size hint based on series count and value column.
   */
  QSize preferredSize() const;

 protected:
  /**
   * @brief Paints swatches, labels, and optional values.
   *
   * @param event Paint event.
   */
  void paintEvent(QPaintEvent* event) override;

  /**
   * @brief Qt size hint forwarding @ref preferredSize().
   *
   * @return Preferred size.
   */
  QSize sizeHint() const override;

 private:
  /** Non-owning series list for labels/colors. */
  std::vector<const PlotSeriesRuntime*> series_;

  /** Whether to show the value column. */
  bool show_values_ = true;

  /** Latest value rows from the chart / panel. */
  QVector<PlotValueRow> value_rows_;
};

}  // namespace plot
}  // namespace autoviz
