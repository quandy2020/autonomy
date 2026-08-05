/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QFrame>
#include <QVector>

#include <vector>

#include "autoviz/ui/plot/plot_types.hpp"

namespace autoviz {
namespace plot {

struct PlotSeriesRuntime;

class PlotLegendWidget : public QFrame {
  Q_OBJECT

 public:
  explicit PlotLegendWidget(QWidget* parent = nullptr);

  void setSeries(const std::vector<const PlotSeriesRuntime*>& series);
  void setShowValues(bool show);
  void setValueRows(const QVector<PlotValueRow>& rows);
  QSize preferredSize() const;

 protected:
  void paintEvent(QPaintEvent* event) override;
  QSize sizeHint() const override;

 private:
  std::vector<const PlotSeriesRuntime*> series_;
  bool show_values_ = true;
  QVector<PlotValueRow> value_rows_;
};

}  // namespace plot
}  // namespace autoviz
