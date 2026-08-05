/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include "autoviz/ui/gauge/gauge_types.hpp"

namespace autoviz {
namespace gauge {

class GaugeViewWidget : public QWidget {
  Q_OBJECT

 public:
  explicit GaugeViewWidget(QWidget* parent = nullptr);

  void setConfig(const GaugePanelConfig& config);
  void setValue(double value, bool has_value);
  void setErrorText(const QString& text);

 protected:
  void paintEvent(QPaintEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;

 private:
  QColor colorForNormalizedValue(double t) const;
  double normalizedValue() const;
  int startAngleSixteenths() const;
  int spanAngleSixteenths() const;
  int valueSpanSixteenths() const;

  GaugePanelConfig config_;
  double value_ = 0.0;
  bool has_value_ = false;
  QString error_text_;
};

}  // namespace gauge
}  // namespace autoviz
