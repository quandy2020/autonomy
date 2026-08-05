/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include "autoviz/ui/indicator/indicator_rule_engine.hpp"
#include "autoviz/ui/indicator/indicator_types.hpp"

namespace autoviz {
namespace indicator {

class IndicatorViewWidget : public QWidget {
  Q_OBJECT

 public:
  explicit IndicatorViewWidget(QWidget* parent = nullptr);

  void setConfig(const IndicatorPanelConfig& config);
  void setMatchResult(const IndicatorMatchResult& result, bool has_value);
  void setErrorText(const QString& text);

 protected:
  void paintEvent(QPaintEvent* event) override;

 private:
  void paintBulb(QPainter* painter, const QRect& area);
  void paintBackground(QPainter* painter, const QRect& area);

  IndicatorPanelConfig config_;
  IndicatorMatchResult result_;
  bool has_value_ = false;
  QString error_text_;
};

}  // namespace indicator
}  // namespace autoviz
