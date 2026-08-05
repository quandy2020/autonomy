/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/indicator/indicator_types.hpp"

namespace autoviz {
namespace indicator {

IndicatorPanelConfig DefaultIndicatorPanelConfig() {
  IndicatorPanelConfig config;
  IndicatorRule low;
  low.comparison = IndicatorComparison::kLessThan;
  low.compare_with = QStringLiteral("11.5");
  low.color = QColor(230, 180, 40);
  low.label = QStringLiteral("Low");

  IndicatorRule critical;
  critical.comparison = IndicatorComparison::kLessThan;
  critical.compare_with = QStringLiteral("10.5");
  critical.color = QColor(220, 70, 70);
  critical.label = QStringLiteral("Critical");

  IndicatorRule normal;
  normal.comparison = IndicatorComparison::kGreaterThanOrEqual;
  normal.compare_with = QStringLiteral("11.5");
  normal.color = QColor(70, 170, 95);
  normal.label = QStringLiteral("Normal");

  config.rules = {critical, low, normal};
  return config;
}

QString IndicatorStyleLabel(IndicatorStyle style) {
  switch (style) {
    case IndicatorStyle::kBulb:
      return QStringLiteral("Bulb");
    case IndicatorStyle::kBackground:
      return QStringLiteral("Background");
  }
  return {};
}

QString IndicatorComparisonLabel(IndicatorComparison comparison) {
  switch (comparison) {
    case IndicatorComparison::kEqual:
      return QStringLiteral("Equal to");
    case IndicatorComparison::kNotEqual:
      return QStringLiteral("Not equal to");
    case IndicatorComparison::kGreaterThan:
      return QStringLiteral("Greater than");
    case IndicatorComparison::kGreaterThanOrEqual:
      return QStringLiteral("Greater than or equal to");
    case IndicatorComparison::kLessThan:
      return QStringLiteral("Less than");
    case IndicatorComparison::kLessThanOrEqual:
      return QStringLiteral("Less than or equal to");
  }
  return {};
}

}  // namespace indicator
}  // namespace autoviz
