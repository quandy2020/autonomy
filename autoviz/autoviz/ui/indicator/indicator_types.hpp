/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QString>
#include <QVector>

namespace autoviz {
namespace indicator {

enum class IndicatorStyle {
  kBulb = 0,
  kBackground = 1,
};

enum class IndicatorComparison {
  kEqual = 0,
  kNotEqual = 1,
  kGreaterThan = 2,
  kGreaterThanOrEqual = 3,
  kLessThan = 4,
  kLessThanOrEqual = 5,
};

struct IndicatorRule {
  IndicatorComparison comparison = IndicatorComparison::kEqual;
  QString compare_with;
  QColor color = QColor(120, 120, 130);
  QString label;
};

struct IndicatorPanelConfig {
  QString title;
  QString channel;
  QString field_path;
  IndicatorStyle style = IndicatorStyle::kBulb;
  QVector<IndicatorRule> rules;
};

IndicatorPanelConfig DefaultIndicatorPanelConfig();
QString IndicatorStyleLabel(IndicatorStyle style);
QString IndicatorComparisonLabel(IndicatorComparison comparison);

}  // namespace indicator
}  // namespace autoviz
