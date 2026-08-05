/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QString>
#include <QVector>

#include "autoviz/ui/indicator/indicator_field_extractor.hpp"
#include "autoviz/ui/indicator/indicator_types.hpp"

namespace autoviz {
namespace indicator {

struct IndicatorMatchResult {
  bool matched = false;
  QColor color = QColor(120, 120, 130);
  QString label;
  QString raw_value_text;

  bool operator==(const IndicatorMatchResult& other) const {
    return matched == other.matched && color == other.color && label == other.label &&
           raw_value_text == other.raw_value_text;
  }
};

/** Evaluates rules top-to-bottom; first matching rule wins. */
class IndicatorRuleEngine {
 public:
  static IndicatorMatchResult Evaluate(const IndicatorFieldValue& value,
                                       const QVector<IndicatorRule>& rules);
};

}  // namespace indicator
}  // namespace autoviz
