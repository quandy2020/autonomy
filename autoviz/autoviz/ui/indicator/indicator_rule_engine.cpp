/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/indicator/indicator_rule_engine.hpp"

#include <QtMath>

namespace autoviz {
namespace indicator {
namespace {

bool ParseBool(const QString& text, bool* out) {
  if (out == nullptr) {
    return false;
  }
  const QString normalized = text.trimmed().toLower();
  if (normalized == QLatin1String("true") || normalized == QLatin1String("1") ||
      normalized == QLatin1String("yes") || normalized == QLatin1String("on")) {
    *out = true;
    return true;
  }
  if (normalized == QLatin1String("false") || normalized == QLatin1String("0") ||
      normalized == QLatin1String("no") || normalized == QLatin1String("off")) {
    *out = false;
    return true;
  }
  return false;
}

bool ParseDouble(const QString& text, double* out) {
  if (out == nullptr || text.trimmed().isEmpty()) {
    return false;
  }
  bool ok = false;
  const double parsed = text.trimmed().toDouble(&ok);
  if (!ok || !std::isfinite(parsed)) {
    return false;
  }
  *out = parsed;
  return true;
}

bool CompareNumeric(double lhs, double rhs, IndicatorComparison comparison) {
  switch (comparison) {
    case IndicatorComparison::kEqual:
      return qFuzzyCompare(1.0 + lhs, 1.0 + rhs);
    case IndicatorComparison::kNotEqual:
      return !qFuzzyCompare(1.0 + lhs, 1.0 + rhs);
    case IndicatorComparison::kGreaterThan:
      return lhs > rhs;
    case IndicatorComparison::kGreaterThanOrEqual:
      return lhs >= rhs || qFuzzyCompare(1.0 + lhs, 1.0 + rhs);
    case IndicatorComparison::kLessThan:
      return lhs < rhs;
    case IndicatorComparison::kLessThanOrEqual:
      return lhs <= rhs || qFuzzyCompare(1.0 + lhs, 1.0 + rhs);
  }
  return false;
}

bool CompareString(const QString& lhs, const QString& rhs,
                   IndicatorComparison comparison) {
  switch (comparison) {
    case IndicatorComparison::kEqual:
      return lhs == rhs;
    case IndicatorComparison::kNotEqual:
      return lhs != rhs;
    default:
      return false;
  }
}

bool CompareBoolean(bool lhs, bool rhs, IndicatorComparison comparison) {
  switch (comparison) {
    case IndicatorComparison::kEqual:
      return lhs == rhs;
    case IndicatorComparison::kNotEqual:
      return lhs != rhs;
    default:
      return false;
  }
}

bool RuleMatches(const IndicatorFieldValue& value, const IndicatorRule& rule) {
  switch (value.kind) {
    case IndicatorValueKind::kNumeric: {
      double threshold = 0.0;
      if (!ParseDouble(rule.compare_with, &threshold)) {
        if (!value.text.isEmpty() &&
            CompareString(value.text, rule.compare_with.trimmed(), rule.comparison)) {
          return true;
        }
        return false;
      }
      return CompareNumeric(value.number, threshold, rule.comparison);
    }
    case IndicatorValueKind::kString:
      return CompareString(value.text, rule.compare_with.trimmed(), rule.comparison);
    case IndicatorValueKind::kBoolean: {
      bool threshold = false;
      if (!ParseBool(rule.compare_with, &threshold)) {
        return false;
      }
      return CompareBoolean(value.boolean, threshold, rule.comparison);
    }
  }
  return false;
}

}  // namespace

IndicatorMatchResult IndicatorRuleEngine::Evaluate(
    const IndicatorFieldValue& value, const QVector<IndicatorRule>& rules) {
  IndicatorMatchResult result;
  result.raw_value_text = FormatIndicatorFieldValue(value);

  for (const IndicatorRule& rule : rules) {
    if (!RuleMatches(value, rule)) {
      continue;
    }
    result.matched = true;
    result.color = rule.color.isValid() ? rule.color : QColor(120, 120, 130);
    result.label = rule.label.trimmed().isEmpty() ? result.raw_value_text : rule.label;
    return result;
  }

  result.label = result.raw_value_text;
  return result;
}

}  // namespace indicator
}  // namespace autoviz
