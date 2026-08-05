/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/state_transitions/state_mapping.hpp"

#include <QCryptographicHash>

namespace autoviz {
namespace state_transitions {
namespace {

QString StateKeyFromValue(const indicator::IndicatorFieldValue& value) {
  switch (value.kind) {
    case indicator::IndicatorValueKind::kBoolean:
      return value.boolean ? QStringLiteral("true") : QStringLiteral("false");
    case indicator::IndicatorValueKind::kString:
      return value.text;
    case indicator::IndicatorValueKind::kNumeric:
    default:
      return QString::number(value.number, 'g', 12);
  }
}

QString DefaultLabelFromValue(const indicator::IndicatorFieldValue& value) {
  switch (value.kind) {
    case indicator::IndicatorValueKind::kBoolean:
      return value.boolean ? QStringLiteral("true") : QStringLiteral("false");
    case indicator::IndicatorValueKind::kString:
      return value.text.isEmpty() ? QStringLiteral("(empty)") : value.text;
    case indicator::IndicatorValueKind::kNumeric:
    default:
      return QString::number(value.number, 'g', 8);
  }
}

bool RuleMatches(const StateMappingRule& rule,
                 const indicator::IndicatorFieldValue& value,
                 const QString& state_key) {
  if (rule.kind == StateMappingKind::kExact) {
    return state_key == rule.match_value;
  }
  if (value.kind != indicator::IndicatorValueKind::kNumeric) {
    return false;
  }
  return value.number >= rule.range_min && value.number <= rule.range_max;
}

}  // namespace

QColor DefaultColorForStateKey(const QString& state_key) {
  const QByteArray digest =
      QCryptographicHash::hash(state_key.toUtf8(), QCryptographicHash::Sha1);
  const int hue = static_cast<unsigned char>(digest.at(0)) * 360 / 255;
  QColor color;
  color.setHsv(hue, 160, 210);
  return color;
}

ResolvedStateDisplay ResolveStateDisplay(
    const indicator::IndicatorFieldValue& value,
    const QVector<StateMappingRule>& mappings) {
  ResolvedStateDisplay display;
  display.state_key = StateKeyFromValue(value);
  display.label = DefaultLabelFromValue(value);
  display.color = DefaultColorForStateKey(display.state_key);

  for (const StateMappingRule& rule : mappings) {
    if (!RuleMatches(rule, value, display.state_key)) {
      continue;
    }
    if (!rule.label.isEmpty()) {
      display.label = rule.label;
    }
    if (rule.color.isValid()) {
      display.color = rule.color;
    }
    break;
  }
  return display;
}

bool AppendInferredMappingRuleIfNeeded(
    QVector<StateMappingRule>& mappings,
    const indicator::IndicatorFieldValue& value) {
  const QString state_key = StateKeyFromValue(value);
  for (const StateMappingRule& rule : mappings) {
    if (RuleMatches(rule, value, state_key)) {
      return false;
    }
  }
  StateMappingRule rule;
  rule.kind = StateMappingKind::kExact;
  rule.match_value = state_key;
  rule.label = DefaultLabelFromValue(value);
  rule.color = DefaultColorForStateKey(state_key);
  mappings.push_back(rule);
  return true;
}

}  // namespace state_transitions
}  // namespace autoviz
