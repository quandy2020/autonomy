/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QString>

#include <vector>

#include "autoviz/ui/indicator/indicator_field_extractor.hpp"
#include "autoviz/ui/state_transitions/state_transition_types.hpp"

namespace autoviz {
namespace state_transitions {

struct ResolvedStateDisplay {
  QString state_key;
  QString label;
  QColor color;
};

/** Build a stable key and human-readable label from a field value. */
ResolvedStateDisplay ResolveStateDisplay(
    const indicator::IndicatorFieldValue& value,
    const QVector<StateMappingRule>& mappings);

QColor DefaultColorForStateKey(const QString& state_key);

bool AppendInferredMappingRuleIfNeeded(
    QVector<StateMappingRule>& mappings,
    const indicator::IndicatorFieldValue& value);

}  // namespace state_transitions
}  // namespace autoviz
