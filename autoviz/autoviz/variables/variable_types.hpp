/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>
#include <QVariant>
#include <QVector>

namespace autoviz {
namespace variables {

enum class VariableType {
  kString = 0,
  kNumber = 1,
  kBoolean = 2,
  kArray = 3,
  kMap = 4,
};

struct VariableEntry {
  QString name;
  VariableType type = VariableType::kString;
  QVariant value;
};

QString VariableTypeToString(VariableType type);
VariableType VariableTypeFromString(const QString& text);

/** Serialize a variable value to a stable session string. */
QString SerializeVariableValue(VariableType type, const QVariant& value);

/** Parse session string into a QVariant; returns invalid QVariant on failure. */
QVariant DeserializeVariableValue(VariableType type, const QString& text);

bool IsValidVariableName(const QString& name);

}  // namespace variables
}  // namespace autoviz
