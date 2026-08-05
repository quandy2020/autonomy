/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/variables/variable_types.hpp"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>

namespace autoviz {
namespace variables {
namespace {

const QRegularExpression kVariableNamePattern(
    QStringLiteral("^[A-Za-z_][A-Za-z0-9_]*$"));

}  // namespace

QString VariableTypeToString(VariableType type) {
  switch (type) {
    case VariableType::kString:
      return QStringLiteral("string");
    case VariableType::kNumber:
      return QStringLiteral("number");
    case VariableType::kBoolean:
      return QStringLiteral("boolean");
    case VariableType::kArray:
      return QStringLiteral("array");
    case VariableType::kMap:
      return QStringLiteral("map");
  }
  return QStringLiteral("string");
}

VariableType VariableTypeFromString(const QString& text) {
  const QString normalized = text.trimmed().toLower();
  if (normalized == QStringLiteral("number")) {
    return VariableType::kNumber;
  }
  if (normalized == QStringLiteral("boolean")) {
    return VariableType::kBoolean;
  }
  if (normalized == QStringLiteral("array")) {
    return VariableType::kArray;
  }
  if (normalized == QStringLiteral("map")) {
    return VariableType::kMap;
  }
  return VariableType::kString;
}

QString SerializeVariableValue(VariableType type, const QVariant& value) {
  switch (type) {
    case VariableType::kString:
      return value.toString();
    case VariableType::kNumber:
      return QString::number(value.toDouble(), 'g', 16);
    case VariableType::kBoolean:
      return value.toBool() ? QStringLiteral("true")
                            : QStringLiteral("false");
    case VariableType::kArray: {
      if (value.canConvert<QJsonArray>()) {
        return QString::fromUtf8(
            QJsonDocument(value.toJsonArray()).toJson(QJsonDocument::Compact));
      }
      return QString::fromUtf8(
          QJsonDocument(QJsonArray::fromStringList(value.toStringList()))
              .toJson(QJsonDocument::Compact));
    }
    case VariableType::kMap: {
      if (value.canConvert<QJsonObject>()) {
        return QString::fromUtf8(
            QJsonDocument(value.toJsonObject()).toJson(QJsonDocument::Compact));
      }
      const QJsonDocument doc =
          QJsonDocument::fromJson(value.toString().toUtf8());
      if (doc.isObject()) {
        return QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
      }
      return QStringLiteral("{}");
    }
  }
  return value.toString();
}

QVariant DeserializeVariableValue(VariableType type, const QString& text) {
  switch (type) {
    case VariableType::kString:
      return text;
    case VariableType::kNumber:
      return text.toDouble();
    case VariableType::kBoolean: {
      const QString normalized = text.trimmed().toLower();
      return normalized == QStringLiteral("true") ||
             normalized == QStringLiteral("1");
    }
    case VariableType::kArray: {
      const QJsonDocument doc = QJsonDocument::fromJson(text.toUtf8());
      if (doc.isArray()) {
        return doc.array();
      }
      return QJsonArray{};
    }
    case VariableType::kMap: {
      const QJsonDocument doc = QJsonDocument::fromJson(text.toUtf8());
      if (doc.isObject()) {
        return doc.object();
      }
      return QJsonObject{};
    }
  }
  return text;
}

bool IsValidVariableName(const QString& name) {
  return kVariableNamePattern.match(name).hasMatch();
}

}  // namespace variables
}  // namespace autoviz
