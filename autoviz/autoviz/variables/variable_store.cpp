/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/variables/variable_store.hpp"

#include <algorithm>

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>

namespace autoviz {
namespace variables {
namespace {

const QRegularExpression kVariableReferencePattern(
    QStringLiteral(R"(\$([A-Za-z_][A-Za-z0-9_]*))"));

QString SubstitutionText(const VariableEntry& entry) {
  switch (entry.type) {
    case VariableType::kString:
      return entry.value.toString();
    case VariableType::kNumber:
      return QString::number(entry.value.toDouble(), 'g', 16);
    case VariableType::kBoolean:
      return entry.value.toBool() ? QStringLiteral("true")
                                  : QStringLiteral("false");
    case VariableType::kArray:
      if (entry.value.canConvert<QJsonArray>()) {
        return QString::fromUtf8(QJsonDocument(entry.value.toJsonArray())
                                     .toJson(QJsonDocument::Compact));
      }
      return entry.value.toString();
    case VariableType::kMap:
      if (entry.value.canConvert<QJsonObject>()) {
        return QString::fromUtf8(QJsonDocument(entry.value.toJsonObject())
                                     .toJson(QJsonDocument::Compact));
      }
      return entry.value.toString();
  }
  return entry.value.toString();
}

}  // namespace

VariableStore::VariableStore(QObject* parent) : QObject(parent) {}

QVector<VariableEntry> VariableStore::variables() const {
  QVector<VariableEntry> result;
  result.reserve(variables_.size());
  for (auto it = variables_.constBegin(); it != variables_.constEnd(); ++it) {
    result.push_back(it.value());
  }
  std::sort(result.begin(), result.end(),
            [](const VariableEntry& a, const VariableEntry& b) {
              return a.name.compare(b.name, Qt::CaseInsensitive) < 0;
            });
  return result;
}

std::optional<VariableEntry> VariableStore::variable(
    const QString& name) const {
  const auto it = variables_.constFind(name);
  if (it == variables_.constEnd()) {
    return std::nullopt;
  }
  return it.value();
}

bool VariableStore::setVariable(const QString& name, VariableType type,
                                const QVariant& value) {
  if (!IsValidVariableName(name)) {
    return false;
  }
  VariableEntry entry;
  entry.name = name;
  entry.type = type;
  entry.value = value;
  variables_.insert(name, entry);
  emit variableChanged(name);
  emit variablesChanged();
  return true;
}

bool VariableStore::renameVariable(const QString& old_name,
                                   const QString& new_name) {
  if (!variables_.contains(old_name) || !IsValidVariableName(new_name) ||
      variables_.contains(new_name)) {
    return false;
  }
  VariableEntry entry = variables_.take(old_name);
  entry.name = new_name;
  variables_.insert(new_name, entry);
  emit variableChanged(new_name);
  emit variablesChanged();
  return true;
}

bool VariableStore::removeVariable(const QString& name) {
  if (!variables_.remove(name)) {
    return false;
  }
  emit variablesChanged();
  return true;
}

void VariableStore::clear() {
  if (variables_.isEmpty()) {
    return;
  }
  variables_.clear();
  emit variablesChanged();
}

bool VariableStore::adjustNumber(const QString& name, double delta) {
  const auto it = variables_.find(name);
  if (it == variables_.end() || it->type != VariableType::kNumber) {
    return false;
  }
  it->value = it->value.toDouble() + delta;
  emit variableChanged(name);
  emit variablesChanged();
  return true;
}

QString VariableStore::substitute(const QString& text) const {
  if (text.isEmpty() || !text.contains(QLatin1Char('$'))) {
    return text;
  }
  QString result = text;
  QRegularExpressionMatchIterator it = kVariableReferencePattern.globalMatch(text);
  while (it.hasNext()) {
    const QRegularExpressionMatch match = it.next();
    const QString var_name = match.captured(1);
    const auto found = variables_.constFind(var_name);
    if (found == variables_.constEnd()) {
      continue;
    }
    result.replace(match.capturedStart(0), match.capturedLength(0),
                   SubstitutionText(found.value()));
  }
  return result;
}

void VariableStore::loadFromSession(
    const std::vector<common::VariablePersistConfig>& configs) {
  variables_.clear();
  for (const common::VariablePersistConfig& config : configs) {
    const QString name = QString::fromStdString(config.name);
    if (!IsValidVariableName(name)) {
      continue;
    }
    const VariableType type = VariableTypeFromString(
        QString::fromStdString(config.type));
    const QVariant value = DeserializeVariableValue(
        type, QString::fromStdString(config.value));
    VariableEntry entry;
    entry.name = name;
    entry.type = type;
    entry.value = value;
    variables_.insert(name, entry);
  }
  emit variablesChanged();
}

std::vector<common::VariablePersistConfig> VariableStore::saveToSession() const {
  std::vector<common::VariablePersistConfig> configs;
  configs.reserve(static_cast<std::size_t>(variables_.size()));
  const QVector<VariableEntry> entries = variables();
  for (const VariableEntry& entry : entries) {
    common::VariablePersistConfig config;
    config.name = entry.name.toStdString();
    config.type = VariableTypeToString(entry.type).toStdString();
    config.value =
        SerializeVariableValue(entry.type, entry.value).toStdString();
    configs.push_back(std::move(config));
  }
  return configs;
}

}  // namespace variables
}  // namespace autoviz
