/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <optional>
#include <vector>

#include <QHash>
#include <QObject>
#include <QString>

#include "autoviz/common/session_config.hpp"
#include "autoviz/variables/variable_types.hpp"

namespace autoviz {
namespace variables {

/** Global layout variables referenced as $name in message paths. */
class VariableStore : public QObject {
  Q_OBJECT

 public:
  explicit VariableStore(QObject* parent = nullptr);

  QVector<VariableEntry> variables() const;
  std::optional<VariableEntry> variable(const QString& name) const;

  bool setVariable(const QString& name, VariableType type, const QVariant& value);
  bool renameVariable(const QString& old_name, const QString& new_name);
  bool removeVariable(const QString& name);
  void clear();

  /** Increment/decrement a numeric variable (Foxglove arrow-key behavior). */
  bool adjustNumber(const QString& name, double delta);

  /** Replace $identifier tokens in message paths and filters. */
  QString substitute(const QString& text) const;

  void loadFromSession(const std::vector<common::VariablePersistConfig>& configs);
  std::vector<common::VariablePersistConfig> saveToSession() const;

 signals:
  void variableChanged(const QString& name);
  void variablesChanged();

 private:
  QHash<QString, VariableEntry> variables_;
};

}  // namespace variables
}  // namespace autoviz
