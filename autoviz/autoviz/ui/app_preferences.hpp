/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QHash>
#include <QKeySequence>
#include <QString>

namespace autoviz {

struct AppShortcutDefinition {
  QString id;
  QString label;
  QKeySequence default_sequence;
};

struct AppUiPreferences {
  /** Empty means follow system locale. */
  QString language_code;
  bool start_maximized = true;
  QHash<QString, QKeySequence> shortcuts;
};

QVector<AppShortcutDefinition> DefaultShortcutDefinitions();
AppUiPreferences LoadAppUiPreferences();
void SaveAppUiPreferences(const AppUiPreferences& preferences);
QKeySequence ShortcutForId(const AppUiPreferences& preferences, const QString& id);

}  // namespace autoviz
