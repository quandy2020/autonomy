/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/app_preferences.hpp"

#include <QSettings>

namespace autoviz {

QVector<AppShortcutDefinition> DefaultShortcutDefinitions() {
  return {
      {QStringLiteral("file.open"), QStringLiteral("Open config"), QKeySequence::Open},
      {QStringLiteral("file.open_record"), QStringLiteral("Open record"),
       QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_O)},
      {QStringLiteral("file.save"), QStringLiteral("Save config"), QKeySequence::Save},
      {QStringLiteral("file.save_as"), QStringLiteral("Save config as"),
       QKeySequence::SaveAs},
      {QStringLiteral("file.quit"), QStringLiteral("Quit"), QKeySequence::Quit},
      {QStringLiteral("panels.fullscreen"), QStringLiteral("Toggle fullscreen"),
       QKeySequence(Qt::Key_F11)},
      {QStringLiteral("tools.reset"), QStringLiteral("Reset active tool"),
       QKeySequence(Qt::Key_Escape)},
  };
}

AppUiPreferences LoadAppUiPreferences() {
  AppUiPreferences preferences;
  QSettings settings;
  preferences.language_code = settings.value(QStringLiteral("ui/language")).toString();
  preferences.start_maximized =
      settings.value(QStringLiteral("ui/start_maximized"), true).toBool();

  settings.beginGroup(QStringLiteral("ui/shortcuts"));
  for (const AppShortcutDefinition& definition : DefaultShortcutDefinitions()) {
    const QVariant stored = settings.value(definition.id);
    if (stored.isValid()) {
      preferences.shortcuts.insert(definition.id,
                                   QKeySequence(stored.toString()));
    } else {
      preferences.shortcuts.insert(definition.id, definition.default_sequence);
    }
  }
  settings.endGroup();
  return preferences;
}

void SaveAppUiPreferences(const AppUiPreferences& preferences) {
  QSettings settings;
  settings.setValue(QStringLiteral("ui/language"), preferences.language_code);
  settings.setValue(QStringLiteral("ui/start_maximized"), preferences.start_maximized);
  settings.beginGroup(QStringLiteral("ui/shortcuts"));
  for (auto it = preferences.shortcuts.constBegin();
       it != preferences.shortcuts.constEnd(); ++it) {
    settings.setValue(it.key(), it.value().toString(QKeySequence::PortableText));
  }
  settings.endGroup();
}

QKeySequence ShortcutForId(const AppUiPreferences& preferences,
                           const QString& id) {
  return preferences.shortcuts.value(id);
}

}  // namespace autoviz
