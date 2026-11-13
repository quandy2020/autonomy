/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/app/preferences.hpp"

#include <QApplication>
#include <QColor>
#include <QSettings>

#include "autoviz/ui/theme/application.hpp"
#include "autoviz/ui/theme/glass.hpp"

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
      {QStringLiteral("tools.reset"), QStringLiteral("Reset active tool"),
       QKeySequence(Qt::Key_Escape)},
  };
}

AppUiPreferences LoadAppUiPreferences() {
  AppUiPreferences preferences;
  QSettings settings;
  preferences.language_code = settings.value(QStringLiteral("ui/language")).toString();
  preferences.theme_mode =
      settings.value(QStringLiteral("ui/theme_mode"), QStringLiteral("light"))
          .toString();
  if (preferences.theme_mode != QLatin1String("system") &&
      preferences.theme_mode != QLatin1String("light") &&
      preferences.theme_mode != QLatin1String("dark") &&
      preferences.theme_mode != QLatin1String("liquid")) {
    preferences.theme_mode = QStringLiteral("light");
  }
  preferences.accent_color =
      settings.value(QStringLiteral("ui/accent_color")).toString().trimmed();
  if (!preferences.accent_color.isEmpty()) {
    const QColor parsed(preferences.accent_color);
    if (!parsed.isValid()) {
      preferences.accent_color.clear();
    } else {
      preferences.accent_color = parsed.name(QColor::HexRgb).toUpper();
    }
  }
  preferences.start_maximized =
      settings.value(QStringLiteral("ui/start_maximized"), true).toBool();
  preferences.viewport_hud_visible =
      settings.value(QStringLiteral("ui/viewport_hud_visible"), true).toBool();

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
  settings.setValue(QStringLiteral("ui/theme_mode"), preferences.theme_mode);
  settings.setValue(QStringLiteral("ui/accent_color"), preferences.accent_color);
  settings.setValue(QStringLiteral("ui/start_maximized"), preferences.start_maximized);
  settings.setValue(QStringLiteral("ui/viewport_hud_visible"),
                    preferences.viewport_hud_visible);
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

glass::ShellAppearance ShellAppearanceFromThemeMode(const QString& theme_mode) {
  if (theme_mode == QLatin1String("dark")) {
    return glass::ShellAppearance::kDark;
  }
  if (theme_mode == QLatin1String("system")) {
    return glass::ShellAppearance::kSystem;
  }
  if (theme_mode == QLatin1String("liquid")) {
    return glass::ShellAppearance::kLiquid;
  }
  return glass::ShellAppearance::kLight;
}

void ApplyUiThemeMode(const QString& theme_mode, QApplication& app) {
  ApplyUiTheme(theme_mode, LoadAppUiPreferences().accent_color, app);
}

void ApplyUiTheme(const QString& theme_mode, const QString& accent_hex,
                  QApplication& app) {
  glass::SetShellAppearance(ShellAppearanceFromThemeMode(theme_mode));
  const QColor accent(accent_hex);
  glass::SetShellAccentOverride(accent.isValid() ? accent : QColor());
  ApplyAppTheme(app);
  RefreshAppThemeChrome(app);
}

}  // namespace autoviz
