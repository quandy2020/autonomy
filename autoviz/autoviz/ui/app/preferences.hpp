/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file preferences.hpp
 * @brief Persisted UI preferences: language, theme, window start, HUD, shortcuts.
 *
 * Stored outside session @c .autoviz files so settings survive config switches.
 * Loaded at startup and edited via @ref AppSettingsDialog.
 *
 * @see AppSettingsDialog
 * @see FrameSession::applyUiPreferences()
 * @see ShortcutsEditorWidget
 */

#pragma once

#include <QHash>
#include <QKeySequence>
#include <QString>

#include "autoviz/ui/theme/tokens.hpp"

class QApplication;

namespace autoviz {

/**
 * @struct AppShortcutDefinition
 * @brief Built-in shortcut catalog entry (id, label, default sequence).
 */
struct AppShortcutDefinition {
  QString id;
  QString label;
  QKeySequence default_sequence;
};

/**
 * @struct AppUiPreferences
 * @brief User UI preferences persisted across sessions.
 */
struct AppUiPreferences {
  /** Empty means follow system locale. */
  QString language_code;

  /**
   * App chrome theme: @c "system", @c "light", or @c "dark".
   * Legacy @c "liquid" is normalized to @c "light".
   */
  QString theme_mode = QStringLiteral("dark");

  /**
   * Optional accent override as @c #RRGGBB.
   * Empty means the theme's default accent.
   */
  QString accent_color;

  bool start_maximized = true;
  bool viewport_hud_visible = false;
  QHash<QString, QKeySequence> shortcuts;
};

QVector<AppShortcutDefinition> DefaultShortcutDefinitions();
AppUiPreferences LoadAppUiPreferences();
void SaveAppUiPreferences(const AppUiPreferences& preferences);

/**
 * @brief Maps a stored theme id to @ref theme::ShellAppearance.
 */
theme::ShellAppearance ShellAppearanceFromThemeMode(const QString& theme_mode);

void ApplyUiThemeMode(const QString& theme_mode, QApplication& app);
void ApplyUiTheme(const QString& theme_mode, const QString& accent_hex,
                  QApplication& app);

QKeySequence ShortcutForId(const AppUiPreferences& preferences,
                           const QString& id);

}  // namespace autoviz
