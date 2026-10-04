/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file preferences.hpp
 * @brief Persisted UI preferences: language, window start, HUD, shortcuts.
 *
 * Stored outside session @c .autoviz files so settings survive config switches.
 * Loaded at startup and edited via @ref AppSettingsDialog.
 *
 * @see AppSettingsDialog
 * @see FrameSession::applyUiPreferences()
 * @see FrameSession::applyShortcutPreferences()
 * @see ShortcutsEditorWidget
 */

#pragma once

#include <QHash>
#include <QKeySequence>
#include <QString>

namespace autoviz {

/**
 * @struct AppShortcutDefinition
 * @brief Built-in shortcut catalog entry (id, label, default sequence).
 */
struct AppShortcutDefinition {
  QString id;                      /**< Stable shortcut id (preferences key). */
  QString label;                   /**< UI label (may be translated later). */
  QKeySequence default_sequence;   /**< Factory-default key sequence. */
};

/**
 * @struct AppUiPreferences
 * @brief User UI preferences persisted across sessions.
 */
struct AppUiPreferences {
  /** Empty means follow system locale. */
  QString language_code;

  /** Start the main window maximized on launch. */
  bool start_maximized = true;

  /** Speed / odometer HUD on 3D viewports. */
  bool viewport_hud_visible = true;

  /** User overrides keyed by @ref AppShortcutDefinition::id. */
  QHash<QString, QKeySequence> shortcuts;
};

/**
 * @brief Returns the built-in shortcut definitions (defaults + labels).
 * @return Catalog used by @ref ShortcutsEditorWidget and preference load.
 */
QVector<AppShortcutDefinition> DefaultShortcutDefinitions();

/**
 * @brief Loads UI preferences from the platform settings store.
 * @return Preferences with defaults filled for missing keys.
 */
AppUiPreferences LoadAppUiPreferences();

/**
 * @brief Persists @p preferences to the platform settings store.
 * @param preferences Values to write.
 */
void SaveAppUiPreferences(const AppUiPreferences& preferences);

/**
 * @brief Resolves a shortcut by @p id (user override or default).
 *
 * @param preferences Loaded preferences.
 * @param id Shortcut id from @ref DefaultShortcutDefinitions().
 * @return Effective key sequence (may be empty if unbound).
 */
QKeySequence ShortcutForId(const AppUiPreferences& preferences,
                           const QString& id);

}  // namespace autoviz
