/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file shortcuts_editor.hpp
 * @brief Table editor for application keyboard shortcuts.
 *
 * Embedded in @ref AppSettingsDialog. Rows come from
 * @ref DefaultShortcutDefinitions(); user edits are returned via
 * @ref shortcuts().
 *
 * @see AppUiPreferences
 * @see TranslateShortcutLabel()
 * @see AppSettingsDialog
 */

#pragma once

#include <QHash>
#include <QKeySequence>
#include <QWidget>

class QTableWidget;

namespace autoviz {

/**
 * @class ShortcutsEditorWidget
 * @brief Editable table of Autoviz shortcut ids and key sequences.
 *
 * ## Columns
 *
 * Action (translated label) | Shortcut (key sequence editor)
 *
 * @ref resetToDefaults() restores sequences from
 * @ref DefaultShortcutDefinitions().
 *
 * @see AppSettingsResult::shortcuts
 * @see FrameSession::applyShortcutPreferences()
 */
class ShortcutsEditorWidget : public QWidget {
  Q_OBJECT

 public:
  /**
   * @brief Builds an empty table and populates rows from the default catalog.
   * @param parent Parent widget (settings page).
   */
  explicit ShortcutsEditorWidget(QWidget* parent = nullptr);

  /**
   * @brief Replaces in-memory shortcuts and rebuilds the table.
   * @param shortcuts Id → sequence map (missing ids keep defaults).
   * @see shortcuts()
   */
  void setShortcuts(const QHash<QString, QKeySequence>& shortcuts);

  /**
   * @brief Reads current sequences from the table / cache.
   * @return Id → key sequence map.
   * @see setShortcuts()
   */
  QHash<QString, QKeySequence> shortcuts() const;

  /**
   * @brief Restores all rows to @ref DefaultShortcutDefinitions() sequences.
   */
  void resetToDefaults();

 private:
  /**
   * @brief Clears and rebuilds table rows from @c shortcuts_ + defaults.
   */
  void rebuildRows();

  /** Two-column shortcut table. */
  QTableWidget* table_ = nullptr;

  /** Working copy of shortcut overrides / values. */
  QHash<QString, QKeySequence> shortcuts_;
};

/**
 * @brief Returns a translated UI label for @p shortcut_id.
 *
 * Falls back to the English label from @ref DefaultShortcutDefinitions() when
 * no translation is available.
 *
 * @param shortcut_id Stable shortcut id.
 * @return Localized label for the editor / settings UI.
 */
QString TranslateShortcutLabel(const QString& shortcut_id);

}  // namespace autoviz
