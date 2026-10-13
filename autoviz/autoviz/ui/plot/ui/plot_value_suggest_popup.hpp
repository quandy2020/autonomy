/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file plot_value_suggest_popup.hpp
 * @brief Non-modal channel/field suggestion menu for plot Value line edits.
 *
 * Anchored under a @c QLineEdit; populated by @ref PlotValuePathSuggestions().
 * Selection is delivered via @c on_selected.
 *
 * @see plot_path_utils.hpp
 * @see PlotSettingsWidget
 */

#pragma once

#include <functional>

#include <QObject>
#include <QPointer>
#include <QStringList>

class QLineEdit;
class QMenu;

namespace autoviz {
namespace plot {

/**
 * @class PlotValueSuggestList
 * @brief Non-modal channel/field suggestion menu anchored to a plot Value line
 *        edit.
 *
 * Uses a @c QMenu under the anchor edit. While @c selecting_ is true, text
 * changes from applying a selection do not immediately re-show the menu.
 *
 * @note Not a @c QWidget subclass; owns the menu as a child @c QObject.
 */
class PlotValueSuggestList : public QObject {
 public:
  /**
   * @brief Binds the suggestion menu to @p anchor.
   *
   * @param anchor Line edit that owns the caret / typed prefix.
   * @param parent Qt parent object.
   */
  explicit PlotValueSuggestList(QLineEdit* anchor, QObject* parent = nullptr);

  /**
   * @brief Shows (or refreshes) the menu with @p suggestions under the anchor.
   *
   * @param suggestions Entries from @ref PlotValuePathSuggestions().
   */
  void showSuggestions(const QStringList& suggestions);

  /**
   * @brief Hides the suggestion menu if visible.
   */
  void hideSuggestions();

  /**
   * @brief Whether the suggestion menu is currently visible.
   *
   * @return @c true when the menu is shown.
   */
  bool isVisible() const;

  /**
   * @brief Callback invoked when the user picks a suggestion.
   *
   * Signature: @c void(const QString& selected_value).
   */
  std::function<void(const QString&)> on_selected;

 private:
  /**
   * @brief Invokes @c on_selected and clears the selecting guard.
   *
   * @param value Chosen suggestion string.
   */
  void emitSelection(const QString& value);

  /** Weak pointer to the Value line edit. */
  QPointer<QLineEdit> anchor_;

  /** Popup menu listing suggestions. */
  QMenu* menu_ = nullptr;

  /** Re-entrancy guard while applying a selection into the edit. */
  bool selecting_ = false;
};

}  // namespace plot
}  // namespace autoviz
