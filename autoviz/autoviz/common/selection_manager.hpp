/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file selection_manager.hpp
 * @brief Central selection state for the 3D viewport (RViz SelectionManager subset).
 *
 * Owns the current @ref SelectionEntry list and notifies listeners when the
 * set changes or when a “focus camera on selection” request is made.
 *
 * @see SelectionEntry
 * @see SelectionHandler
 * @see VisualizationManager::setSelectionChangedCallback
 */

#pragma once

#include <functional>
#include <vector>

#include <QVector3D>

#include "autoviz/common/selection.hpp"

namespace autoviz {
namespace common {

/**
 * @class SelectionManager
 * @brief Stores and mutates the global selection set.
 *
 * ## Modes
 *
 * @ref SelectMode controls how @ref select() merges with the existing set
 * (replace / add / remove), matching RViz selection modifiers.
 */
class SelectionManager {
 public:
  /**
   * @enum SelectMode
   * @brief How a new selection interacts with the current set.
   */
  enum class SelectMode {
    kReplace,  /**< Replace the entire selection with the new entry. */
    kAdd,      /**< Add the entry if not already present. */
    kRemove,   /**< Remove a matching entry if present. */
  };

  /**
   * @brief Returns the current selection (read-only).
   *
   * @return Reference to the in-memory entry list.
   */
  const std::vector<SelectionEntry>& selection() const { return selection_; }

  /**
   * @brief Replaces the entire selection and fires the changed callback.
   *
   * @param entries New selection set (moved).
   */
  void setSelection(std::vector<SelectionEntry> entries);

  /**
   * @brief Applies one entry according to @p mode.
   *
   * @param entry Entry to add, replace-with, or remove.
   * @param mode Merge strategy.
   * @see SelectMode
   */
  void select(const SelectionEntry& entry, SelectMode mode);

  /**
   * @brief Clears all selected entries and notifies listeners.
   */
  void clear();

  /**
   * @brief Registers a callback invoked whenever the selection set changes.
   *
   * @param callback Receives the new entry list (by const ref semantics via
   *        copy into the lambda as needed).
   */
  void setChangedCallback(
      std::function<void(const std::vector<SelectionEntry>&)> callback);

  /**
   * @brief Registers a callback for “focus camera on selection” requests.
   *
   * @param callback Receives a world-space target point.
   * @see focusOnSelection()
   */
  void setFocusCallback(std::function<void(const QVector3D& target)> callback);

  /**
   * @brief Invokes the focus callback with a representative selection point.
   *
   * No-op if the selection is empty or no focus callback is set.
   */
  void focusOnSelection();

 private:
  /**
   * @brief Equality helper used when adding/removing entries.
   *
   * @param a First entry.
   * @param b Second entry.
   * @return @c true if both identify the same selectable object.
   */
  static bool SameEntry(const SelectionEntry& a, const SelectionEntry& b);

  /** Current selection set. */
  std::vector<SelectionEntry> selection_;

  /** Fired after any mutation of @c selection_. */
  std::function<void(const std::vector<SelectionEntry>&)> changed_callback_;

  /** Fired by @ref focusOnSelection(). */
  std::function<void(const QVector3D& target)> focus_callback_;
};

}  // namespace common
}  // namespace autoviz
