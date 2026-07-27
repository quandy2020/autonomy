/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <functional>
#include <vector>

#include <QVector3D>

#include "autoviz/common/selection.hpp"

namespace autoviz {
namespace common {

/** rviz_common::SelectionManager (subset) — central selection state. */
class SelectionManager {
 public:
  enum class SelectMode {
    kReplace,
    kAdd,
    kRemove,
  };

  const std::vector<SelectionEntry>& selection() const { return selection_; }

  void setSelection(std::vector<SelectionEntry> entries);
  void select(const SelectionEntry& entry, SelectMode mode);
  void clear();

  void setChangedCallback(
      std::function<void(const std::vector<SelectionEntry>&)> callback);
  void setFocusCallback(std::function<void(const QVector3D& target)> callback);

  void focusOnSelection();

 private:
  static bool SameEntry(const SelectionEntry& a, const SelectionEntry& b);

  std::vector<SelectionEntry> selection_;
  std::function<void(const std::vector<SelectionEntry>&)> changed_callback_;
  std::function<void(const QVector3D& target)> focus_callback_;
};

}  // namespace common
}  // namespace autoviz
