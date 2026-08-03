/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/selection_manager.hpp"

#include <algorithm>
#include <cmath>

namespace autoviz {
namespace common {
namespace {

constexpr float kSelectionEpsilon = 1e-3f;

}  // namespace

bool SelectionManager::SameEntry(const SelectionEntry& a,
                                 const SelectionEntry& b) {
  return (a.position - b.position()).lengthSquared() <=
         kSelectionEpsilon * kSelectionEpsilon;
}

void SelectionManager::setChangedCallback(
    std::function<void(const std::vector<SelectionEntry>&)> callback) {
  changed_callback_ = std::move(callback);
}

void SelectionManager::setFocusCallback(
    std::function<void(const QVector3D& target)> callback) {
  focus_callback_ = std::move(callback);
}

void SelectionManager::setSelection(std::vector<SelectionEntry> entries) {
  selection_ = std::move(entries);
  if (changed_callback_) {
    changed_callback_(selection_);
  }
}

void SelectionManager::select(const SelectionEntry& entry, SelectMode mode) {
  switch (mode) {
    case SelectMode::kReplace:
      selection_ = {entry};
      break;
    case SelectMode::kAdd: {
      const auto existing = std::find_if(
          selection_.begin(), selection_.end(),
          [&entry](const SelectionEntry& item) { return SameEntry(item, entry); });
      if (existing == selection_.end()) {
        selection_.push_back(entry);
      }
      break;
    }
    case SelectMode::kRemove: {
      selection_.erase(
          std::remove_if(selection_.begin(), selection_.end(),
                         [&entry](const SelectionEntry& item) {
                           return SameEntry(item, entry);
                         }),
          selection_.end());
      break;
    }
  }
  if (changed_callback_) {
    changed_callback_(selection_);
  }
}

void SelectionManager::clear() {
  if (selection_.empty()) {
    return;
  }
  selection_.clear();
  if (changed_callback_) {
    changed_callback_(selection_);
  }
}

void SelectionManager::focusOnSelection() {
  if (selection_.empty() || !focus_callback_) {
    return;
  }
  if (selection_.size() == 1) {
    focus_callback_(selection_.front().position());
    return;
  }
  QVector3D center(0.f, 0.f, 0.f);
  for (const auto& entry : selection_) {
    center += entry.position();
  }
  center /= static_cast<float>(selection_.size());
  focus_callback_(center);
}

}  // namespace common
}  // namespace autoviz
