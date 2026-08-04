/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/view_manager.hpp"

#include <algorithm>

#include "autoviz/common/view_controller_registry.hpp"

namespace autoviz {
namespace common {

void ViewManager::setCurrentView(const SavedViewConfig& view) {
  current_view_ = view;
  has_current_view_ = true;
  current_type_name_ = view.type;
}

void ViewManager::setCurrentTypeName(const std::string& name) {
  current_type_name_ = ViewControllerRegistry::instance().isKnownType(name)
                           ? name
                           : ViewControllerRegistry::instance().defaultTypeName();
}

std::vector<std::string> ViewManager::declaredTypeNames() const {
  return ViewControllerRegistry::instance().typeNames();
}

void ViewManager::loadFromSession(const SessionConfig& config) {
  saved_views_ = config.views;
  has_current_view_ = config.has_current_view;
  if (config.has_current_view) {
    current_view_ = config.current_view;
    current_type_name_ = config.current_view.type;
  } else {
    current_type_name_ = config.view_controller;
  }
  setCurrentTypeName(current_type_name_);
}

void ViewManager::saveToSession(SessionConfig* config) const {
  if (config == nullptr) {
    return;
  }
  config->views = saved_views_;
  config->view_controller = current_type_name_;
  config->has_current_view = has_current_view_;
  if (has_current_view_) {
    config->current_view = current_view_;
  }
}

bool ViewManager::addSavedView(const SavedViewConfig& view) {
  if (view.name.empty()) {
    return false;
  }
  for (const auto& existing : saved_views_) {
    if (existing.name == view.name) {
      return false;
    }
  }
  saved_views_.push_back(view);
  return true;
}

bool ViewManager::removeSavedView(const std::string& name) {
  const auto it = std::find_if(
      saved_views_.begin(), saved_views_.end(),
      [&name](const SavedViewConfig& view) { return view.name == name; });
  if (it == saved_views_.end()) {
    return false;
  }
  saved_views_.erase(it);
  return true;
}

const SavedViewConfig* ViewManager::savedViewByName(
    const std::string& name) const {
  for (const auto& view : saved_views_) {
    if (view.name == name) {
      return &view;
    }
  }
  return nullptr;
}

}  // namespace common
}  // namespace autoviz
