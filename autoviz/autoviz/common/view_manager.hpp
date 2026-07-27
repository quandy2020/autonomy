/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>

#include "autoviz/common/session_config.hpp"

namespace autoviz {
namespace common {

/** rviz_common::ViewManager — saved views + current view type (config layer). */
class ViewManager {
 public:
  const std::vector<SavedViewConfig>& savedViews() const { return saved_views_; }
  void setSavedViews(const std::vector<SavedViewConfig>& views) {
    saved_views_ = views;
  }

  bool hasCurrentView() const { return has_current_view_; }
  const SavedViewConfig& currentView() const { return current_view_; }
  void setCurrentView(const SavedViewConfig& view);

  const std::string& currentTypeName() const { return current_type_name_; }
  void setCurrentTypeName(const std::string& name);

  std::vector<std::string> declaredTypeNames() const;

  void loadFromSession(const SessionConfig& config);
  void saveToSession(SessionConfig* config) const;

  bool addSavedView(const SavedViewConfig& view);
  bool removeSavedView(const std::string& name);
  const SavedViewConfig* savedViewByName(const std::string& name) const;

 private:
  std::vector<SavedViewConfig> saved_views_;
  SavedViewConfig current_view_;
  bool has_current_view_ = false;
  std::string current_type_name_ = "Orbit";
};

}  // namespace common
}  // namespace autoviz
