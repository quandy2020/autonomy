/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file view_manager.hpp
 * @brief Config-layer manager for saved viewpoints and current view type.
 *
 * Mirrors @c rviz_common::ViewManager at the session-config level (not the
 * live @ref rendering::ViewController). Persists bookmarks and the active
 * controller type name into @ref SessionConfig.
 *
 * @see SavedViewConfig
 * @see ViewControllerRegistry
 * @see VisualizationManager
 */

#pragma once

#include <string>
#include <vector>

#include "autoviz/common/session_config.hpp"

namespace autoviz {
namespace common {

/**
 * @class ViewManager
 * @brief Owns saved-view bookmarks and the “current view” snapshot.
 *
 * The Views panel and VisualizationFrame read/write this object via
 * VisualizationManager accessors; the render window binds a separate
 * @ref rendering::ViewController for live camera math.
 */
class ViewManager {
 public:
  /**
   * @brief Returns the in-memory bookmark list.
   * @return Const reference to saved views.
   */
  const std::vector<SavedViewConfig>& savedViews() const { return saved_views_; }

  /**
   * @brief Replaces the entire bookmark list.
   * @param views New bookmarks (order preserved).
   */
  void setSavedViews(const std::vector<SavedViewConfig>& views) {
    saved_views_ = views;
  }

  /**
   * @brief Whether a current-view snapshot has been set.
   * @return @c true if @ref currentView() is meaningful.
   */
  bool hasCurrentView() const { return has_current_view_; }

  /**
   * @brief Returns the current-view snapshot.
   * @return Const reference; valid when @ref hasCurrentView() is @c true.
   */
  const SavedViewConfig& currentView() const { return current_view_; }

  /**
   * @brief Stores the current-view snapshot and marks it present.
   * @param view Snapshot to keep (also updates @ref currentTypeName).
   */
  void setCurrentView(const SavedViewConfig& view);

  /**
   * @brief Active view-controller type name (e.g. @c "Orbit").
   * @return Const reference to the type string.
   */
  const std::string& currentTypeName() const { return current_type_name_; }

  /**
   * @brief Sets the active view-controller type name.
   * @param name Type id known to @ref ViewControllerRegistry.
   */
  void setCurrentTypeName(const std::string& name);

  /**
   * @brief Lists declared (registry) view-controller type names.
   * @return Type name vector for UI combo boxes.
   */
  std::vector<std::string> declaredTypeNames() const;

  /**
   * @brief Loads saved views / current view from a session config.
   * @param config Source session.
   */
  void loadFromSession(const SessionConfig& config);

  /**
   * @brief Writes saved views / current view into a session config.
   * @param[in,out] config Destination session (must be non-null).
   */
  void saveToSession(SessionConfig* config) const;

  /**
   * @brief Appends a bookmark.
   * @param view New saved view.
   * @return @c true on success.
   */
  bool addSavedView(const SavedViewConfig& view);

  /**
   * @brief Removes a bookmark by name.
   * @param name Bookmark name to remove.
   * @return @c true if a matching view was removed.
   */
  bool removeSavedView(const std::string& name);

  /**
   * @brief Finds a bookmark by name.
   * @param name Bookmark name.
   * @return Pointer into @c saved_views_, or @c nullptr if absent.
   */
  const SavedViewConfig* savedViewByName(const std::string& name) const;

 private:
  std::vector<SavedViewConfig> saved_views_; /**< Bookmarks. */
  SavedViewConfig current_view_;             /**< Live snapshot. */
  bool has_current_view_ = false;            /**< Whether @c current_view_ is set. */
  std::string current_type_name_ = "Orbit";  /**< Active controller type. */
};

}  // namespace common
}  // namespace autoviz
