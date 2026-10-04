/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file view_state_io.hpp
 * @brief Conversion between @ref SavedViewConfig and @ref rendering::ViewState.
 *
 * Used when saving/restoring viewpoints from the Views panel and session
 * config, and when applying bookmarks to the live @ref rendering::ViewController.
 *
 * @see SavedViewConfig
 * @see rendering::ViewState
 * @see ViewManager
 */

#pragma once

#include <string>

#include "autoviz/common/session_config.hpp"
#include "autoviz/rendering/view_controller.hpp"

namespace autoviz {
namespace common {

/**
 * @brief Builds a persistable saved-view record from live camera state.
 *
 * @param name Bookmark display name (e.g. @c "View 1").
 * @param state Current @ref rendering::ViewState from the controller.
 * @return Populated @ref SavedViewConfig.
 * @see ToViewState()
 */
SavedViewConfig ToSavedViewConfig(const std::string& name,
                                  const rendering::ViewState& state);

/**
 * @brief Restores a @ref rendering::ViewState from a saved-view record.
 *
 * @param config Bookmark / current-view config from session.
 * @return View state suitable for @c ViewController::setViewState.
 * @see ToSavedViewConfig()
 */
rendering::ViewState ToViewState(const SavedViewConfig& config);

}  // namespace common
}  // namespace autoviz
