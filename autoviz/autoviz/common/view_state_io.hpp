/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <string>

#include "autoviz/common/session_config.hpp"
#include "autoviz/rendering/view_controller.hpp"

namespace autoviz {
namespace common {

SavedViewConfig ToSavedViewConfig(const std::string& name,
                                  const rendering::ViewState& state);
rendering::ViewState ToViewState(const SavedViewConfig& config);

}  // namespace common
}  // namespace autoviz
