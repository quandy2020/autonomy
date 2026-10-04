/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file service_config_io.hpp
 * @brief Convert Service Call runtime config ↔ session persist records.
 */

#pragma once

#include <QString>

#include "autoviz/common/session_config.hpp"
#include "autoviz/ui/service/service_types.hpp"

namespace autoviz {
namespace service_panel {

common::ServicePanelPersistConfig ToPersistConfig(
    const QString& object_name, const ServiceCallPanelConfig& config,
    bool settings_visible);

ServiceCallPanelConfig FromPersistConfig(
    const common::ServicePanelPersistConfig& persist);

}  // namespace service_panel
}  // namespace autoviz
