/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file map_config_io.hpp
 * @brief Convert Map runtime config ↔ session persist records.
 */

#pragma once

#include <QString>

#include "autoviz/common/session_config.hpp"
#include "autoviz/ui/map/map_types.hpp"

namespace autoviz {
namespace map {

common::MapPanelPersistConfig ToPersistConfig(const QString& object_name,
                                              const MapPanelConfig& config,
                                              bool settings_visible);

MapPanelConfig FromPersistConfig(const common::MapPanelPersistConfig& persist);

}  // namespace map
}  // namespace autoviz
