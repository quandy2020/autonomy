/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file teleop_config_io.hpp
 * @brief Convert Teleop runtime config ↔ session persist records.
 */

#pragma once

#include <QString>

#include "autoviz/common/session_config.hpp"
#include "autoviz/ui/teleop/teleop_types.hpp"

namespace autoviz {
namespace teleop {

common::TeleopPanelPersistConfig ToPersistConfig(
    const QString& object_name, const TeleopPanelConfig& config);

TeleopPanelConfig FromPersistConfig(
    const common::TeleopPanelPersistConfig& persist);

}  // namespace teleop
}  // namespace autoviz
