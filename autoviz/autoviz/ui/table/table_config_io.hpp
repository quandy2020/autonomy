/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file table_config_io.hpp
 * @brief Convert Table panel runtime config ↔ session persist records.
 */

#pragma once

#include <QString>

#include "autoviz/common/session_config.hpp"
#include "autoviz/ui/table/table_types.hpp"

namespace autoviz {
namespace table_panel {

/**
 * @brief Runtime config → persist record.
 *
 * @param object_name Dock @c objectName.
 * @param config Runtime panel config.
 * @return Persist record for SessionConfig.
 */
common::TablePanelPersistConfig ToPersistConfig(const QString& object_name,
                                                const TablePanelConfig& config);

/**
 * @brief Persist record → runtime config.
 *
 * @param persist Session record.
 * @return Runtime config.
 */
TablePanelConfig FromPersistConfig(
    const common::TablePanelPersistConfig& persist);

}  // namespace table_panel
}  // namespace autoviz
