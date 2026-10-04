/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file publish_config_io.hpp
 * @brief Convert between runtime @ref PublishPanelConfig and session-persist
 *        @ref common::PublishPanelPersistConfig.
 *
 * Used by VisualizationFrame / FrameSession when loading and saving Autoviz
 * session files so Publish panel state survives restart.
 *
 * @see PublishPanelConfig
 * @see common::PublishPanelPersistConfig
 * @see common::SessionConfig
 */

#pragma once

#include <QString>

#include "autoviz/common/session_config.hpp"
#include "autoviz/ui/publish/publish_types.hpp"

namespace autoviz {
namespace publish_panel {

/**
 * @brief Maps an in-memory panel config to the session-persist POD.
 *
 * @param object_name Qt objectName of the panel dock (used as persist key).
 * @param config Runtime panel configuration to serialize.
 * @return Persist struct suitable for @ref common::SessionConfig::publish_panels.
 *
 * @see FromPersistConfig()
 */
common::PublishPanelPersistConfig ToPersistConfig(const QString& object_name,
                                                 const PublishPanelConfig& config);

/**
 * @brief Rebuilds a runtime panel config from session-persist data.
 *
 * @param persist Loaded persist entry (channels, presets, publishers, chrome).
 * @return Runtime @ref PublishPanelConfig ready for @ref PublishPanel::setConfig().
 *
 * @see ToPersistConfig()
 */
PublishPanelConfig FromPersistConfig(
    const common::PublishPanelPersistConfig& persist);

}  // namespace publish_panel
}  // namespace autoviz
