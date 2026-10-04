/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file channel_graph_config_io.hpp
 * @brief Convert Channel Graph runtime config ↔ session persist records.
 */

#pragma once

#include <QString>

#include "autoviz/common/session_config.hpp"
#include "autoviz/ui/channel_graph/channel_graph_panel.hpp"

namespace autoviz {
namespace channel_graph {

common::ChannelGraphPanelPersistConfig ToPersistConfig(
    const QString& object_name, const ChannelGraphPanelConfig& config);

ChannelGraphPanelConfig FromPersistConfig(
    const common::ChannelGraphPanelPersistConfig& persist);

}  // namespace channel_graph
}  // namespace autoviz
