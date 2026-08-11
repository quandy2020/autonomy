/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>

#include "autoviz/common/session_config.hpp"
#include "autoviz/ui/publish/publish_types.hpp"

namespace autoviz {
namespace publish_panel {

common::PublishPanelPersistConfig ToPersistConfig(const QString& object_name,
                                                 const PublishPanelConfig& config);
PublishPanelConfig FromPersistConfig(
    const common::PublishPanelPersistConfig& persist);

}  // namespace publish_panel
}  // namespace autoviz
