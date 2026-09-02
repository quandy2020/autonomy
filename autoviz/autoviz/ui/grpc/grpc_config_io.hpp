/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>

#include "autoviz/common/session_config.hpp"
#include "autoviz/ui/grpc/grpc_types.hpp"

namespace autoviz {
namespace grpc_panel {

common::GrpcPanelPersistConfig ToPersistConfig(
    const QString& object_name, const GrpcPanelPersistConfig& config);
GrpcPanelPersistConfig FromPersistConfig(
    const common::GrpcPanelPersistConfig& persist);

}  // namespace grpc_panel
}  // namespace autoviz
