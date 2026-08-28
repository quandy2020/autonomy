/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/grpc/grpc_types.hpp"

namespace autoviz {
namespace grpc_panel {

GrpcPanelPersistConfig DefaultGrpcPanelPersistConfig() {
  GrpcPanelPersistConfig config;
  config.message_json = QStringLiteral("{}");
  config.timeout_ms = 10000;
  config.verify_cert = true;
  config.include_defaults = false;
  config.max_response_mb = 16;
  return config;
}

}  // namespace grpc_panel
}  // namespace autoviz
