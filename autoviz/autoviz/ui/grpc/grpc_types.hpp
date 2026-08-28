/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>

namespace autoviz {
namespace grpc_panel {

struct GrpcPanelPersistConfig {
  QString title;
  QString url;
  bool tls = false;
  QString method_full_name;
  QString message_json = QStringLiteral("{}");
  int timeout_ms = 10000;
  bool verify_cert = true;
  QString ssl_override;
  bool include_defaults = false;
  int max_response_mb = 16;
};

GrpcPanelPersistConfig DefaultGrpcPanelPersistConfig();

}  // namespace grpc_panel
}  // namespace autoviz
