/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/service/service_types.hpp"

namespace autoviz {
namespace service_panel {

ServiceCallPanelConfig DefaultServiceCallPanelConfig() {
  ServiceCallPanelConfig config;
  config.request_json = QStringLiteral("{}");
  config.timeout_sec = 5;
  return config;
}

}  // namespace service_panel
}  // namespace autoviz
