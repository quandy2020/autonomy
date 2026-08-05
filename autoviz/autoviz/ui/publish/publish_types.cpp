/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/publish/publish_types.hpp"

namespace autoviz {
namespace publish_panel {

PublishPanelConfig DefaultPublishPanelConfig() {
  PublishPanelConfig config;
  config.channel = QStringLiteral("/cmd_vel");
  config.message_type = QStringLiteral("automsgs.msgs.geometry_msgs.Twist");
  return config;
}

}  // namespace publish_panel
}  // namespace autoviz
