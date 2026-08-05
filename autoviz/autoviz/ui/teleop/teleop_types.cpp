/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/teleop/teleop_types.hpp"

namespace autoviz {
namespace teleop {

TeleopPanelConfig DefaultTeleopPanelConfig() {
  TeleopPanelConfig config;
  config.up.field = TeleopTwistField::kLinearX;
  config.up.value = 0.5;
  config.down.field = TeleopTwistField::kLinearX;
  config.down.value = -0.5;
  config.left.field = TeleopTwistField::kAngularZ;
  config.left.value = 0.5;
  config.right.field = TeleopTwistField::kAngularZ;
  config.right.value = -0.5;
  config.stop.value = 0.0;
  return config;
}

}  // namespace teleop
}  // namespace autoviz
