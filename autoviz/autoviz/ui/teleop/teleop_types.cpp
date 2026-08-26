/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/teleop/teleop_types.hpp"

namespace autoviz {
namespace teleop {

TeleopPanelConfig DefaultTeleopPanelConfig() {
  TeleopPanelConfig config;
  config.max_linear_speed = 0.5;
  config.max_angular_speed = 0.5;
  config.up.field = TeleopTwistField::kLinearX;
  config.up.value = config.max_linear_speed;
  config.down.field = TeleopTwistField::kLinearX;
  config.down.value = -config.max_linear_speed;
  config.left.field = TeleopTwistField::kAngularZ;
  config.left.value = config.max_angular_speed;
  config.right.field = TeleopTwistField::kAngularZ;
  config.right.value = -config.max_angular_speed;
  config.stop.value = 0.0;
  return config;
}

}  // namespace teleop
}  // namespace autoviz
