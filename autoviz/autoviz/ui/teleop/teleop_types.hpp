/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>

namespace autoviz {
namespace teleop {

enum class TeleopDirection {
  kUp = 0,
  kDown,
  kLeft,
  kRight,
  kStop,
};

enum class TeleopTwistField {
  kLinearX = 0,
  kLinearY,
  kLinearZ,
  kAngularX,
  kAngularY,
  kAngularZ,
};

struct TeleopButtonConfig {
  TeleopTwistField field = TeleopTwistField::kLinearX;
  double value = 0.0;
};

enum class TeleopStickMode {
  kDual = 0,    /** Move + Turn sticks (default). */
  kArcade = 1,  /** One stick: forward/back = speed, left/right = yaw. */
};

struct TeleopPanelConfig {
  QString title = QStringLiteral("Teleop");
  QString topic = QStringLiteral("/cmd_vel");
  double publish_rate_hz = 1.0;
  bool stop_on_release = true;
  /** Route commands through task teleop goal channel instead of /cmd_vel. */
  bool smart_teleop_enabled = false;
  TeleopStickMode stick_mode = TeleopStickMode::kDual;
  /** Max |linear.x / linear.y| in m/s when stick is fully deflected. */
  double max_linear_speed = 0.5;
  /** Max |angular.z| in rad/s when stick is fully deflected. */
  double max_angular_speed = 0.5;
  TeleopButtonConfig up;
  TeleopButtonConfig down;
  TeleopButtonConfig left;
  TeleopButtonConfig right;
  TeleopButtonConfig stop;
  bool settings_visible = false;
};

TeleopPanelConfig DefaultTeleopPanelConfig();

}  // namespace teleop
}  // namespace autoviz
