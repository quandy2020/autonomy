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

struct TeleopPanelConfig {
  QString title = QStringLiteral("Teleop");
  QString topic = QStringLiteral("/cmd_vel");
  double publish_rate_hz = 1.0;
  bool stop_on_release = true;
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
