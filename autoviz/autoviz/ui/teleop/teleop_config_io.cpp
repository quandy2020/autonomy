/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/teleop/teleop_config_io.hpp"

namespace autoviz {
namespace teleop {
namespace {

common::TeleopButtonPersistConfig ToButton(const TeleopButtonConfig& button) {
  common::TeleopButtonPersistConfig out;
  out.field = static_cast<int>(button.field);
  out.value = button.value;
  return out;
}

TeleopButtonConfig FromButton(const common::TeleopButtonPersistConfig& button) {
  TeleopButtonConfig out;
  const int field = button.field;
  out.field = (field >= static_cast<int>(TeleopTwistField::kLinearX) &&
               field <= static_cast<int>(TeleopTwistField::kAngularZ))
                  ? static_cast<TeleopTwistField>(field)
                  : TeleopTwistField::kLinearX;
  out.value = button.value;
  return out;
}

}  // namespace

common::TeleopPanelPersistConfig ToPersistConfig(
    const QString& object_name, const TeleopPanelConfig& config) {
  common::TeleopPanelPersistConfig persist;
  persist.object_name = object_name.toStdString();
  persist.title = config.title.toStdString();
  persist.topic = config.topic.toStdString();
  persist.publish_rate_hz = config.publish_rate_hz;
  persist.stop_on_release = config.stop_on_release;
  persist.smart_teleop_enabled = config.smart_teleop_enabled;
  persist.stick_mode = static_cast<int>(config.stick_mode);
  persist.max_linear_speed = config.max_linear_speed;
  persist.max_angular_speed = config.max_angular_speed;
  persist.up = ToButton(config.up);
  persist.down = ToButton(config.down);
  persist.left = ToButton(config.left);
  persist.right = ToButton(config.right);
  persist.stop = ToButton(config.stop);
  persist.settings_visible = config.settings_visible;
  return persist;
}

TeleopPanelConfig FromPersistConfig(
    const common::TeleopPanelPersistConfig& persist) {
  TeleopPanelConfig config = DefaultTeleopPanelConfig();
  config.title = QString::fromStdString(persist.title);
  if (!persist.topic.empty()) {
    config.topic = QString::fromStdString(persist.topic);
  }
  if (persist.publish_rate_hz > 0.0) {
    config.publish_rate_hz = persist.publish_rate_hz;
  }
  config.stop_on_release = persist.stop_on_release;
  config.smart_teleop_enabled = persist.smart_teleop_enabled;
  config.stick_mode = persist.stick_mode == static_cast<int>(TeleopStickMode::kArcade)
                          ? TeleopStickMode::kArcade
                          : TeleopStickMode::kDual;
  if (persist.max_linear_speed > 0.0) {
    config.max_linear_speed = persist.max_linear_speed;
  }
  if (persist.max_angular_speed > 0.0) {
    config.max_angular_speed = persist.max_angular_speed;
  }
  config.up = FromButton(persist.up);
  config.down = FromButton(persist.down);
  config.left = FromButton(persist.left);
  config.right = FromButton(persist.right);
  config.stop = FromButton(persist.stop);
  config.settings_visible = persist.settings_visible;
  return config;
}

}  // namespace teleop
}  // namespace autoviz
