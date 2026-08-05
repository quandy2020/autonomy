/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

namespace autoviz {
namespace teleop {

class TeleopJoystickWidget;

class TeleopControlWidget : public QWidget {
  Q_OBJECT

 public:
  explicit TeleopControlWidget(QWidget* parent = nullptr);

  void resetJoysticks();

 signals:
  /** Left stick: x = strafe, y = forward. Right stick: x = turn. */
  void linearChanged(double x, double y);
  void angularChanged(double turn);
  void linearReleased();
  void angularReleased();
  void stopClicked();

 private:
  TeleopJoystickWidget* move_joystick_ = nullptr;
  TeleopJoystickWidget* turn_joystick_ = nullptr;
};

}  // namespace teleop
}  // namespace autoviz
