/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/teleop/teleop_control_widget.hpp"

#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>

#include "autoviz/ui/teleop/teleop_joystick_widget.hpp"

namespace autoviz {
namespace teleop {

TeleopControlWidget::TeleopControlWidget(QWidget* parent) : QWidget(parent) {
  setAutoFillBackground(true);
  QPalette palette = this->palette();
  palette.setColor(QPalette::Window, QColor(28, 28, 30));
  setPalette(palette);

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(12, 12, 12, 12);
  root->setSpacing(12);

  auto* sticks_row = new QHBoxLayout();
  sticks_row->setSpacing(16);
  move_joystick_ = new TeleopJoystickWidget(tr("Move"), this);
  turn_joystick_ = new TeleopJoystickWidget(tr("Turn"), this);
  sticks_row->addStretch(1);
  sticks_row->addWidget(move_joystick_, 2);
  sticks_row->addWidget(turn_joystick_, 2);
  sticks_row->addStretch(1);
  root->addLayout(sticks_row, 1);

  auto* stop_row = new QHBoxLayout();
  stop_row->addStretch(1);
  auto* stop_button = new QPushButton(tr("STOP"), this);
  stop_button->setToolTip(tr("Emergency stop"));
  stop_button->setMinimumSize(120, 40);
  stop_button->setMaximumWidth(160);
  stop_button->setCursor(Qt::PointingHandCursor);
  stop_button->setStyleSheet(
      QStringLiteral(
          "QPushButton {"
          "  background-color: #3a2020;"
          "  border: 1px solid #aa4444;"
          "  border-radius: 20px;"
          "  color: #ff6666;"
          "  font-weight: bold;"
          "  font-size: 13px;"
          "  padding: 8px 24px;"
          "}"
          "QPushButton:hover { background-color: #4a2828; color: #ff8888; }"
          "QPushButton:pressed { background-color: #662222; color: #ffffff; }"));
  stop_row->addWidget(stop_button);
  stop_row->addStretch(1);
  root->addLayout(stop_row);

  connect(move_joystick_, &TeleopJoystickWidget::valueChanged, this,
          [this](double x, double y) { emit linearChanged(x, y); });
  connect(move_joystick_, &TeleopJoystickWidget::released, this,
          &TeleopControlWidget::linearReleased);
  connect(turn_joystick_, &TeleopJoystickWidget::valueChanged, this,
          [this](double x, double /*y*/) { emit angularChanged(x); });
  connect(turn_joystick_, &TeleopJoystickWidget::released, this,
          &TeleopControlWidget::angularReleased);
  connect(stop_button, &QPushButton::clicked, this, &TeleopControlWidget::stopClicked);
}

void TeleopControlWidget::resetJoysticks() {
  if (move_joystick_ != nullptr) {
    move_joystick_->resetVisual();
  }
  if (turn_joystick_ != nullptr) {
    turn_joystick_->resetVisual();
  }
}

}  // namespace teleop
}  // namespace autoviz
