/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/teleop/teleop_control_widget.hpp"

#include <algorithm>

#include <QAbstractButton>
#include <QButtonGroup>
#include <QDoubleSpinBox>
#include <QFocusEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QPushButton>
#include <QToolButton>
#include <QVBoxLayout>

#include "autoviz/ui/teleop/teleop_joystick_widget.hpp"

namespace autoviz {
namespace teleop {
namespace {

constexpr char kBg[] = "#f8f9fb";
constexpr char kSurface[] = "#ffffff";
constexpr char kBorder[] = "#cbd5e1";
constexpr char kText[] = "#1e293b";
constexpr char kTextMuted[] = "#64748b";
constexpr char kAccent[] = "#0891b2";
constexpr char kDanger[] = "#dc2626";

QString ModeToggleStyle() {
  return QStringLiteral(
      "QToolButton {"
      "  background: transparent; color: %1;"
      "  border: none; border-radius: 8px;"
      "  padding: 6px 14px; font-weight: 600;"
      "}"
      "QToolButton:checked {"
      "  background: rgba(8,145,178,0.16); color: %2;"
      "}"
      "QToolButton:hover:!checked {"
      "  background: rgba(15,23,42,0.04); color: %3;"
      "}")
      .arg(QLatin1String(kTextMuted), QLatin1String(kAccent),
           QLatin1String(kText));
}

}  // namespace

TeleopControlWidget::TeleopControlWidget(QWidget* parent) : QWidget(parent) {
  setObjectName(QStringLiteral("TeleopControlContent"));
  setAttribute(Qt::WA_StyledBackground, true);
  setFocusPolicy(Qt::StrongFocus);
  setStyleSheet(QStringLiteral(
      "QWidget#TeleopControlContent {"
      "  background: %1; color: %2;"
      "}")
                    .arg(QLatin1String(kBg), QLatin1String(kText)));

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(14, 14, 14, 14);
  root->setSpacing(12);

  auto* mode_bar = new QFrame(this);
  mode_bar->setObjectName(QStringLiteral("TeleopModeBar"));
  mode_bar->setStyleSheet(QStringLiteral(
      "QFrame#TeleopModeBar {"
      "  background: %1;"
      "  border: 1px solid %2;"
      "  border-radius: 12px;"
      "}")
                              .arg(QLatin1String(kSurface),
                                   QLatin1String(kBorder)));
  auto* mode_layout = new QHBoxLayout(mode_bar);
  mode_layout->setContentsMargins(6, 6, 6, 6);
  mode_layout->setSpacing(4);

  auto* mode_label = new QLabel(tr("Mode"), mode_bar);
  mode_label->setStyleSheet(
      QStringLiteral("color: %1; font-size: 12px; font-weight: 700; padding: 0 6px;")
          .arg(QLatin1String(kTextMuted)));
  mode_layout->addWidget(mode_label);

  mode_group_ = new QButtonGroup(this);
  mode_group_->setExclusive(true);
  dual_mode_button_ = new QToolButton(mode_bar);
  dual_mode_button_->setText(tr("Dual sticks"));
  dual_mode_button_->setCheckable(true);
  dual_mode_button_->setChecked(true);
  dual_mode_button_->setCursor(Qt::PointingHandCursor);
  dual_mode_button_->setStyleSheet(ModeToggleStyle());
  arcade_mode_button_ = new QToolButton(mode_bar);
  arcade_mode_button_->setText(tr("Arcade"));
  arcade_mode_button_->setCheckable(true);
  arcade_mode_button_->setCursor(Qt::PointingHandCursor);
  arcade_mode_button_->setStyleSheet(ModeToggleStyle());
  mode_group_->addButton(dual_mode_button_,
                         static_cast<int>(TeleopStickMode::kDual));
  mode_group_->addButton(arcade_mode_button_,
                         static_cast<int>(TeleopStickMode::kArcade));
  mode_layout->addWidget(dual_mode_button_);
  mode_layout->addWidget(arcade_mode_button_);
  mode_layout->addSpacing(10);

  auto* speed_label = new QLabel(tr("Max v"), mode_bar);
  speed_label->setStyleSheet(
      QStringLiteral("color: %1; font-size: 11px; font-weight: 700;")
          .arg(QLatin1String(kTextMuted)));
  mode_layout->addWidget(speed_label);
  max_linear_spin_ = new QDoubleSpinBox(mode_bar);
  max_linear_spin_->setRange(0.01, 10.0);
  max_linear_spin_->setSingleStep(0.05);
  max_linear_spin_->setDecimals(2);
  max_linear_spin_->setSuffix(QStringLiteral(" m/s"));
  max_linear_spin_->setValue(max_linear_speed_);
  max_linear_spin_->setFixedWidth(110);
  max_linear_spin_->setToolTip(tr("Maximum linear speed"));
  max_linear_spin_->setStyleSheet(QStringLiteral(
      "QDoubleSpinBox {"
      "  background: %1; color: %2;"
      "  border: 1px solid %3; border-radius: 8px;"
      "  padding: 3px 6px;"
      "}")
                                      .arg(QLatin1String(kBg), QLatin1String(kText),
                                           QLatin1String(kBorder)));
  mode_layout->addWidget(max_linear_spin_);

  auto* yaw_label = new QLabel(tr("Max ω"), mode_bar);
  yaw_label->setStyleSheet(
      QStringLiteral("color: %1; font-size: 11px; font-weight: 700;")
          .arg(QLatin1String(kTextMuted)));
  mode_layout->addWidget(yaw_label);
  max_angular_spin_ = new QDoubleSpinBox(mode_bar);
  max_angular_spin_->setRange(0.01, 10.0);
  max_angular_spin_->setSingleStep(0.05);
  max_angular_spin_->setDecimals(2);
  max_angular_spin_->setSuffix(QStringLiteral(" rad/s"));
  max_angular_spin_->setValue(max_angular_speed_);
  max_angular_spin_->setFixedWidth(120);
  max_angular_spin_->setToolTip(tr("Maximum angular speed"));
  max_angular_spin_->setStyleSheet(max_linear_spin_->styleSheet());
  mode_layout->addWidget(max_angular_spin_);
  mode_layout->addStretch(1);
  root->addWidget(mode_bar);

  hint_label_ = new QLabel(this);
  hint_label_->setAlignment(Qt::AlignCenter);
  hint_label_->setWordWrap(true);
  hint_label_->setStyleSheet(
      QStringLiteral("color: %1; font-size: 12px;").arg(QLatin1String(kTextMuted)));
  root->addWidget(hint_label_);

  dual_frame_ = new QFrame(this);
  dual_frame_->setObjectName(QStringLiteral("TeleopSticksCard"));
  dual_frame_->setStyleSheet(QStringLiteral(
      "QFrame#TeleopSticksCard {"
      "  background: %1;"
      "  border: 1px solid %2;"
      "  border-radius: 16px;"
      "}")
                                 .arg(QLatin1String(kSurface),
                                      QLatin1String(kBorder)));
  auto* dual_layout = new QVBoxLayout(dual_frame_);
  dual_layout->setContentsMargins(16, 16, 16, 12);
  dual_layout->setSpacing(8);
  auto* dual_row = new QHBoxLayout();
  dual_row->setSpacing(20);
  move_joystick_ = new TeleopJoystickWidget(tr("Move"), dual_frame_);
  turn_joystick_ = new TeleopJoystickWidget(tr("Turn"), dual_frame_);
  dual_row->addStretch(1);
  dual_row->addWidget(move_joystick_, 2);
  dual_row->addWidget(turn_joystick_, 2);
  dual_row->addStretch(1);
  dual_layout->addLayout(dual_row, 1);
  auto* dual_legend = new QLabel(
      tr("Move: forward / strafe    ·    Turn: yaw"), dual_frame_);
  dual_legend->setAlignment(Qt::AlignCenter);
  dual_legend->setStyleSheet(QStringLiteral(
      "color: %1; font-size: 11px; font-weight: 600;")
                                 .arg(QLatin1String(kAccent)));
  dual_layout->addWidget(dual_legend);
  root->addWidget(dual_frame_, 1);

  arcade_frame_ = new QFrame(this);
  arcade_frame_->setObjectName(QStringLiteral("TeleopArcadeCard"));
  arcade_frame_->setStyleSheet(QStringLiteral(
      "QFrame#TeleopArcadeCard {"
      "  background: %1;"
      "  border: 1px solid %2;"
      "  border-radius: 16px;"
      "}")
                                   .arg(QLatin1String(kSurface),
                                        QLatin1String(kBorder)));
  auto* arcade_layout = new QVBoxLayout(arcade_frame_);
  arcade_layout->setContentsMargins(16, 16, 16, 12);
  arcade_layout->setSpacing(8);
  auto* arcade_row = new QHBoxLayout();
  arcade_joystick_ = new TeleopJoystickWidget(tr("Drive"), arcade_frame_);
  arcade_row->addStretch(1);
  arcade_row->addWidget(arcade_joystick_, 3);
  arcade_row->addStretch(1);
  arcade_layout->addLayout(arcade_row, 1);
  auto* arcade_legend = new QLabel(
      tr("↑↓ speed    ·    ←→ turn    ·    WASD / arrows / Space"),
      arcade_frame_);
  arcade_legend->setAlignment(Qt::AlignCenter);
  arcade_legend->setStyleSheet(QStringLiteral(
      "color: %1; font-size: 11px; font-weight: 600;")
                                   .arg(QLatin1String(kAccent)));
  arcade_layout->addWidget(arcade_legend);
  root->addWidget(arcade_frame_, 1);

  auto* stop_row = new QHBoxLayout();
  stop_row->addStretch(1);
  auto* stop_button = new QPushButton(tr("STOP"), this);
  stop_button->setToolTip(tr("Emergency stop"));
  stop_button->setMinimumSize(140, 42);
  stop_button->setMaximumWidth(180);
  stop_button->setCursor(Qt::PointingHandCursor);
  stop_button->setStyleSheet(QStringLiteral(
      "QPushButton {"
      "  background: %1;"
      "  border: 1px solid %1;"
      "  border-radius: 21px;"
      "  color: white;"
      "  font-weight: 800;"
      "  font-size: 14px;"
      "  letter-spacing: 1px;"
      "  padding: 8px 28px;"
      "}"
      "QPushButton:hover { background: #b91c1c; border-color: #b91c1c; }"
      "QPushButton:pressed { background: #991b1b; border-color: #991b1b; }")
                                 .arg(QLatin1String(kDanger)));
  stop_row->addWidget(stop_button);
  stop_row->addStretch(1);
  root->addLayout(stop_row);

  connect(mode_group_, &QButtonGroup::idClicked, this, [this](int id) {
    setStickMode(static_cast<TeleopStickMode>(id));
  });
  connect(max_linear_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, [this](double) { emitMaxSpeedsFromUi(); });
  connect(max_angular_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, [this](double) { emitMaxSpeedsFromUi(); });
  connect(move_joystick_, &TeleopJoystickWidget::valueChanged, this,
          [this](double x, double y) {
            if (stick_mode_ == TeleopStickMode::kDual) {
              emit linearChanged(x, y);
            }
          });
  connect(move_joystick_, &TeleopJoystickWidget::released, this, [this]() {
    if (stick_mode_ == TeleopStickMode::kDual) {
      emit linearReleased();
    }
  });
  connect(turn_joystick_, &TeleopJoystickWidget::valueChanged, this,
          [this](double x, double /*y*/) {
            if (stick_mode_ == TeleopStickMode::kDual) {
              emit angularChanged(x);
            }
          });
  connect(turn_joystick_, &TeleopJoystickWidget::released, this, [this]() {
    if (stick_mode_ == TeleopStickMode::kDual) {
      emit angularReleased();
    }
  });
  connect(arcade_joystick_, &TeleopJoystickWidget::valueChanged, this,
          [this](double x, double y) {
            if (stick_mode_ == TeleopStickMode::kArcade && !keyboard_driving_) {
              emitArcadeFromStick(x, y, false);
            }
          });
  connect(arcade_joystick_, &TeleopJoystickWidget::released, this, [this]() {
    if (stick_mode_ == TeleopStickMode::kArcade && !keyboard_driving_) {
      emitArcadeFromStick(0.0, 0.0, true);
    }
  });
  connect(stop_button, &QPushButton::clicked, this, [this]() {
    clearKeyboardState();
    emit stopClicked();
  });

  installEventFilter(this);
  applyModeUi();
}

void TeleopControlWidget::resetJoysticks() {
  clearKeyboardState();
  if (move_joystick_ != nullptr) {
    move_joystick_->resetVisual();
  }
  if (turn_joystick_ != nullptr) {
    turn_joystick_->resetVisual();
  }
  if (arcade_joystick_ != nullptr) {
    arcade_joystick_->resetVisual();
  }
}

void TeleopControlWidget::setMaxSpeeds(double max_linear, double max_angular) {
  max_linear_speed_ = std::max(0.01, max_linear);
  max_angular_speed_ = std::max(0.01, max_angular);
  suppress_speed_signal_ = true;
  if (max_linear_spin_ != nullptr) {
    max_linear_spin_->setValue(max_linear_speed_);
  }
  if (max_angular_spin_ != nullptr) {
    max_angular_spin_->setValue(max_angular_speed_);
  }
  suppress_speed_signal_ = false;
}

void TeleopControlWidget::emitMaxSpeedsFromUi() {
  if (suppress_speed_signal_ || max_linear_spin_ == nullptr ||
      max_angular_spin_ == nullptr) {
    return;
  }
  max_linear_speed_ = max_linear_spin_->value();
  max_angular_speed_ = max_angular_spin_->value();
  emit maxSpeedsChanged(max_linear_speed_, max_angular_speed_);
}

void TeleopControlWidget::setStickMode(TeleopStickMode mode) {
  if (stick_mode_ == mode) {
    applyModeUi();
    return;
  }
  clearKeyboardState();
  resetJoysticks();
  stick_mode_ = mode;
  applyModeUi();
  emit stickModeChanged(mode);
  emit stopClicked();
}

void TeleopControlWidget::applyModeUi() {
  const bool arcade = stick_mode_ == TeleopStickMode::kArcade;
  if (dual_mode_button_ != nullptr) {
    dual_mode_button_->setChecked(!arcade);
  }
  if (arcade_mode_button_ != nullptr) {
    arcade_mode_button_->setChecked(arcade);
  }
  if (dual_frame_ != nullptr) {
    dual_frame_->setVisible(!arcade);
  }
  if (arcade_frame_ != nullptr) {
    arcade_frame_->setVisible(arcade);
  }
  if (hint_label_ != nullptr) {
    hint_label_->setText(
        arcade ? tr("Arcade: one stick or keyboard (WASD / arrows). Click panel to focus.")
               : tr("Dual: left stick moves, right stick turns."));
  }
  if (arcade) {
    setFocus(Qt::OtherFocusReason);
  }
}

void TeleopControlWidget::emitArcadeFromStick(double x, double y, bool released) {
  // Stick Y: forward is negative (screen up). Stick X: turn.
  emit linearChanged(0.0, y);
  emit angularChanged(x);
  if (released) {
    emit linearReleased();
    emit angularReleased();
  }
}

void TeleopControlWidget::clearKeyboardState() {
  key_forward_ = false;
  key_back_ = false;
  key_left_ = false;
  key_right_ = false;
  keyboard_driving_ = false;
}

void TeleopControlWidget::updateArcadeFromKeyboard() {
  if (stick_mode_ != TeleopStickMode::kArcade) {
    return;
  }
  double x = 0.0;
  double y = 0.0;
  if (key_left_) {
    x -= 1.0;
  }
  if (key_right_) {
    x += 1.0;
  }
  if (key_forward_) {
    y -= 1.0;
  }
  if (key_back_) {
    y += 1.0;
  }
  const bool any = key_forward_ || key_back_ || key_left_ || key_right_;
  keyboard_driving_ = any;
  if (arcade_joystick_ != nullptr) {
    arcade_joystick_->setNormalizedValue(QPointF(x, y), false);
  }
  if (any) {
    emitArcadeFromStick(x, y, false);
  } else {
    emitArcadeFromStick(0.0, 0.0, true);
  }
}

void TeleopControlWidget::keyPressEvent(QKeyEvent* event) {
  if (stick_mode_ != TeleopStickMode::kArcade || event->isAutoRepeat()) {
    QWidget::keyPressEvent(event);
    return;
  }
  bool handled = true;
  switch (event->key()) {
    case Qt::Key_W:
    case Qt::Key_Up:
      key_forward_ = true;
      break;
    case Qt::Key_S:
    case Qt::Key_Down:
      key_back_ = true;
      break;
    case Qt::Key_A:
    case Qt::Key_Left:
      key_left_ = true;
      break;
    case Qt::Key_D:
    case Qt::Key_Right:
      key_right_ = true;
      break;
    case Qt::Key_Space:
      clearKeyboardState();
      if (arcade_joystick_ != nullptr) {
        arcade_joystick_->setNormalizedValue(QPointF(0.0, 0.0), false);
      }
      emit stopClicked();
      event->accept();
      return;
    default:
      handled = false;
      break;
  }
  if (!handled) {
    QWidget::keyPressEvent(event);
    return;
  }
  updateArcadeFromKeyboard();
  event->accept();
}

void TeleopControlWidget::keyReleaseEvent(QKeyEvent* event) {
  if (stick_mode_ != TeleopStickMode::kArcade || event->isAutoRepeat()) {
    QWidget::keyReleaseEvent(event);
    return;
  }
  bool handled = true;
  switch (event->key()) {
    case Qt::Key_W:
    case Qt::Key_Up:
      key_forward_ = false;
      break;
    case Qt::Key_S:
    case Qt::Key_Down:
      key_back_ = false;
      break;
    case Qt::Key_A:
    case Qt::Key_Left:
      key_left_ = false;
      break;
    case Qt::Key_D:
    case Qt::Key_Right:
      key_right_ = false;
      break;
    default:
      handled = false;
      break;
  }
  if (!handled) {
    QWidget::keyReleaseEvent(event);
    return;
  }
  updateArcadeFromKeyboard();
  event->accept();
}

void TeleopControlWidget::focusInEvent(QFocusEvent* event) {
  QWidget::focusInEvent(event);
}

bool TeleopControlWidget::eventFilter(QObject* watched, QEvent* event) {
  if (watched == this && event->type() == QEvent::MouseButtonPress &&
      stick_mode_ == TeleopStickMode::kArcade) {
    setFocus(Qt::MouseFocusReason);
  }
  return QWidget::eventFilter(watched, event);
}

}  // namespace teleop
}  // namespace autoviz
