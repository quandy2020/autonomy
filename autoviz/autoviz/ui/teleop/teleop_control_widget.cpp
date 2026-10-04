/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/integration/teleop_channels.hpp"
#include "autoviz/ui/teleop/teleop_control_widget.hpp"

#include <algorithm>

#include <QAbstractButton>
#include <QAbstractSpinBox>
#include <QButtonGroup>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFocusEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QPushButton>
#include <QSizePolicy>
#include <QToolButton>
#include <QVBoxLayout>

#include "autoviz/ui/theme/glass.hpp"
#include "autoviz/ui/theme/style.hpp"
#include "autoviz/ui/teleop/teleop_joystick_widget.hpp"

namespace autoviz {
namespace teleop {
namespace {

QString HexAccent() { return glass::ShellCssAccent(); }
QString HexText() { return glass::ShellCssText(); }
QString HexMuted() { return glass::ShellCssMuted(); }
QString HexBorder() { return glass::ShellCssBorder(); }
QString HexBg() { return glass::ShellCssBg(); }

constexpr char kDanger[] = "#E11D48";
constexpr char kDangerHover[] = "#BE123C";
constexpr char kDangerPress[] = "#9F1239";

/** Refined instrument readout — quiet chrome, crisp value hierarchy. */
QWidget* MakeSpeedChip(const QString& title, const QString& tip,
                       QDoubleSpinBox** spin_out, double value,
                       const QString& unit, QWidget* parent) {
  auto* card = new QFrame(parent);
  card->setObjectName(QStringLiteral("TeleopSpeedCard"));
  card->setToolTip(tip);
  card->setStyleSheet(
      style::sheet(QStringLiteral("teleop/speed_card")));

  auto* root = new QVBoxLayout(card);
  root->setContentsMargins(12, 10, 12, 10);
  root->setSpacing(0);

  auto* title_label = new QLabel(title, card);
  title_label->setAlignment(Qt::AlignHCenter);
  title_label->setStyleSheet(
      style::sheet(QStringLiteral("teleop/speed_caption")));
  root->addWidget(title_label);
  root->addSpacing(6);

  auto* spin = new QDoubleSpinBox(card);
  spin->setRange(0.01, 10.0);
  spin->setSingleStep(0.05);
  spin->setDecimals(2);
  spin->setValue(value);
  spin->setToolTip(tip);
  spin->setButtonSymbols(QAbstractSpinBox::NoButtons);
  spin->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
  spin->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  spin->setFixedHeight(28);
  spin->setStyleSheet(
      style::sheet(QStringLiteral("teleop/speed_spin")));
  root->addWidget(spin);

  auto* unit_label = new QLabel(unit.trimmed(), card);
  unit_label->setAlignment(Qt::AlignHCenter);
  unit_label->setStyleSheet(
      style::sheet(QStringLiteral("teleop/speed_unit")));
  root->addWidget(unit_label);
  root->addSpacing(8);

  auto* rule = new QFrame(card);
  rule->setFixedHeight(1);
  rule->setStyleSheet(
      style::sheet(QStringLiteral("teleop/speed_rule")));
  root->addWidget(rule);
  root->addSpacing(8);

  auto* steppers = new QHBoxLayout();
  steppers->setContentsMargins(0, 0, 0, 0);
  steppers->setSpacing(10);
  steppers->addStretch(1);

  const QString step_style =
      style::sheet(QStringLiteral("teleop/step_button"));

  auto* minus = new QToolButton(card);
  minus->setText(QStringLiteral("−"));
  minus->setToolTip(tip);
  minus->setCursor(Qt::PointingHandCursor);
  minus->setFixedSize(28, 28);
  minus->setStyleSheet(step_style);
  steppers->addWidget(minus);

  auto* plus = new QToolButton(card);
  plus->setText(QStringLiteral("+"));
  plus->setToolTip(tip);
  plus->setCursor(Qt::PointingHandCursor);
  plus->setFixedSize(28, 28);
  plus->setStyleSheet(step_style);
  steppers->addWidget(plus);
  steppers->addStretch(1);
  root->addLayout(steppers);

  QObject::connect(minus, &QToolButton::clicked, spin, [spin]() {
    spin->setValue(spin->value() - spin->singleStep());
  });
  QObject::connect(plus, &QToolButton::clicked, spin, [spin]() {
    spin->setValue(spin->value() + spin->singleStep());
  });

  *spin_out = spin;
  return card;
}

}  // namespace

TeleopControlWidget::TeleopControlWidget(QWidget* parent) : QWidget(parent) {
  setObjectName(QStringLiteral("TeleopControlContent"));
  setAttribute(Qt::WA_StyledBackground, true);
  setFocusPolicy(Qt::StrongFocus);
  setMinimumWidth(220);
  setStyleSheet(
      style::sheet(QStringLiteral("teleop/control_root")));

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(10, 10, 10, 10);
  root->setSpacing(10);

  // —— Header glass strip ——
  auto* header = new QFrame(this);
  header->setObjectName(QStringLiteral("TeleopHeader"));
  header->setStyleSheet(
      style::sheet(QStringLiteral("teleop/header")));
  auto* header_layout = new QVBoxLayout(header);
  header_layout->setContentsMargins(8, 8, 8, 8);
  header_layout->setSpacing(8);

  auto* top_row = new QHBoxLayout();
  top_row->setSpacing(8);
  top_row->setContentsMargins(0, 0, 0, 0);

  auto* mode_shell = new QFrame(header);
  mode_shell->setObjectName(QStringLiteral("TeleopModeSegment"));
  mode_shell->setStyleSheet(
      style::sheet(QStringLiteral("teleop/mode_segment")));
  auto* mode_row = new QHBoxLayout(mode_shell);
  mode_row->setContentsMargins(3, 3, 3, 3);
  mode_row->setSpacing(2);
  mode_group_ = new QButtonGroup(this);
  mode_group_->setExclusive(true);
  dual_mode_button_ = new QToolButton(mode_shell);
  dual_mode_button_->setText(tr("Dual"));
  dual_mode_button_->setToolTip(tr("Left: Move · Right: Turn"));
  dual_mode_button_->setCheckable(true);
  dual_mode_button_->setChecked(true);
  dual_mode_button_->setCursor(Qt::PointingHandCursor);
  dual_mode_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  dual_mode_button_->setStyleSheet(
      style::sheet(QStringLiteral("teleop/mode_toggle")));
  arcade_mode_button_ = new QToolButton(mode_shell);
  arcade_mode_button_->setText(tr("Arcade"));
  arcade_mode_button_->setToolTip(tr("WASD / arrows to drive · Space: E-Stop"));
  arcade_mode_button_->setCheckable(true);
  arcade_mode_button_->setCursor(Qt::PointingHandCursor);
  arcade_mode_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  arcade_mode_button_->setStyleSheet(
      style::sheet(QStringLiteral("teleop/mode_toggle")));
  mode_group_->addButton(dual_mode_button_,
                         static_cast<int>(TeleopStickMode::kDual));
  mode_group_->addButton(arcade_mode_button_,
                         static_cast<int>(TeleopStickMode::kArcade));
  mode_row->addWidget(dual_mode_button_);
  mode_row->addWidget(arcade_mode_button_);
  top_row->addWidget(mode_shell, 1);

  smart_teleop_check_ = new QCheckBox(tr("Smart"), header);
  smart_teleop_check_->setCursor(Qt::PointingHandCursor);
  smart_teleop_check_->setStyleSheet(
      style::sheet(QStringLiteral("teleop/smart_check")));
  smart_teleop_check_->setToolTip(
      tr("Send velocity via autonomy task teleop (MPPI assist), not /cmd_vel"));
  top_row->addWidget(smart_teleop_check_, 0, Qt::AlignVCenter);
  header_layout->addLayout(top_row);

  auto* speed_row = new QHBoxLayout();
  speed_row->setSpacing(8);
  speed_row->addWidget(
      MakeSpeedChip(tr("LINEAR"), tr("Max linear speed"), &max_linear_spin_,
                    max_linear_speed_, QStringLiteral("m/s"), header),
      1);
  speed_row->addWidget(
      MakeSpeedChip(tr("ANGULAR"), tr("Max angular speed"), &max_angular_spin_,
                    max_angular_speed_, QStringLiteral("rad/s"), header),
      1);
  header_layout->addLayout(speed_row);
  root->addWidget(header);

  // —— Drive surface (soft glass, pads breathe) ——
  dual_frame_ = new QFrame(this);
  dual_frame_->setObjectName(QStringLiteral("TeleopSticksCard"));
  dual_frame_->setMinimumHeight(180);
  dual_frame_->setStyleSheet(
      style::sheet(QStringLiteral("teleop/sticks_card")));
  auto* dual_layout = new QVBoxLayout(dual_frame_);
  dual_layout->setContentsMargins(14, 16, 14, 14);
  dual_layout->setSpacing(0);
  auto* dual_row = new QHBoxLayout();
  dual_row->setSpacing(18);

  auto make_pad_column = [&](TeleopJoystickWidget** stick_out, TeleopStickAxes axes,
                             const QString& caption, const QString& tip) {
    auto* col = new QWidget(dual_frame_);
    auto* col_layout = new QVBoxLayout(col);
    col_layout->setContentsMargins(0, 0, 0, 0);
    col_layout->setSpacing(6);
    auto* stick = new TeleopJoystickWidget(tr("OK"), col);
    stick->setAxes(axes);
    stick->setToolTip(tip);
    *stick_out = stick;
    col_layout->addWidget(stick, 1);
    auto* cap = new QLabel(caption, col);
    cap->setAlignment(Qt::AlignHCenter);
    cap->setStyleSheet(
        style::sheet(QStringLiteral("teleop/pad_caption")));
    col_layout->addWidget(cap);
    return col;
  };

  dual_row->addWidget(
      make_pad_column(&move_joystick_, TeleopStickAxes::kOmni, tr("Move"),
                      tr("Move — drag the ring")),
      1);
  dual_row->addWidget(
      make_pad_column(&turn_joystick_, TeleopStickAxes::kYaw, tr("Turn"),
                      tr("Turn — drag left / right on the ring")),
      1);
  dual_layout->addLayout(dual_row, 1);
  root->addWidget(dual_frame_, 1);

  arcade_frame_ = new QFrame(this);
  arcade_frame_->setObjectName(QStringLiteral("TeleopArcadeCard"));
  arcade_frame_->setMinimumHeight(180);
  arcade_frame_->setStyleSheet(
      style::sheet(QStringLiteral("teleop/arcade_card")));
  auto* arcade_layout = new QVBoxLayout(arcade_frame_);
  arcade_layout->setContentsMargins(20, 18, 20, 18);
  arcade_layout->setSpacing(0);
  arcade_joystick_ = new TeleopJoystickWidget(tr("OK"), arcade_frame_);
  arcade_joystick_->setAxes(TeleopStickAxes::kOmni);
  arcade_joystick_->setToolTip(tr("Drive — drag the ring · WASD / arrows"));
  arcade_layout->addWidget(arcade_joystick_, 1);
  root->addWidget(arcade_frame_, 1);

  hint_label_ = new QLabel(this);
  hint_label_->setWordWrap(true);
  hint_label_->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
  hint_label_->setStyleSheet(
      style::sheet(QStringLiteral("teleop/hint")));
  root->addWidget(hint_label_);

  auto* stop_button = new QPushButton(tr("E-STOP"), this);
  stop_button->setToolTip(tr("Emergency stop (Space)"));
  stop_button->setMinimumHeight(40);
  stop_button->setCursor(Qt::PointingHandCursor);
  stop_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  stop_button->setStyleSheet(
      style::sheet(QStringLiteral("teleop/estop")));
  root->addWidget(stop_button);

  connect(mode_group_, &QButtonGroup::idClicked, this, [this](int id) {
    setStickMode(static_cast<TeleopStickMode>(id));
  });
  connect(max_linear_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, [this](double) { emitMaxSpeedsFromUi(); });
  connect(max_angular_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, [this](double) { emitMaxSpeedsFromUi(); });
  connect(smart_teleop_check_, &QCheckBox::toggled, this, [this](bool checked) {
    smart_teleop_enabled_ = checked;
    updateSmartTeleopHint();
    emit smartTeleopChanged(checked);
  });
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
              emit angularChanged(-x);
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
  updateSmartTeleopHint();
  applyModeUi();
}

void TeleopControlWidget::setSmartTeleopEnabled(bool enabled) {
  smart_teleop_enabled_ = enabled;
  if (smart_teleop_check_ != nullptr) {
    smart_teleop_check_->blockSignals(true);
    smart_teleop_check_->setChecked(enabled);
    smart_teleop_check_->blockSignals(false);
  }
  updateSmartTeleopHint();
}

void TeleopControlWidget::setSmartTeleopAvailable(bool available) {
  if (smart_teleop_check_ == nullptr) {
    return;
  }
  smart_teleop_check_->setEnabled(available);
  if (!available) {
    smart_teleop_check_->setToolTip(
        tr("Requires building with autonomy (task teleop proto)"));
  } else {
    smart_teleop_check_->setToolTip(
        tr("Send velocity via autonomy task teleop (MPPI assist), not /cmd_vel"));
  }
}

void TeleopControlWidget::setSmartTeleopStatusText(const QString& status) {
  smart_teleop_status_text_ = status;
  updateSmartTeleopHint();
}

void TeleopControlWidget::updateSmartTeleopHint() {
  if (hint_label_ == nullptr) {
    return;
  }
  QString tip;
  if (stick_mode_ == TeleopStickMode::kArcade) {
    tip = tr("WASD / arrows · Space: E-Stop");
  } else {
    tip = tr("Left: Move · Right: Turn");
  }
  if (smart_teleop_enabled_) {
    tip += tr(" · Smart %1")
               .arg(QString::fromUtf8(integration::kTeleopGoalChannel));
    if (!smart_teleop_status_text_.isEmpty()) {
      tip += tr(" · %1").arg(smart_teleop_status_text_);
    }
  }
  hint_label_->setText(tip);
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
    updateSmartTeleopHint();
  }
  if (arcade) {
    setFocus(Qt::OtherFocusReason);
  }
}

void TeleopControlWidget::emitArcadeFromStick(double x, double y, bool released) {
  emit linearChanged(0.0, y);
  emit angularChanged(-x);
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
