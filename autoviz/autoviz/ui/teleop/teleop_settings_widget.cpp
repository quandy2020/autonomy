/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/teleop/teleop_settings_widget.hpp"

#include <QAbstractSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace teleop {
namespace {

QComboBox* MakeFieldCombo(QWidget* parent) {
  auto* combo = new QComboBox(parent);
  combo->addItem(QStringLiteral("linear.x"),
                 static_cast<int>(TeleopTwistField::kLinearX));
  combo->addItem(QStringLiteral("linear.y"),
                 static_cast<int>(TeleopTwistField::kLinearY));
  combo->addItem(QStringLiteral("linear.z"),
                 static_cast<int>(TeleopTwistField::kLinearZ));
  combo->addItem(QStringLiteral("angular.x"),
                 static_cast<int>(TeleopTwistField::kAngularX));
  combo->addItem(QStringLiteral("angular.y"),
                 static_cast<int>(TeleopTwistField::kAngularY));
  combo->addItem(QStringLiteral("angular.z"),
                 static_cast<int>(TeleopTwistField::kAngularZ));
  return combo;
}

void SetFieldCombo(QComboBox* combo, TeleopTwistField field) {
  if (combo == nullptr) {
    return;
  }
  const int index = combo->findData(static_cast<int>(field));
  if (index >= 0) {
    combo->setCurrentIndex(index);
  }
}

QLabel* MakeFormLabel(const QString& text, QWidget* parent) {
  auto* label = new QLabel(text, parent);
  label->setStyleSheet(
      style::sheet(QStringLiteral("teleop/settings_form_label")));
  return label;
}

void StylePlainSpin(QDoubleSpinBox* spin) {
  if (spin == nullptr) {
    return;
  }
  spin->setButtonSymbols(QAbstractSpinBox::NoButtons);
  spin->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
}

QWidget* MakeSpeedField(const QString& caption, const QString& tip,
                        QDoubleSpinBox** spin_out, double value,
                        const QString& suffix, QWidget* parent) {
  auto* card = new QFrame(parent);
  card->setObjectName(QStringLiteral("TeleopSettingsSpeedCard"));
  card->setStyleSheet(
      style::sheet(QStringLiteral("teleop/settings_speed_card")));

  auto* layout = new QVBoxLayout(card);
  layout->setContentsMargins(8, 6, 8, 6);
  layout->setSpacing(2);

  auto* caption_label = new QLabel(caption, card);
  caption_label->setStyleSheet(
      style::sheet(QStringLiteral("teleop/settings_speed_caption")));
  layout->addWidget(caption_label);

  auto* spin = new QDoubleSpinBox(card);
  spin->setRange(0.01, 10.0);
  spin->setSingleStep(0.05);
  spin->setDecimals(2);
  spin->setSuffix(suffix);
  spin->setValue(value);
  spin->setToolTip(tip);
  spin->setProperty("autoviz_skip_settings_polish", true);
  StylePlainSpin(spin);
  spin->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  spin->setStyleSheet(
      style::sheet(QStringLiteral("teleop/settings_speed_spin")));
  layout->addWidget(spin);
  *spin_out = spin;
  return card;
}

}  // namespace

TeleopSettingsWidget::TeleopSettingsWidget(common::VisualizationManager* manager,
                                           QWidget* parent)
    : QWidget(parent), manager_(manager), config_(DefaultTeleopPanelConfig()) {
  ApplyCompactSettingsShell(this);

  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(PanelSettingsLayout::kOuterMargin,
                            PanelSettingsLayout::kOuterMargin,
                            PanelSettingsLayout::kOuterMargin,
                            PanelSettingsLayout::kOuterMargin);
  outer->setSpacing(PanelSettingsLayout::kOuterSpacing);
  outer->setAlignment(Qt::AlignTop);

  // —— Panel ——
  auto* panel_group = new QGroupBox(tr("Panel"), this);
  StyleSettingsGroupBox(panel_group);
  auto* panel_form = new QFormLayout(panel_group);
  ApplyCompactForm(panel_form);
  title_edit_ = new QLineEdit(config_.title, panel_group);
  title_edit_->setPlaceholderText(tr("Teleop"));
  panel_form->addRow(MakeFormLabel(tr("Title"), panel_group), title_edit_);
  outer->addWidget(panel_group);

  // —— Publish ——
  auto* publish_group = new QGroupBox(tr("Publish"), this);
  StyleSettingsGroupBox(publish_group);
  auto* publish_form = new QFormLayout(publish_group);
  ApplyCompactForm(publish_form);
  topic_edit_ = new QLineEdit(config_.topic, publish_group);
  topic_edit_->setPlaceholderText(QStringLiteral("/cmd_vel"));
  publish_form->addRow(MakeFormLabel(tr("Topic"), publish_group), topic_edit_);
  publish_rate_spin_ = new QDoubleSpinBox(publish_group);
  publish_rate_spin_->setRange(0.1, 100.0);
  publish_rate_spin_->setSingleStep(0.5);
  publish_rate_spin_->setDecimals(1);
  publish_rate_spin_->setSuffix(QStringLiteral(" Hz"));
  publish_rate_spin_->setValue(config_.publish_rate_hz);
  StylePlainSpin(publish_rate_spin_);
  publish_form->addRow(MakeFormLabel(tr("Rate"), publish_group),
                       publish_rate_spin_);

  stop_on_release_check_ =
      new QCheckBox(tr("Stop on stick release"), publish_group);
  stop_on_release_check_->setChecked(config_.stop_on_release);
  stop_on_release_check_->setStyleSheet(
      style::sheet(QStringLiteral("teleop/settings_form_check")));
  publish_form->addRow(QString(), stop_on_release_check_);

  smart_teleop_check_ =
      new QCheckBox(tr("Smart teleop (task assist)"), publish_group);
  smart_teleop_check_->setChecked(config_.smart_teleop_enabled);
  smart_teleop_check_->setToolTip(
      tr("Send velocity via autonomy task teleop instead of /cmd_vel"));
  smart_teleop_check_->setStyleSheet(
      style::sheet(QStringLiteral("teleop/settings_form_check")));
  publish_form->addRow(QString(), smart_teleop_check_);
  outer->addWidget(publish_group);

  // —— Control ——
  auto* control_group = new QGroupBox(tr("Control"), this);
  StyleSettingsGroupBox(control_group);
  auto* control_form = new QFormLayout(control_group);
  ApplyCompactForm(control_form);
  stick_mode_combo_ = new QComboBox(control_group);
  stick_mode_combo_->addItem(tr("Dual sticks"),
                             static_cast<int>(TeleopStickMode::kDual));
  stick_mode_combo_->addItem(tr("Arcade + keyboard"),
                             static_cast<int>(TeleopStickMode::kArcade));
  stick_mode_combo_->setCurrentIndex(
      stick_mode_combo_->findData(static_cast<int>(config_.stick_mode)));
  control_form->addRow(MakeFormLabel(tr("Mode"), control_group),
                       stick_mode_combo_);

  auto* speed_row = new QWidget(control_group);
  auto* speed_layout = new QHBoxLayout(speed_row);
  speed_layout->setContentsMargins(0, 0, 0, 0);
  speed_layout->setSpacing(8);
  speed_layout->addWidget(
      MakeSpeedField(tr("LINEAR"), tr("Max linear speed"), &max_linear_spin_,
                     config_.max_linear_speed, QStringLiteral(" m/s"),
                     speed_row),
      1);
  speed_layout->addWidget(
      MakeSpeedField(tr("ANGULAR"), tr("Max angular speed"), &max_angular_spin_,
                     config_.max_angular_speed, QStringLiteral(" rad/s"),
                     speed_row),
      1);
  control_form->addRow(MakeFormLabel(tr("Max speeds"), control_group),
                       speed_row);
  outer->addWidget(control_group);

  // —— Axis mapping ——
  auto* mapping_body = new QWidget(this);
  auto* mapping_layout = new QVBoxLayout(mapping_body);
  mapping_layout->setContentsMargins(0, 2, 0, 0);
  mapping_layout->setSpacing(4);

  auto* header = new QWidget(mapping_body);
  auto* header_row = new QHBoxLayout(header);
  header_row->setContentsMargins(6, 0, 6, 0);
  header_row->setSpacing(6);
  auto* dir_h = new QLabel(tr("AXIS"), header);
  auto* field_h = new QLabel(tr("TWIST FIELD"), header);
  auto* value_h = new QLabel(tr("VALUE"), header);
  const QString head_css =
      style::sheet(QStringLiteral("teleop/mapping_header"));
  dir_h->setStyleSheet(head_css);
  field_h->setStyleSheet(head_css);
  value_h->setStyleSheet(head_css);
  header_row->addWidget(dir_h, 2);
  header_row->addWidget(field_h, 5);
  header_row->addWidget(value_h, 3);
  mapping_layout->addWidget(header);

  addAxisRow(mapping_layout, tr("Up"), &up_field_, &up_value_, config_.up);
  addAxisRow(mapping_layout, tr("Down"), &down_field_, &down_value_,
             config_.down);
  addAxisRow(mapping_layout, tr("Left"), &left_field_, &left_value_,
             config_.left);
  addAxisRow(mapping_layout, tr("Right"), &right_field_, &right_value_,
             config_.right);
  addAxisRow(mapping_layout, tr("Stop"), &stop_field_, &stop_value_,
             config_.stop);

  outer->addWidget(
      MakeCollapsibleSection(this, tr("Axis mapping"), mapping_body, false));
  outer->addStretch(1);

  connect(title_edit_, &QLineEdit::textChanged, this,
          &TeleopSettingsWidget::emitConfigChanged);
  connect(topic_edit_, &QLineEdit::textChanged, this,
          &TeleopSettingsWidget::emitConfigChanged);
  connect(publish_rate_spin_,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &TeleopSettingsWidget::emitConfigChanged);
  connect(stop_on_release_check_, &QCheckBox::toggled, this,
          &TeleopSettingsWidget::emitConfigChanged);
  connect(smart_teleop_check_, &QCheckBox::toggled, this,
          &TeleopSettingsWidget::emitConfigChanged);
  connect(stick_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &TeleopSettingsWidget::emitConfigChanged);
  connect(max_linear_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &TeleopSettingsWidget::emitConfigChanged);
  connect(max_angular_spin_,
          QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &TeleopSettingsWidget::emitConfigChanged);
}

void TeleopSettingsWidget::addAxisRow(QVBoxLayout* layout, const QString& name,
                                      QComboBox** field_out,
                                      QDoubleSpinBox** value_out,
                                      const TeleopButtonConfig& seed) {
  auto* row = new QFrame(this);
  row->setObjectName(QStringLiteral("TeleopAxisRow"));
  row->setStyleSheet(
      style::sheet(QStringLiteral("teleop/axis_row")));

  auto* row_layout = new QHBoxLayout(row);
  row_layout->setContentsMargins(8, 4, 6, 4);
  row_layout->setSpacing(6);

  auto* name_label = new QLabel(name, row);
  name_label->setStyleSheet(
      style::sheet(QStringLiteral("teleop/axis_name")));
  name_label->setMinimumWidth(40);
  row_layout->addWidget(name_label, 2);

  auto* field = MakeFieldCombo(row);
  SetFieldCombo(field, seed.field);
  field->setStyleSheet(
      style::sheet(QStringLiteral("teleop/axis_combo")));
  row_layout->addWidget(field, 5);

  auto* value = new QDoubleSpinBox(row);
  value->setRange(-1000.0, 1000.0);
  value->setSingleStep(0.05);
  value->setDecimals(3);
  value->setValue(seed.value);
  StylePlainSpin(value);
  value->setStyleSheet(
      style::sheet(QStringLiteral("teleop/axis_spin")));
  row_layout->addWidget(value, 3);

  *field_out = field;
  *value_out = value;
  layout->addWidget(row);

  connect(field, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &TeleopSettingsWidget::emitConfigChanged);
  connect(value, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &TeleopSettingsWidget::emitConfigChanged);
}

void TeleopSettingsWidget::syncAxisEditors() {
  const QSignalBlocker b_up_f(up_field_);
  const QSignalBlocker b_dn_f(down_field_);
  const QSignalBlocker b_lf_f(left_field_);
  const QSignalBlocker b_rt_f(right_field_);
  const QSignalBlocker b_st_f(stop_field_);
  const QSignalBlocker b_up_v(up_value_);
  const QSignalBlocker b_dn_v(down_value_);
  const QSignalBlocker b_lf_v(left_value_);
  const QSignalBlocker b_rt_v(right_value_);
  const QSignalBlocker b_st_v(stop_value_);

  SetFieldCombo(up_field_, config_.up.field);
  SetFieldCombo(down_field_, config_.down.field);
  SetFieldCombo(left_field_, config_.left.field);
  SetFieldCombo(right_field_, config_.right.field);
  SetFieldCombo(stop_field_, config_.stop.field);
  if (up_value_ != nullptr) {
    up_value_->setValue(config_.up.value);
  }
  if (down_value_ != nullptr) {
    down_value_->setValue(config_.down.value);
  }
  if (left_value_ != nullptr) {
    left_value_->setValue(config_.left.value);
  }
  if (right_value_ != nullptr) {
    right_value_->setValue(config_.right.value);
  }
  if (stop_value_ != nullptr) {
    stop_value_->setValue(config_.stop.value);
  }
}

TeleopPanelConfig TeleopSettingsWidget::config() const {
  TeleopPanelConfig out = config_;
  out.title = title_edit_->text().trimmed();
  out.topic = topic_edit_->text().trimmed();
  out.publish_rate_hz = publish_rate_spin_->value();
  out.stop_on_release = stop_on_release_check_->isChecked();
  out.smart_teleop_enabled = smart_teleop_check_->isChecked();
  out.stick_mode =
      static_cast<TeleopStickMode>(stick_mode_combo_->currentData().toInt());
  out.max_linear_speed = max_linear_spin_->value();
  out.max_angular_speed = max_angular_spin_->value();

  const auto read = [](QComboBox* field, QDoubleSpinBox* value,
                       TeleopButtonConfig* target) {
    if (field == nullptr || value == nullptr || target == nullptr) {
      return;
    }
    target->field = static_cast<TeleopTwistField>(field->currentData().toInt());
    target->value = value->value();
  };
  read(up_field_, up_value_, &out.up);
  read(down_field_, down_value_, &out.down);
  read(left_field_, left_value_, &out.left);
  read(right_field_, right_value_, &out.right);
  read(stop_field_, stop_value_, &out.stop);
  return out;
}

void TeleopSettingsWidget::setConfig(const TeleopPanelConfig& config) {
  config_ = config;
  const QSignalBlocker b1(title_edit_);
  const QSignalBlocker b2(topic_edit_);
  const QSignalBlocker b3(publish_rate_spin_);
  const QSignalBlocker b4(stop_on_release_check_);
  const QSignalBlocker b5(smart_teleop_check_);
  const QSignalBlocker b6(stick_mode_combo_);
  const QSignalBlocker b7(max_linear_spin_);
  const QSignalBlocker b8(max_angular_spin_);

  title_edit_->setText(config_.title);
  topic_edit_->setText(config_.topic);
  publish_rate_spin_->setValue(config_.publish_rate_hz);
  stop_on_release_check_->setChecked(config_.stop_on_release);
  smart_teleop_check_->setChecked(config_.smart_teleop_enabled);
  stick_mode_combo_->setCurrentIndex(
      stick_mode_combo_->findData(static_cast<int>(config_.stick_mode)));
  max_linear_spin_->setValue(config_.max_linear_speed);
  max_angular_spin_->setValue(config_.max_angular_speed);
  syncAxisEditors();
}

void TeleopSettingsWidget::emitConfigChanged() {
  config_ = config();
  emit configChanged();
}

}  // namespace teleop
}  // namespace autoviz
