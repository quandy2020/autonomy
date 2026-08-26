/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/teleop/teleop_settings_widget.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>

#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/ui/teleop/teleop_twist_utils.hpp"

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

}  // namespace

TeleopSettingsWidget::TeleopSettingsWidget(common::VisualizationManager* manager,
                                           QWidget* parent)
    : manager_(manager), config_(DefaultTeleopPanelConfig()), QWidget(parent) {
  ApplyCompactSettingsShell(this);
  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin,
                            PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin);
  outer->setSpacing(PanelSettingsLayout::kOuterSpacing);
  outer->setAlignment(Qt::AlignTop);

  auto* title_form = new QFormLayout();
  ApplyCompactForm(title_form);
  title_edit_ = new QLineEdit(config_.title, this);
  title_form->addRow(tr("Title"), title_edit_);
  outer->addLayout(title_form);

  auto* general_body = new QWidget(this);
  auto* general_form = new QFormLayout(general_body);
  ApplyCompactForm(general_form);
  topic_edit_ = new QLineEdit(config_.topic, general_body);
  topic_edit_->setPlaceholderText(QStringLiteral("/cmd_vel"));
  general_form->addRow(tr("Topic"), topic_edit_);
  publish_rate_spin_ = new QDoubleSpinBox(general_body);
  publish_rate_spin_->setRange(0.1, 100.0);
  publish_rate_spin_->setSingleStep(0.5);
  publish_rate_spin_->setDecimals(1);
  publish_rate_spin_->setValue(config_.publish_rate_hz);
  general_form->addRow(tr("Publish rate (Hz)"), publish_rate_spin_);
  stop_on_release_check_ = new QCheckBox(tr("Stop on release"), general_body);
  stop_on_release_check_->setChecked(config_.stop_on_release);
  general_form->addRow(QString(), stop_on_release_check_);
  stick_mode_combo_ = new QComboBox(general_body);
  stick_mode_combo_->addItem(tr("Dual sticks (Move + Turn)"),
                             static_cast<int>(TeleopStickMode::kDual));
  stick_mode_combo_->addItem(tr("Arcade (one stick + keyboard)"),
                             static_cast<int>(TeleopStickMode::kArcade));
  stick_mode_combo_->setCurrentIndex(
      stick_mode_combo_->findData(static_cast<int>(config_.stick_mode)));
  general_form->addRow(tr("Control mode"), stick_mode_combo_);
  max_linear_spin_ = new QDoubleSpinBox(general_body);
  max_linear_spin_->setRange(0.01, 10.0);
  max_linear_spin_->setSingleStep(0.05);
  max_linear_spin_->setDecimals(2);
  max_linear_spin_->setSuffix(QStringLiteral(" m/s"));
  max_linear_spin_->setValue(config_.max_linear_speed);
  general_form->addRow(tr("Max linear speed"), max_linear_spin_);
  max_angular_spin_ = new QDoubleSpinBox(general_body);
  max_angular_spin_->setRange(0.01, 10.0);
  max_angular_spin_->setSingleStep(0.05);
  max_angular_spin_->setDecimals(2);
  max_angular_spin_->setSuffix(QStringLiteral(" rad/s"));
  max_angular_spin_->setValue(config_.max_angular_speed);
  general_form->addRow(tr("Max angular speed"), max_angular_spin_);
  outer->addWidget(MakeCollapsibleSection(this, tr("General"), general_body, true));

  auto* buttons_body = new QWidget(this);
  button_sections_layout_ = new QVBoxLayout(buttons_body);
  up_config_ = &config_.up;
  down_config_ = &config_.down;
  left_config_ = &config_.left;
  right_config_ = &config_.right;
  stop_config_ = &config_.stop;
  rebuildButtonSections();
  outer->addWidget(
      MakeCollapsibleSection(this, tr("Button behavior"), buttons_body, true));
  outer->addStretch();

  connect(title_edit_, &QLineEdit::textChanged, this,
          &TeleopSettingsWidget::emitConfigChanged);
  connect(topic_edit_, &QLineEdit::textChanged, this,
          &TeleopSettingsWidget::emitConfigChanged);
  connect(publish_rate_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &TeleopSettingsWidget::emitConfigChanged);
  connect(stop_on_release_check_, &QCheckBox::toggled, this,
          &TeleopSettingsWidget::emitConfigChanged);
  connect(stick_mode_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &TeleopSettingsWidget::emitConfigChanged);
  connect(max_linear_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &TeleopSettingsWidget::emitConfigChanged);
  connect(max_angular_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &TeleopSettingsWidget::emitConfigChanged);
}

QWidget* TeleopSettingsWidget::makeButtonSection(const QString& title,
                                                 TeleopButtonConfig* target) {
  auto* row = new QWidget(this);
  auto* form = new QFormLayout(row);
  ApplyCompactForm(form);
  auto* field = MakeFieldCombo(row);
  field->setCurrentIndex(field->findData(static_cast<int>(target->field)));
  auto* value = new QDoubleSpinBox(row);
  value->setRange(-1000.0, 1000.0);
  value->setSingleStep(0.05);
  value->setDecimals(3);
  value->setValue(target->value);
  form->addRow(tr("Field"), field);
  form->addRow(tr("Value"), value);

  connect(field, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &TeleopSettingsWidget::emitConfigChanged);
  connect(value, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &TeleopSettingsWidget::emitConfigChanged);
  row->setProperty("buttonSectionTitle", title);
  return MakeCollapsibleSection(this, title, row, false);
}

void TeleopSettingsWidget::rebuildButtonSections() {
  while (QLayoutItem* item = button_sections_layout_->takeAt(0)) {
    if (item->widget() != nullptr) {
      item->widget()->deleteLater();
    }
    delete item;
  }
  button_sections_layout_->addWidget(makeButtonSection(tr("Up button"), up_config_));
  button_sections_layout_->addWidget(
      makeButtonSection(tr("Down button"), down_config_));
  button_sections_layout_->addWidget(
      makeButtonSection(tr("Left button"), left_config_));
  button_sections_layout_->addWidget(
      makeButtonSection(tr("Right button"), right_config_));
  button_sections_layout_->addWidget(
      makeButtonSection(tr("Stop button"), stop_config_));
}

TeleopPanelConfig TeleopSettingsWidget::config() const {
  TeleopPanelConfig out = config_;
  out.title = title_edit_->text().trimmed();
  out.topic = topic_edit_->text().trimmed();
  out.publish_rate_hz = publish_rate_spin_->value();
  out.stop_on_release = stop_on_release_check_->isChecked();
  out.stick_mode =
      static_cast<TeleopStickMode>(stick_mode_combo_->currentData().toInt());
  out.max_linear_speed = max_linear_spin_->value();
  out.max_angular_speed = max_angular_spin_->value();
  out.up.value = out.max_linear_speed;
  out.down.value = -out.max_linear_speed;
  out.left.value = out.max_angular_speed;
  out.right.value = -out.max_angular_speed;

  const auto readSection = [&](TeleopButtonConfig* target, int section_index) {
    auto* section = button_sections_layout_->itemAt(section_index)->widget();
    if (section == nullptr) {
      return;
    }
    const auto combos = section->findChildren<QComboBox*>();
    const auto spins = section->findChildren<QDoubleSpinBox*>();
    if (combos.isEmpty() || spins.isEmpty()) {
      return;
    }
    target->field =
        static_cast<TeleopTwistField>(combos.first()->currentData().toInt());
    target->value = spins.first()->value();
  };

  readSection(&out.up, 0);
  readSection(&out.down, 1);
  readSection(&out.left, 2);
  readSection(&out.right, 3);
  readSection(&out.stop, 4);
  return out;
}

void TeleopSettingsWidget::setConfig(const TeleopPanelConfig& config) {
  config_ = config;
  title_edit_->setText(config_.title);
  topic_edit_->setText(config_.topic);
  publish_rate_spin_->setValue(config_.publish_rate_hz);
  stop_on_release_check_->setChecked(config_.stop_on_release);
  stick_mode_combo_->setCurrentIndex(
      stick_mode_combo_->findData(static_cast<int>(config_.stick_mode)));
  max_linear_spin_->setValue(config_.max_linear_speed);
  max_angular_spin_->setValue(config_.max_angular_speed);
  rebuildButtonSections();
}

void TeleopSettingsWidget::emitConfigChanged() {
  config_ = config();
  emit configChanged();
}

}  // namespace teleop
}  // namespace autoviz
