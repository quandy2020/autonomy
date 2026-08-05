/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/service/service_settings_widget.hpp"

#include <QColorDialog>
#include <QComboBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {
namespace service_panel {

ServiceSettingsWidget::ServiceSettingsWidget(QWidget* parent)
    : config_(DefaultServiceCallPanelConfig()), QWidget(parent) {
  ApplyCompactSettingsShell(this);
  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin,
                            PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin);
  outer->setSpacing(PanelSettingsLayout::kOuterSpacing);
  outer->setAlignment(Qt::AlignTop);

  auto* general = new QGroupBox(tr("General"), this);
  StyleSettingsGroupBox(general);
  auto* general_form = new QFormLayout(general);
  ApplyCompactForm(general_form);
  title_edit_ = new QLineEdit(config_.title, general);
  timeout_spin_ = new QSpinBox(general);
  timeout_spin_->setRange(1, 300);
  timeout_spin_->setValue(config_.timeout_sec);
  layout_combo_ = new QComboBox(general);
  layout_combo_->addItem(tr("Vertical"), true);
  layout_combo_->addItem(tr("Horizontal"), false);
  general_form->addRow(tr("Title"), title_edit_);
  general_form->addRow(tr("Timeout (s)"), timeout_spin_);
  general_form->addRow(tr("Layout"), layout_combo_);
  outer->addWidget(general);

  auto* button_group = new QGroupBox(tr("Call button"), this);
  StyleSettingsGroupBox(button_group);
  auto* form = new QFormLayout(button_group);
  ApplyCompactForm(form);
  button_label_edit_ = new QLineEdit(config_.button_label, button_group);
  button_tooltip_edit_ = new QLineEdit(config_.button_tooltip, button_group);
  button_color_button_ = MakeFlatActionButton(tr("Choose color"), button_group);
  form->addRow(tr("Label"), button_label_edit_);
  form->addRow(tr("Tooltip"), button_tooltip_edit_);
  form->addRow(tr("Color"), button_color_button_);
  outer->addWidget(button_group);
  outer->addStretch();

  connect(title_edit_, &QLineEdit::textChanged, this,
          &ServiceSettingsWidget::emitConfigChanged);
  connect(timeout_spin_, qOverload<int>(&QSpinBox::valueChanged), this,
          &ServiceSettingsWidget::emitConfigChanged);
  connect(layout_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          &ServiceSettingsWidget::emitConfigChanged);
  connect(button_label_edit_, &QLineEdit::textChanged, this,
          &ServiceSettingsWidget::emitConfigChanged);
  connect(button_tooltip_edit_, &QLineEdit::textChanged, this,
          &ServiceSettingsWidget::emitConfigChanged);
  connect(button_color_button_, &QPushButton::clicked, this,
          &ServiceSettingsWidget::pickButtonColor);

  setConfig(config_);
}

ServiceCallPanelConfig ServiceSettingsWidget::config() const {
  ServiceCallPanelConfig out = config_;
  out.title = title_edit_->text().trimmed();
  out.timeout_sec = timeout_spin_->value();
  out.vertical_layout = layout_combo_->currentData().toBool();
  out.button_label = button_label_edit_->text().trimmed();
  out.button_tooltip = button_tooltip_edit_->text().trimmed();
  return out;
}

void ServiceSettingsWidget::setConfig(const ServiceCallPanelConfig& config) {
  config_ = config;
  title_edit_->setText(config_.title);
  timeout_spin_->setValue(std::max(1, config_.timeout_sec));
  const int layout_index = layout_combo_->findData(config_.vertical_layout);
  if (layout_index >= 0) {
    layout_combo_->setCurrentIndex(layout_index);
  }
  button_label_edit_->setText(config_.button_label);
  button_tooltip_edit_->setText(config_.button_tooltip);
  if (config_.button_color.isValid()) {
    UpdateColorButton(button_color_button_, config_.button_color);
  } else {
    UpdateColorButton(button_color_button_, QColor());
  }
}

void ServiceSettingsWidget::pickButtonColor() {
  const QColor chosen = QColorDialog::getColor(
      config_.button_color.isValid() ? config_.button_color : QColor(70, 120, 200),
      this, tr("Call button color"));
  if (!chosen.isValid()) {
    return;
  }
  config_.button_color = chosen;
  UpdateColorButton(button_color_button_, chosen);
  emitConfigChanged();
}

void ServiceSettingsWidget::emitConfigChanged() { emit configChanged(); }

}  // namespace service_panel
}  // namespace autoviz
