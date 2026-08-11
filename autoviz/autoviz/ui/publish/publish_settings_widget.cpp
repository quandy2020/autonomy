/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/publish/publish_settings_widget.hpp"

#include <QColorDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QLineEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {
namespace publish_panel {

PublishSettingsWidget::PublishSettingsWidget(QWidget* parent)
    : config_(DefaultPublishPanelConfig()), QWidget(parent) {
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

  auto* button_group = new QGroupBox(tr("Send button"), this);
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
          &PublishSettingsWidget::emitConfigChanged);
  connect(button_label_edit_, &QLineEdit::textChanged, this,
          &PublishSettingsWidget::emitConfigChanged);
  connect(button_tooltip_edit_, &QLineEdit::textChanged, this,
          &PublishSettingsWidget::emitConfigChanged);
  connect(button_color_button_, &QPushButton::clicked, this,
          &PublishSettingsWidget::pickButtonColor);

  setConfig(config_);
}

PublishPanelConfig PublishSettingsWidget::config() const {
  PublishPanelConfig out = config_;
  out.title = title_edit_->text().trimmed();
  out.button_label = button_label_edit_->text().trimmed();
  out.button_tooltip = button_tooltip_edit_->text().trimmed();
  return out;
}

void PublishSettingsWidget::setConfig(const PublishPanelConfig& config) {
  const QSignalBlocker block_title(title_edit_);
  const QSignalBlocker block_label(button_label_edit_);
  const QSignalBlocker block_tooltip(button_tooltip_edit_);
  config_ = config;
  title_edit_->setText(config_.title);
  button_label_edit_->setText(config_.button_label);
  button_tooltip_edit_->setText(config_.button_tooltip);
  if (config_.button_color.isValid()) {
    UpdateColorButton(button_color_button_, config_.button_color);
  } else {
    UpdateColorButton(button_color_button_, QColor());
  }
}

void PublishSettingsWidget::pickButtonColor() {
  const QColor chosen = QColorDialog::getColor(
      config_.button_color.isValid() ? config_.button_color : QColor(70, 120, 200),
      this, tr("Send button color"));
  if (!chosen.isValid()) {
    return;
  }
  config_.button_color = chosen;
  UpdateColorButton(button_color_button_, chosen);
  emitConfigChanged();
}

void PublishSettingsWidget::emitConfigChanged() { emit configChanged(); }

}  // namespace publish_panel
}  // namespace autoviz
