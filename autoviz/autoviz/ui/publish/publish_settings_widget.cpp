/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/publish/publish_settings_widget.hpp"

#include <QColorDialog>
#include <QFormLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include "autoviz/ui/theme/panel.hpp"

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

  auto* general_body = new QWidget(this);
  auto* title_form = new QFormLayout(general_body);
  ApplyCompactForm(title_form);
  title_edit_ = new QLineEdit(config_.title, general_body);
  title_form->addRow(tr("Title"), title_edit_);
  outer->addWidget(
      MakeCollapsibleSection(this, tr("General"), general_body, true));

  auto* button_body = new QWidget(this);
  auto* form = new QFormLayout(button_body);
  ApplyCompactForm(form);
  button_label_edit_ = new QLineEdit(config_.button_label, button_body);
  button_tooltip_edit_ = new QLineEdit(config_.button_tooltip, button_body);
  button_color_button_ = MakeFlatActionButton(tr("Choose color"), button_body);
  form->addRow(tr("Label"), button_label_edit_);
  form->addRow(tr("Tooltip"), button_tooltip_edit_);
  form->addRow(tr("Color"), button_color_button_);
  outer->addWidget(
      MakeCollapsibleSection(this, tr("Send button"), button_body, true));
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
