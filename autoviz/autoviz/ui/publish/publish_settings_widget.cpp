/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/publish/publish_settings_widget.hpp"

#include <QColorDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

namespace autoviz {
namespace publish_panel {

PublishSettingsWidget::PublishSettingsWidget(QWidget* parent)
    : config_(DefaultPublishPanelConfig()), QWidget(parent) {
  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(8, 8, 8, 8);
  outer->setSpacing(8);

  auto* title_row = new QHBoxLayout();
  title_row->addWidget(new QLabel(tr("Title"), this));
  title_edit_ = new QLineEdit(config_.title, this);
  title_row->addWidget(title_edit_, 1);
  outer->addLayout(title_row);

  auto* button_group = new QGroupBox(tr("Publish button"), this);
  auto* form = new QFormLayout(button_group);
  button_label_edit_ = new QLineEdit(config_.button_label, button_group);
  button_tooltip_edit_ = new QLineEdit(config_.button_tooltip, button_group);
  button_color_button_ = new QPushButton(button_group);
  button_color_button_->setText(tr("Choose color"));
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
  config_ = config;
  title_edit_->setText(config_.title);
  button_label_edit_->setText(config_.button_label);
  button_tooltip_edit_->setText(config_.button_tooltip);
  if (config_.button_color.isValid()) {
    button_color_button_->setStyleSheet(
        QStringLiteral("background: %1;").arg(config_.button_color.name()));
  } else {
    button_color_button_->setStyleSheet(QString());
  }
}

void PublishSettingsWidget::pickButtonColor() {
  const QColor chosen = QColorDialog::getColor(
      config_.button_color.isValid() ? config_.button_color : QColor(70, 120, 200),
      this, tr("Publish button color"));
  if (!chosen.isValid()) {
    return;
  }
  config_.button_color = chosen;
  button_color_button_->setStyleSheet(
      QStringLiteral("background: %1;").arg(chosen.name()));
  emitConfigChanged();
}

void PublishSettingsWidget::emitConfigChanged() { emit configChanged(); }

}  // namespace publish_panel
}  // namespace autoviz
