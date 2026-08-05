/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/table/table_settings_widget.hpp"

#include <QComboBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLineEdit>
#include <QVBoxLayout>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {
namespace table {
namespace {

QStringList AllChannels(common::VisualizationManager* manager) {
  QStringList channels;
  if (manager == nullptr) {
    return channels;
  }
  for (const integration::ChannelInfo& info : manager->channels()) {
    channels.push_back(QString::fromStdString(info.channel_name));
  }
  channels.sort(Qt::CaseInsensitive);
  return channels;
}

}  // namespace

TableSettingsWidget::TableSettingsWidget(common::VisualizationManager* manager,
                                         QWidget* parent)
    : manager_(manager), config_(DefaultTablePanelConfig()), QWidget(parent) {
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

  auto* general = new QGroupBox(tr("Data source"), this);
  StyleSettingsGroupBox(general);
  auto* form = new QFormLayout(general);
  ApplyCompactForm(form);
  channel_combo_ = new QComboBox(general);
  channel_combo_->setEditable(true);
  array_path_edit_ = new QLineEdit(config_.array_path, general);
  array_path_edit_->setPlaceholderText(
      tr("Array path, e.g. objects[:]{id==$vehicle_id}"));
  form->addRow(tr("Channel"), channel_combo_);
  form->addRow(tr("Array path"), array_path_edit_);
  outer->addWidget(general);
  outer->addStretch();

  connect(title_edit_, &QLineEdit::textChanged, this,
          &TableSettingsWidget::emitConfigChanged);
  connect(channel_combo_, &QComboBox::currentTextChanged, this,
          &TableSettingsWidget::emitConfigChanged);
  connect(array_path_edit_, &QLineEdit::textChanged, this,
          &TableSettingsWidget::emitConfigChanged);

  refreshChannels();
  setConfig(config_);
}

TablePanelConfig TableSettingsWidget::config() const {
  TablePanelConfig out = config_;
  out.title = title_edit_->text().trimmed();
  out.channel = channel_combo_->currentText().trimmed();
  out.array_path = array_path_edit_->text().trimmed();
  return out;
}

void TableSettingsWidget::setConfig(const TablePanelConfig& config) {
  config_ = config;
  title_edit_->setText(config_.title);
  array_path_edit_->setText(config_.array_path);
  const int channel_index = channel_combo_->findText(config_.channel);
  if (channel_index >= 0) {
    channel_combo_->setCurrentIndex(channel_index);
  } else {
    channel_combo_->setEditText(config_.channel);
  }
}

void TableSettingsWidget::refreshChannels() {
  const QString current = channel_combo_->currentText();
  channel_combo_->blockSignals(true);
  channel_combo_->clear();
  channel_combo_->addItems(AllChannels(manager_));
  const int index = channel_combo_->findText(current);
  if (index >= 0) {
    channel_combo_->setCurrentIndex(index);
  } else if (!current.isEmpty()) {
    channel_combo_->setEditText(current);
  }
  channel_combo_->blockSignals(false);
}

void TableSettingsWidget::emitConfigChanged() { emit configChanged(); }

}  // namespace table
}  // namespace autoviz
