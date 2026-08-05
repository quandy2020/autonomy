/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/table/table_settings_widget.hpp"

#include <QComboBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>

#include "autoviz/common/visualization_manager.hpp"

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
  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(8, 8, 8, 8);
  outer->setSpacing(8);

  auto* title_row = new QHBoxLayout();
  title_row->addWidget(new QLabel(tr("Title"), this));
  title_edit_ = new QLineEdit(config_.title, this);
  title_row->addWidget(title_edit_, 1);
  outer->addLayout(title_row);

  auto* general = new QGroupBox(tr("Data source"), this);
  auto* form = new QFormLayout(general);
  channel_combo_ = new QComboBox(general);
  channel_combo_->setEditable(true);
  array_path_edit_ = new QLineEdit(config_.array_path, general);
  array_path_edit_->setPlaceholderText(tr("Array path, e.g. markers or tracked_objects"));
  form->addRow(tr("Topic"), channel_combo_);
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
