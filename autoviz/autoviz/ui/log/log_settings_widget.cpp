/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/log/log_parser.hpp"
#include "autoviz/ui/log/log_settings_widget.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>

namespace autoviz {
namespace log_panel {
namespace {

QStringList LogChannels(common::VisualizationManager* manager) {
  QStringList channels;
  if (manager == nullptr) {
    return channels;
  }
  for (const integration::ChannelInfo& info : manager->channels()) {
    if (isLogMessageType(info.message_type)) {
      channels.push_back(QString::fromStdString(info.channel_name));
    }
  }
  channels.sort(Qt::CaseInsensitive);
  return channels;
}

}  // namespace

LogSettingsWidget::LogSettingsWidget(common::VisualizationManager* manager,
                                     QWidget* parent)
    : manager_(manager), config_(DefaultLogPanelConfig()), QWidget(parent) {
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
  topic_edit_->setPlaceholderText(tr("/rosout (auto-discovers Log channels)"));
  general_form->addRow(tr("Topic"), topic_edit_);
  min_level_combo_ = new QComboBox(general_body);
  min_level_combo_->addItem(tr("DEBUG"), static_cast<int>(LogLevel::kDebug));
  min_level_combo_->addItem(tr("INFO"), static_cast<int>(LogLevel::kInfo));
  min_level_combo_->addItem(tr("WARN"), static_cast<int>(LogLevel::kWarn));
  min_level_combo_->addItem(tr("ERROR"), static_cast<int>(LogLevel::kError));
  min_level_combo_->addItem(tr("FATAL"), static_cast<int>(LogLevel::kFatal));
  general_form->addRow(tr("Log level"), min_level_combo_);
  font_size_spin_ = new QDoubleSpinBox(general_body);
  font_size_spin_->setRange(8.0, 24.0);
  font_size_spin_->setSingleStep(1.0);
  font_size_spin_->setValue(config_.font_size);
  general_form->addRow(tr("Font size"), font_size_spin_);
  capture_glog_check_ = new QCheckBox(tr("Capture glog output"), general_body);
  capture_glog_check_->setChecked(config_.capture_glog);
  general_form->addRow(QString(), capture_glog_check_);
  auto_subscribe_check_ =
      new QCheckBox(tr("Auto-subscribe foxglove.Log channels"), general_body);
  auto_subscribe_check_->setChecked(config_.auto_subscribe_log_channels);
  auto_subscribe_check_->setToolTip(
      tr("Also listen on /rosout and every discovered foxglove.Log channel"));
  general_form->addRow(QString(), auto_subscribe_check_);
  follow_playback_check_ =
      new QCheckBox(tr("Follow playback time"), general_body);
  follow_playback_check_->setChecked(config_.follow_playback);
  general_form->addRow(QString(), follow_playback_check_);
  outer->addWidget(MakeCollapsibleSection(this, tr("General"), general_body, true));

  auto* namespace_body = new QWidget(this);
  namespace_layout_ = new QVBoxLayout(namespace_body);
  rebuildNamespaceSection();
  outer->addWidget(
      MakeCollapsibleSection(this, tr("Namespaces"), namespace_body, true));
  outer->addStretch();

  connect(title_edit_, &QLineEdit::textChanged, this, &LogSettingsWidget::emitConfigChanged);
  connect(topic_edit_, &QLineEdit::textChanged, this, &LogSettingsWidget::emitConfigChanged);
  connect(min_level_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &LogSettingsWidget::emitConfigChanged);
  connect(font_size_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &LogSettingsWidget::emitConfigChanged);
  connect(capture_glog_check_, &QCheckBox::toggled, this,
          &LogSettingsWidget::emitConfigChanged);
  connect(auto_subscribe_check_, &QCheckBox::toggled, this,
          &LogSettingsWidget::emitConfigChanged);
  connect(follow_playback_check_, &QCheckBox::toggled, this,
          &LogSettingsWidget::emitConfigChanged);

  setConfig(config_);
}

void LogSettingsWidget::setKnownNamespaces(const QSet<QString>& namespaces) {
  known_namespaces_ = namespaces;
  rebuildNamespaceSection();
}

void LogSettingsWidget::rebuildNamespaceSection() {
  while (QLayoutItem* item = namespace_layout_->takeAt(0)) {
    if (item->widget() != nullptr) {
      item->widget()->deleteLater();
    }
    delete item;
  }
  if (known_namespaces_.isEmpty()) {
    auto* hint = new QLabel(tr("Namespaces appear as logs arrive."), this);
    hint->setStyleSheet(PropertyInspectorHintStyle());
    namespace_layout_->addWidget(hint);
    return;
  }
  QStringList sorted = known_namespaces_.values();
  sorted.sort(Qt::CaseInsensitive);
  for (const QString& name : sorted) {
    auto* check = new QCheckBox(name, this);
    check->setChecked(true);
    check->setProperty("namespaceName", name);
    namespace_layout_->addWidget(check);
    connect(check, &QCheckBox::toggled, this, &LogSettingsWidget::emitConfigChanged);
  }
}

LogPanelConfig LogSettingsWidget::config() const {
  LogPanelConfig out = config_;
  out.title = title_edit_->text().trimmed();
  out.topic = topic_edit_->text().trimmed();
  out.min_level = static_cast<LogLevel>(min_level_combo_->currentData().toInt());
  out.font_size = font_size_spin_->value();
  out.capture_glog = capture_glog_check_->isChecked();
  out.auto_subscribe_log_channels = auto_subscribe_check_->isChecked();
  out.follow_playback = follow_playback_check_->isChecked();
  return out;
}

void LogSettingsWidget::setConfig(const LogPanelConfig& config) {
  config_ = config;
  title_edit_->setText(config_.title);
  topic_edit_->setText(config_.topic);
  min_level_combo_->setCurrentIndex(
      min_level_combo_->findData(static_cast<int>(config_.min_level)));
  font_size_spin_->setValue(config_.font_size);
  capture_glog_check_->setChecked(config_.capture_glog);
  auto_subscribe_check_->setChecked(config_.auto_subscribe_log_channels);
  follow_playback_check_->setChecked(config_.follow_playback);
}

void LogSettingsWidget::emitConfigChanged() {
  config_ = config();
  emit configChanged();
}

}  // namespace log_panel
}  // namespace autoviz
