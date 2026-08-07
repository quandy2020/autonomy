/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/publish/publish_editor_widget.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QFont>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/channel_writer_registry.hpp"
#include "autoviz/ui/publish/publish_message_codec.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {
namespace publish_panel {
namespace {

QStringList KnownChannels(common::VisualizationManager* manager) {
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

QString ShortTypeLabel(const std::string& type) {
  QString label = QString::fromStdString(type);
  static const QString kPrefix = QStringLiteral("automsgs.msgs.");
  if (label.startsWith(kPrefix)) {
    label = label.mid(kPrefix.size());
  }
  return label;
}

}  // namespace

PublishEditorWidget::PublishEditorWidget(common::VisualizationManager* manager,
                                         QWidget* parent)
    : manager_(manager), config_(DefaultPublishPanelConfig()), QWidget(parent) {
  ApplyPanelShell(this);

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin,
                           PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin);
  root->setSpacing(PanelSettingsLayout::kOuterSpacing);

  auto* header = new QHBoxLayout();
  editing_mode_check_ = new QCheckBox(tr("Editing mode"), this);
  editing_mode_check_->setChecked(config_.editing_mode);
  header->addWidget(editing_mode_check_);
  header->addStretch();
  status_label_ = new QLabel(this);
  StylePanelStatusLabel(status_label_);
  status_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  header->addWidget(status_label_, 1);
  root->addLayout(header);

  auto* scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  editor_body_ = new QWidget(scroll);
  auto* form = new QFormLayout(editor_body_);
  form->setContentsMargins(0, 0, 0, 0);
  form->setSpacing(6);

  channel_combo_ = new QComboBox(editor_body_);
  channel_combo_->setEditable(true);
  channel_combo_->setInsertPolicy(QComboBox::NoInsert);
  form->addRow(tr("Channel"), channel_combo_);

  auto* type_row = new QHBoxLayout();
  message_type_combo_ = new QComboBox(editor_body_);
  message_type_combo_->setEditable(true);
  message_type_combo_->setInsertPolicy(QComboBox::NoInsert);
  reset_template_button_ = new QPushButton(tr("Reset template"), editor_body_);
  reset_template_button_->setFlat(true);
  type_row->addWidget(message_type_combo_, 1);
  type_row->addWidget(reset_template_button_);
  form->addRow(tr("Message type"), type_row);

  message_edit_ = new QPlainTextEdit(editor_body_);
  message_edit_->setPlaceholderText(tr("Protobuf JSON message body"));
  message_edit_->setMinimumHeight(160);
  QFont mono = message_edit_->font();
  mono.setFamily(QStringLiteral("Monospace"));
  mono.setPointSizeF(std::max(9.0, mono.pointSizeF() - 1.0));
  message_edit_->setFont(mono);
  form->addRow(tr("Message"), message_edit_);

  scroll->setWidget(editor_body_);
  root->addWidget(scroll, 1);

  publish_button_ = new QPushButton(config_.button_label, this);
  publish_button_->setCursor(Qt::PointingHandCursor);
  publish_button_->setMinimumHeight(32);
  root->addWidget(publish_button_);

  connect(editing_mode_check_, &QCheckBox::toggled, this,
          &PublishEditorWidget::onEditingModeToggled);
  connect(channel_combo_, &QComboBox::currentTextChanged, this,
          &PublishEditorWidget::onChannelChanged);
  connect(message_type_combo_, &QComboBox::currentTextChanged, this,
          &PublishEditorWidget::onMessageTypeChanged);
  connect(reset_template_button_, &QPushButton::clicked, this,
          &PublishEditorWidget::onResetTemplate);
  connect(message_edit_, &QPlainTextEdit::textChanged, this,
          &PublishEditorWidget::onFieldEdited);
  connect(publish_button_, &QPushButton::clicked, this,
          &PublishEditorWidget::onPublishClicked);

  rebuildMessageTypeList();
  refreshChannels();
  setConfig(config_);
}

PublishPanelConfig PublishEditorWidget::config() const {
  PublishPanelConfig out = config_;
  out.channel = channel_combo_->currentText().trimmed();
  const QVariant type_data = message_type_combo_->currentData();
  out.message_type = type_data.isValid() && !type_data.toString().isEmpty()
                         ? type_data.toString()
                         : message_type_combo_->currentText().trimmed();
  out.message_json = message_edit_->toPlainText();
  out.editing_mode = editing_mode_check_->isChecked();
  return out;
}

void PublishEditorWidget::setConfig(const PublishPanelConfig& config) {
  config_ = config;
  suppress_template_update_ = true;

  editing_mode_check_->setChecked(config_.editing_mode);
  publish_button_->setText(config_.button_label.isEmpty() ? tr("Publish")
                                                          : config_.button_label);
  publish_button_->setToolTip(config_.button_tooltip);
  if (config_.button_color.isValid()) {
    publish_button_->setStyleSheet(
        QStringLiteral(
            "QPushButton { background: %1; color: palette(button-text); border: none;"
            " border-radius: 4px; padding: 6px 12px; font-weight: 600; }"
            "QPushButton:hover { background: %2; }"
            "QPushButton:disabled { background: palette(mid); color: palette(midlight); }")
            .arg(config_.button_color.name(), config_.button_color.lighter(110).name()));
  } else {
    publish_button_->setStyleSheet(
        QStringLiteral(
            "QPushButton { background: palette(highlight); color: palette(highlighted-text);"
            " border: none; border-radius: 4px; padding: 6px 12px; font-weight: 600; }"
            "QPushButton:hover { background: palette(highlight); }"
            "QPushButton:disabled { background: palette(mid); color: palette(midlight); }"));
  }

  const int channel_index = channel_combo_->findText(config_.channel);
  if (channel_index >= 0) {
    channel_combo_->setCurrentIndex(channel_index);
  } else {
    channel_combo_->setEditText(config_.channel);
  }

  const int type_index = message_type_combo_->findData(config_.message_type);
  if (type_index >= 0) {
    message_type_combo_->setCurrentIndex(type_index);
  } else {
    const int text_index = message_type_combo_->findText(config_.message_type);
    if (text_index >= 0) {
      message_type_combo_->setCurrentIndex(text_index);
    } else {
      message_type_combo_->setEditText(config_.message_type);
    }
  }

  message_edit_->setPlainText(config_.message_json);
  if (config_.message_json.trimmed().isEmpty() &&
      !config_.message_type.trimmed().isEmpty()) {
    maybeFillTemplateForType(config_.message_type);
  }

  suppress_template_update_ = false;
  applyEditingModeUi();
  updatePublishButtonState();
}

void PublishEditorWidget::refreshChannels() {
  const QString current = channel_combo_->currentText();
  channel_combo_->blockSignals(true);
  channel_combo_->clear();
  channel_combo_->addItems(KnownChannels(manager_));
  const int index = channel_combo_->findText(current);
  if (index >= 0) {
    channel_combo_->setCurrentIndex(index);
  } else if (!current.isEmpty()) {
    channel_combo_->setEditText(current);
  }
  channel_combo_->blockSignals(false);
}

void PublishEditorWidget::rebuildMessageTypeList() {
  message_type_combo_->blockSignals(true);
  const QString current = message_type_combo_->currentText();
  message_type_combo_->clear();
  for (const std::string& type : PublishMessageCodec::instance().listMessageTypes()) {
    message_type_combo_->addItem(ShortTypeLabel(type), QString::fromStdString(type));
  }
  const int index = message_type_combo_->findText(current);
  if (index >= 0) {
    message_type_combo_->setCurrentIndex(index);
  } else if (!current.isEmpty()) {
    message_type_combo_->setEditText(current);
  }
  message_type_combo_->blockSignals(false);
}

void PublishEditorWidget::applyEditingModeUi() {
  const bool editing = editing_mode_check_->isChecked();
  editor_body_->setVisible(editing);
  editing_mode_check_->setVisible(true);
}

void PublishEditorWidget::updatePublishButtonState() {
  const PublishPanelConfig current = config();
  const bool has_node =
      manager_ != nullptr && manager_->autolinkNode() != nullptr;
  const bool has_channel = !current.channel.isEmpty();
  const bool has_type = !current.message_type.isEmpty();
  const bool has_message = !current.message_json.trimmed().isEmpty();
  publish_button_->setEnabled(has_node && has_channel && has_type && has_message);

  if (!has_node) {
    updateStatus(tr("Autolink node unavailable"), true);
  } else if (!has_channel || !has_type) {
    updateStatus(tr("Configure channel and message type"), false);
  } else if (!has_message) {
    updateStatus(tr("Enter message JSON"), false);
  } else {
    updateStatus(QString(), false);
  }
}

void PublishEditorWidget::updateStatus(const QString& text, bool is_error) {
  status_label_->setText(text);
  StylePanelStatusLabel(status_label_, is_error);
}

QString PublishEditorWidget::messageTypeForChannel(const QString& channel) const {
  if (manager_ == nullptr || channel.isEmpty()) {
    return {};
  }
  for (const integration::ChannelInfo& info : manager_->channels()) {
    if (QString::fromStdString(info.channel_name) == channel &&
        !info.message_type.empty()) {
      return QString::fromStdString(info.message_type);
    }
  }
  return {};
}

void PublishEditorWidget::maybeFillTemplateForType(const QString& message_type) {
  if (suppress_template_update_ || message_type.trimmed().isEmpty()) {
    return;
  }
  const auto template_json =
      PublishMessageCodec::instance().defaultJsonTemplate(
          message_type.trimmed().toStdString());
  if (!template_json.has_value()) {
    return;
  }
  message_edit_->setPlainText(*template_json);
  config_.message_json = *template_json;
}

void PublishEditorWidget::emitConfigChanged() { emit configChanged(); }

void PublishEditorWidget::onEditingModeToggled(bool enabled) {
  config_.editing_mode = enabled;
  applyEditingModeUi();
  emitConfigChanged();
}

void PublishEditorWidget::onChannelChanged(const QString& text) {
  config_.channel = text.trimmed();
  const QString inferred = messageTypeForChannel(config_.channel);
  if (!inferred.isEmpty() && message_type_combo_->currentText().trimmed().isEmpty()) {
    suppress_template_update_ = true;
    const int index = message_type_combo_->findData(inferred);
    if (index >= 0) {
      message_type_combo_->setCurrentIndex(index);
    } else {
      message_type_combo_->setEditText(inferred);
    }
    suppress_template_update_ = false;
    maybeFillTemplateForType(inferred);
  }
  updatePublishButtonState();
  emitConfigChanged();
}

void PublishEditorWidget::onMessageTypeChanged(const QString& text) {
  config_.message_type = text.trimmed();
  if (message_edit_->toPlainText().trimmed().isEmpty()) {
    maybeFillTemplateForType(config_.message_type);
  }
  updatePublishButtonState();
  emitConfigChanged();
}

void PublishEditorWidget::onResetTemplate() {
  const QString message_type = message_type_combo_->currentText().trimmed();
  if (message_type.isEmpty()) {
    return;
  }
  maybeFillTemplateForType(message_type);
  updatePublishButtonState();
  emitConfigChanged();
}

void PublishEditorWidget::onFieldEdited() {
  config_.message_json = message_edit_->toPlainText();
  updatePublishButtonState();
  emitConfigChanged();
}

void PublishEditorWidget::onPublishClicked() {
  const PublishPanelConfig current = config();
  const CodecResult encoded = PublishMessageCodec::instance().encodeJson(
      current.message_type.toStdString(), current.message_json);
  if (!encoded.ok) {
    updateStatus(encoded.error, true);
    return;
  }

  if (!integration::ChannelWriterRegistry::instance().publish(
          current.channel.toStdString(), encoded.payload)) {
    updateStatus(tr("Failed to publish to channel"), true);
    return;
  }

  updateStatus(tr("Published to %1").arg(current.channel), false);
  emit publishRequested();
}

}  // namespace publish_panel
}  // namespace autoviz
