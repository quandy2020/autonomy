/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/publish/publish_editor_widget.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QCompleter>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFont>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSplitter>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QFrame>

#include <algorithm>
#include <chrono>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"
#include "autoviz/integration/channel_payload.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/integration/channel_writer_registry.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/ui/plot/plot_path_utils.hpp"
#include "autoviz/ui/publish/publish_field_tree.hpp"
#include "autoviz/ui/publish/publish_message_codec.hpp"

namespace autoviz {
namespace publish_panel {
namespace {

QString ShortTypeLabel(const std::string& type) {
  QString label = QString::fromStdString(type);
  static const QString kPrefix = QStringLiteral("automsgs.msgs.");
  if (label.startsWith(kPrefix)) {
    label = label.mid(kPrefix.size());
  }
  return label;
}

int FindPresetIndex(const QVector<PublishPreset>& presets, const QString& name) {
  for (int i = 0; i < presets.size(); ++i) {
    if (presets.at(i).name.compare(name, Qt::CaseInsensitive) == 0) {
      return i;
    }
  }
  return -1;
}

QString FormatTimestampNow() {
  return QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd hh:mm:ss.zzz"));
}

}  // namespace

PublishEditorWidget::PublishEditorWidget(common::VisualizationManager* manager,
                                         QWidget* parent)
    : manager_(manager), config_(DefaultPublishPanelConfig()), QWidget(parent) {
  ApplyPanelShell(this);
  setFocusPolicy(Qt::StrongFocus);

  latest_fill_timer_ = new QTimer(this);
  latest_fill_timer_->setSingleShot(true);

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin,
                           PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin);
  root->setSpacing(6);

  collection_bar_ = new QWidget(this);
  auto* collection_layout = new QHBoxLayout(collection_bar_);
  collection_layout->setContentsMargins(0, 0, 0, 0);
  collection_layout->setSpacing(6);
  preset_combo_ = new QComboBox(collection_bar_);
  preset_combo_->setMinimumWidth(140);
  save_preset_button_ = new QPushButton(tr("Save"), collection_bar_);
  save_preset_button_->setFlat(true);
  delete_preset_button_ = new QPushButton(tr("Delete"), collection_bar_);
  delete_preset_button_->setFlat(true);
  collection_layout->addWidget(new QLabel(tr("Collection"), collection_bar_));
  collection_layout->addWidget(preset_combo_, 1);
  collection_layout->addWidget(save_preset_button_);
  collection_layout->addWidget(delete_preset_button_);
  collection_layout->addStretch();
  root->addWidget(collection_bar_);

  rqt_top_bar_ = new QWidget(this);
  auto* top_layout = new QHBoxLayout(rqt_top_bar_);
  top_layout->setContentsMargins(0, 0, 0, 0);
  top_layout->setSpacing(6);
  top_layout->addWidget(new QLabel(tr("Channel:"), rqt_top_bar_));
  channel_combo_ = new QComboBox(rqt_top_bar_);
  channel_combo_->setEditable(true);
  channel_combo_->setInsertPolicy(QComboBox::NoInsert);
  channel_combo_->setMinimumWidth(140);
  channel_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  channel_combo_->setCompleter(new QCompleter(channel_combo_));
  channel_combo_->completer()->setCaseSensitivity(Qt::CaseInsensitive);
  channel_combo_->completer()->setFilterMode(Qt::MatchContains);
  top_layout->addWidget(channel_combo_, 2);
  top_layout->addWidget(new QLabel(tr("Type:"), rqt_top_bar_));
  message_type_combo_ = new QComboBox(rqt_top_bar_);
  message_type_combo_->setEditable(true);
  message_type_combo_->setInsertPolicy(QComboBox::NoInsert);
  message_type_combo_->setMinimumWidth(160);
  message_type_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  top_layout->addWidget(message_type_combo_, 2);
  top_layout->addWidget(new QLabel(tr("Freq.:"), rqt_top_bar_));
  publish_rate_spin_ = new QDoubleSpinBox(rqt_top_bar_);
  publish_rate_spin_->setRange(0.1, 100.0);
  publish_rate_spin_->setSingleStep(0.5);
  publish_rate_spin_->setDecimals(1);
  publish_rate_spin_->setValue(config_.publish_rate_hz);
  publish_rate_spin_->setFixedWidth(64);
  top_layout->addWidget(publish_rate_spin_);
  top_layout->addWidget(new QLabel(tr("Hz"), rqt_top_bar_));
  editing_mode_check_ = new QCheckBox(tr("Advanced"), rqt_top_bar_);
  editing_mode_check_->setChecked(config_.editing_mode);
  editing_mode_check_->setToolTip(tr("Show JSON editor and collection"));
  top_layout->addWidget(editing_mode_check_);

  auto style_icon_button = [](QPushButton* button) {
    button->setFlat(true);
    button->setFixedSize(32, 32);
    QFont font = button->font();
    font.setPointSizeF(font.pointSizeF() + 2.0);
    button->setFont(font);
  };

  add_publisher_button_ = new QPushButton(QStringLiteral("+"), rqt_top_bar_);
  add_publisher_button_->setToolTip(tr("Add publisher to list"));
  style_icon_button(add_publisher_button_);
  remove_publisher_button_ = new QPushButton(QStringLiteral("-"), rqt_top_bar_);
  remove_publisher_button_->setToolTip(tr("Remove selected publisher"));
  style_icon_button(remove_publisher_button_);
  refresh_topics_button_ = new QPushButton(QStringLiteral("\u21bb"), rqt_top_bar_);
  refresh_topics_button_->setToolTip(tr("Refresh channel list"));
  style_icon_button(refresh_topics_button_);

  auto* action_separator = new QFrame(rqt_top_bar_);
  action_separator->setFrameShape(QFrame::VLine);
  action_separator->setFrameShadow(QFrame::Sunken);
  action_separator->setFixedWidth(2);

  publish_once_button_ = new QPushButton(tr("Publish"), rqt_top_bar_);
  publish_once_button_->setToolTip(tr("Publish once (Ctrl+Enter)"));
  publish_once_button_->setMinimumSize(88, 32);
  publish_once_button_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
  publish_once_button_->setDefault(true);
  publish_once_button_->setStyleSheet(
      QStringLiteral("QPushButton { font-weight: 600; padding: 4px 16px; }"));

  top_layout->addWidget(add_publisher_button_);
  top_layout->addWidget(remove_publisher_button_);
  top_layout->addWidget(refresh_topics_button_);
  top_layout->addWidget(action_separator);
  top_layout->addWidget(publish_once_button_);
  root->addWidget(rqt_top_bar_);

  publishers_tree_ = new PublishFieldTreeWidget(this);
  publishers_tree_->setDisplayMode(PublishFieldTreeWidget::DisplayMode::kRqtPublishers);
  publishers_tree_->setMinimumHeight(160);
  publishers_tree_->setToolTip(
      tr("rqt-style publisher list.\n"
         "Checkbox toggles loop publish.\n"
         "Edit expression values inline.\n"
         "Right-click arrays for +/- elements."));
  root->addWidget(publishers_tree_, 1);

  editor_body_ = new QWidget(this);
  auto* editor_layout = new QVBoxLayout(editor_body_);
  editor_layout->setContentsMargins(0, 0, 0, 0);
  editor_layout->setSpacing(6);

  expression_toolbar_ = new QWidget(editor_body_);
  auto* expression_toolbar_layout = new QHBoxLayout(expression_toolbar_);
  expression_toolbar_layout->setContentsMargins(0, 0, 0, 0);
  expression_toolbar_layout->setSpacing(6);
  expression_toolbar_layout->addWidget(new QLabel(tr("Expression"), expression_toolbar_));
  reset_template_button_ = new QPushButton(tr("Use template"), expression_toolbar_);
  reset_template_button_->setFlat(true);
  fill_latest_button_ = new QPushButton(tr("From latest"), expression_toolbar_);
  fill_latest_button_->setFlat(true);
  expression_toolbar_layout->addWidget(reset_template_button_);
  expression_toolbar_layout->addWidget(fill_latest_button_);
  expression_toolbar_layout->addStretch();
  editor_layout->addWidget(expression_toolbar_);

  payload_splitter_ = new QSplitter(Qt::Vertical, editor_body_);
  payload_splitter_->setChildrenCollapsible(false);

  request_group_ = new QGroupBox(tr("Message"), editor_body_);
  auto* request_layout = new QVBoxLayout(request_group_);
  request_layout->setContentsMargins(8, 12, 8, 8);
  message_tabs_ = new QTabWidget(request_group_);
  message_edit_ = makeJsonEditor(message_tabs_, tr("Message expression (JSON)"));

  auto* fields_tab = new QWidget(message_tabs_);
  auto* fields_layout = new QVBoxLayout(fields_tab);
  fields_layout->setContentsMargins(0, 0, 0, 0);
  fields_layout->setSpacing(4);
  auto* fields_toolbar = new QHBoxLayout();
  fields_toolbar->setContentsMargins(0, 0, 0, 0);
  auto* expand_all_button = new QPushButton(tr("Expand all"), fields_tab);
  expand_all_button->setFlat(true);
  auto* collapse_all_button = new QPushButton(tr("Collapse all"), fields_tab);
  collapse_all_button->setFlat(true);
  auto* add_array_button = new QPushButton(QStringLiteral("+"), fields_tab);
  add_array_button->setFlat(true);
  auto* remove_array_button = new QPushButton(QStringLiteral("-"), fields_tab);
  remove_array_button->setFlat(true);
  fields_toolbar->addWidget(expand_all_button);
  fields_toolbar->addWidget(collapse_all_button);
  fields_toolbar->addStretch();
  fields_toolbar->addWidget(add_array_button);
  fields_toolbar->addWidget(remove_array_button);
  field_tree_ = new PublishFieldTreeWidget(fields_tab);
  field_tree_->setDisplayMode(PublishFieldTreeWidget::DisplayMode::kFieldsEditor);
  fields_layout->addLayout(fields_toolbar);
  fields_layout->addWidget(field_tree_, 1);

  metadata_edit_ = makeJsonEditor(message_tabs_, tr("Channel metadata"), true);
  message_tabs_->addTab(message_edit_, tr("JSON"));
  message_tabs_->addTab(fields_tab, tr("Fields"));
  message_tabs_->addTab(metadata_edit_, tr("Metadata"));
  request_layout->addWidget(message_tabs_);
  payload_splitter_->addWidget(request_group_);

  result_group_ = new QGroupBox(tr("Status"), editor_body_);
  auto* result_layout = new QVBoxLayout(result_group_);
  result_layout->setContentsMargins(8, 12, 8, 8);
  result_status_label_ = new QLabel(tr("Ready"), result_group_);
  result_edit_ = makeJsonEditor(result_group_, tr("Publish status"), true);
  result_edit_->setMaximumHeight(80);
  result_layout->addWidget(result_status_label_);
  result_layout->addWidget(result_edit_);
  payload_splitter_->addWidget(result_group_);
  payload_splitter_->setStretchFactor(0, 3);
  payload_splitter_->setStretchFactor(1, 1);

  editor_layout->addWidget(payload_splitter_, 1);
  root->addWidget(editor_body_, 2);

  connect(editing_mode_check_, &QCheckBox::toggled, this,
          &PublishEditorWidget::onEditingModeToggled);
  connect(preset_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          &PublishEditorWidget::onPresetSelected);
  connect(save_preset_button_, &QPushButton::clicked, this,
          &PublishEditorWidget::onSavePreset);
  connect(delete_preset_button_, &QPushButton::clicked, this,
          &PublishEditorWidget::onDeletePreset);
  connect(channel_combo_, &QComboBox::currentTextChanged, this,
          &PublishEditorWidget::onChannelChanged);
  connect(message_type_combo_, &QComboBox::currentTextChanged, this,
          &PublishEditorWidget::onMessageTypeChanged);
  connect(reset_template_button_, &QPushButton::clicked, this,
          &PublishEditorWidget::onResetTemplate);
  connect(fill_latest_button_, &QPushButton::clicked, this,
          &PublishEditorWidget::onFillFromLatest);
  connect(publish_rate_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
          this, &PublishEditorWidget::onPublishRateChanged);
  connect(add_publisher_button_, &QPushButton::clicked, this,
          &PublishEditorWidget::onAddPublisher);
  connect(remove_publisher_button_, &QPushButton::clicked, this,
          &PublishEditorWidget::onRemovePublisher);
  connect(refresh_topics_button_, &QPushButton::clicked, this,
          &PublishEditorWidget::onRefreshTopics);
  connect(publish_once_button_, &QPushButton::clicked, this,
          &PublishEditorWidget::onPublishOnceClicked);
  connect(publishers_tree_, &PublishFieldTreeWidget::publisherEdited, this,
          &PublishEditorWidget::onPublisherEdited);
  connect(publishers_tree_, &PublishFieldTreeWidget::publisherPublishingChanged, this,
          &PublishEditorWidget::onPublisherPublishingChanged);
  connect(publishers_tree_, &PublishFieldTreeWidget::publisherRateChanged, this,
          &PublishEditorWidget::onPublisherRateChanged);
  connect(publishers_tree_, &PublishFieldTreeWidget::publisherSelectionChanged, this,
          [this](int index) {
            if (index >= 0 && !suppress_draft_sync_) {
              loadPublisherAt(index);
              emitConfigChanged();
            } else if (!suppress_draft_sync_) {
              updatePublishButtonState();
            }
          });
  connect(message_tabs_, &QTabWidget::currentChanged, this,
          &PublishEditorWidget::onMessageTabChanged);
  connect(field_tree_, &PublishFieldTreeWidget::messageEdited, this,
          &PublishEditorWidget::onFieldsEdited);
  connect(expand_all_button, &QPushButton::clicked, field_tree_,
          &PublishFieldTreeWidget::expandAllFields);
  connect(collapse_all_button, &QPushButton::clicked, field_tree_,
          &PublishFieldTreeWidget::collapseAllFields);
  connect(add_array_button, &QPushButton::clicked, field_tree_,
          &PublishFieldTreeWidget::addArrayElement);
  connect(remove_array_button, &QPushButton::clicked, field_tree_,
          &PublishFieldTreeWidget::removeArrayElement);
  connect(message_edit_, &QPlainTextEdit::textChanged, this,
          &PublishEditorWidget::onFieldEdited);
  connect(latest_fill_timer_, &QTimer::timeout, this, [this]() {
    if (!message_user_edited_ &&
        message_edit_->toPlainText().trimmed().isEmpty()) {
      maybeFillTemplateForType(resolvedMessageType());
    }
    cancelLatestMessageFill();
  });

  rebuildMessageTypeList();
  refreshChannels();
  rebuildPresetCombo();
  setConfig(config_);
}

QPlainTextEdit* PublishEditorWidget::makeJsonEditor(QWidget* parent,
                                                    const QString& placeholder,
                                                    bool read_only) {
  auto* editor = new QPlainTextEdit(parent);
  editor->setPlaceholderText(placeholder);
  editor->setReadOnly(read_only);
  editor->setMinimumHeight(read_only ? 80 : 160);
  QFont mono = editor->font();
  mono.setFamily(QStringLiteral("Monospace"));
  mono.setPointSizeF(std::max(9.0, mono.pointSizeF() - 1.0));
  editor->setFont(mono);
  if (read_only) {
    editor->setStyleSheet(QStringLiteral("QPlainTextEdit { background: palette(alternate-base); }"));
  }
  return editor;
}

void PublishEditorWidget::keyPressEvent(QKeyEvent* event) {
  if (event->matches(QKeySequence::InsertLineSeparator)) {
    onPublishOnceClicked();
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_Space && publishers_tree_->hasFocus()) {
    const int row = selectedPublisherRow();
    if (row >= 0 && row < config_.publishers.size()) {
      PublishEntry& entry = config_.publishers[row];
      entry.publishing = !entry.publishing;
      publishers_tree_->setPublisherPublishing(row, entry.publishing);
      if (entry.publishing) {
        syncPublisherJsonFromTree(row);
        startPublisherTimer(entry);
        publishEntry(row, true);
      } else {
        stopPublisherTimer(entry.id);
      }
      emitConfigChanged();
    }
    event->accept();
    return;
  }
  QWidget::keyPressEvent(event);
}

PublishPanelConfig PublishEditorWidget::config() const {
  PublishPanelConfig out = config_;
  out.channel = channel_combo_->currentText().trimmed();
  out.message_type = resolvedMessageType();
  out.message_json = message_edit_->toPlainText();
  out.publish_rate_hz = publish_rate_spin_->value();
  out.editing_mode = editing_mode_check_->isChecked();
  out.custom_channels = custom_channels_;
  out.publishers = config_.publishers;
  out.selected_publisher_index = selectedPublisherRow();
  if (preset_combo_ != nullptr && preset_combo_->currentIndex() > 0) {
    out.active_preset_name = preset_combo_->currentData().toString();
  } else {
    out.active_preset_name.clear();
  }
  return out;
}

void PublishEditorWidget::setConfig(const PublishPanelConfig& config) {
  stopAllPublisherTimers();
  config_ = config;
  // Persistently saved layouts may still use plain Twist on /cmd_vel; autonomy
  // stack requires TwistStamped (controller / autosim / task).
  if (config_.channel == QStringLiteral("/cmd_vel") &&
      (config_.message_type == QStringLiteral("automsgs.msgs.geometry_msgs.Twist") ||
       config_.message_type == QStringLiteral("geometry_msgs/Twist") ||
       config_.message_type == QStringLiteral("geometry_msgs.Twist"))) {
    config_.message_type =
        QStringLiteral("automsgs.msgs.geometry_msgs.TwistStamped");
  }
  for (auto& entry : config_.publishers) {
    if (entry.channel == QStringLiteral("/cmd_vel") &&
        (entry.message_type ==
             QStringLiteral("automsgs.msgs.geometry_msgs.Twist") ||
         entry.message_type == QStringLiteral("geometry_msgs/Twist") ||
         entry.message_type == QStringLiteral("geometry_msgs.Twist"))) {
      entry.message_type =
          QStringLiteral("automsgs.msgs.geometry_msgs.TwistStamped");
    }
  }
  custom_channels_ = config_.custom_channels;
  suppress_template_update_ = true;
  suppress_draft_sync_ = true;

  editing_mode_check_->setChecked(config_.editing_mode);

  if (config_.publishers.isEmpty() && !config_.channel.isEmpty() &&
      !config_.message_type.isEmpty() &&
      (!config_.message_json.trimmed().isEmpty() || config_.loop_publish)) {
    PublishEntry migrated;
    migrated.id = NewPublishEntryId();
    migrated.channel = config_.channel;
    migrated.message_type = config_.message_type;
    migrated.publish_rate_hz = config_.publish_rate_hz;
    migrated.message_json = config_.message_json;
    migrated.publishing = config_.loop_publish;
    config_.publishers.push_back(migrated);
  }

  if (!config_.channel.isEmpty()) {
    rememberCustomChannel(config_.channel);
  }
  refreshChannels();
  channel_combo_->setCurrentText(config_.channel);
  setMessageTypeField(config_.message_type);
  publish_rate_spin_->setValue(config_.publish_rate_hz);

  message_user_edited_ = !config_.message_json.trimmed().isEmpty();
  message_edit_->setPlainText(config_.message_json);
  fields_dirty_ = true;

  rebuildPresetCombo();
  if (!config_.active_preset_name.isEmpty()) {
    const int preset_index =
        preset_combo_->findData(config_.active_preset_name);
    if (preset_index >= 0) {
      preset_combo_->setCurrentIndex(preset_index);
    }
  }

  rebuildPublishersTree();
  restorePublisherTimers();

  suppress_template_update_ = false;
  suppress_draft_sync_ = false;
  if (message_edit_->toPlainText().trimmed().isEmpty() &&
      !config_.message_type.trimmed().isEmpty()) {
    maybeFillTemplateForType(config_.message_type);
  }
  applyEditingModeUi();
  updatePublishButtonState();
}

void PublishEditorWidget::refreshChannels() {
  const QString current = channel_combo_->currentText().trimmed();
  channel_combo_->blockSignals(true);
  channel_combo_->clear();

  QStringList channels = plot::AllKnownChannels(manager_);
  for (const PublishEntry& entry : config_.publishers) {
    const QString channel = entry.channel.trimmed();
    if (!channel.isEmpty() && !channels.contains(channel, Qt::CaseInsensitive)) {
      channels.push_back(channel);
    }
  }
  for (const QString& custom : custom_channels_) {
    if (!custom.isEmpty() && !channels.contains(custom, Qt::CaseInsensitive)) {
      channels.push_back(custom);
    }
  }
  if (!current.isEmpty() &&
      !channels.contains(current, Qt::CaseInsensitive)) {
    channels.push_back(current);
  }
  channels.sort(Qt::CaseInsensitive);
  channel_combo_->addItems(channels);
  channel_combo_->setCurrentText(current);
  channel_combo_->blockSignals(false);
}

void PublishEditorWidget::rebuildPresetCombo() {
  if (preset_combo_ == nullptr) {
    return;
  }
  const QString current = preset_combo_->currentData().toString();
  preset_combo_->blockSignals(true);
  preset_combo_->clear();
  preset_combo_->addItem(tr("New request"), QString());
  for (const PublishPreset& preset : config_.saved_presets) {
    preset_combo_->addItem(preset.name, preset.name);
  }
  const int restore_index = preset_combo_->findData(current);
  preset_combo_->setCurrentIndex(restore_index >= 0 ? restore_index : 0);
  preset_combo_->blockSignals(false);
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
  collection_bar_->setVisible(editing);
  editor_body_->setVisible(editing);
  if (editing) {
    syncFieldsFromJson();
  }
}

void PublishEditorWidget::ensureDraftExpression() {
  if (message_edit_->toPlainText().trimmed().isEmpty()) {
    const QString message_type = resolvedMessageType();
    if (!message_type.isEmpty()) {
      maybeFillTemplateForType(message_type);
    }
  }
}

bool PublishEditorWidget::prepareDraftPublisher(PublishEntry* entry,
                                                QString* error) {
  if (entry == nullptr) {
    return false;
  }
  enterDraftMode();
  if (message_tabs_->currentIndex() == 1) {
    syncJsonFromFields();
  }
  ensureDraftExpression();
  captureDraftFromUi();
  if (config_.message_json.trimmed().isEmpty() && !config_.message_type.isEmpty()) {
    maybeFillTemplateForType(config_.message_type);
    captureDraftFromUi();
  }

  if (config_.channel.isEmpty()) {
    if (error != nullptr) {
      *error = tr("Set a channel first.");
    }
    return false;
  }
  if (config_.message_type.isEmpty()) {
    if (error != nullptr) {
      *error = tr("Set a message type first.");
    }
    return false;
  }
  if (config_.message_json.trimmed().isEmpty()) {
    const auto template_json =
        PublishMessageCodec::instance().defaultJsonTemplate(
            config_.message_type.toStdString());
    if (template_json.has_value()) {
      config_.message_json = *template_json;
      fillMessageJson(*template_json, false);
    }
  }
  if (config_.message_json.trimmed().isEmpty()) {
    if (error != nullptr) {
      *error = tr("Could not build an expression for %1.\n"
                  "Check that the type is registered (e.g. "
                  "automsgs.msgs.geometry_msgs.TwistStamped).")
                   .arg(ShortTypeLabel(config_.message_type.toStdString()));
    }
    return false;
  }

  entry->id = NewPublishEntryId();
  entry->channel = config_.channel;
  entry->message_type = config_.message_type;
  entry->publish_rate_hz = config_.publish_rate_hz;
  entry->message_json = config_.message_json;
  entry->publishing = true;
  return true;
}

void PublishEditorWidget::enterDraftMode() {
  if (publishers_tree_ != nullptr) {
    publishers_tree_->setCurrentItem(nullptr);
  }
}

void PublishEditorWidget::loadPublisherAt(int index) {
  if (index < 0 || index >= config_.publishers.size()) {
    return;
  }
  if (selectedPublisherRow() == index &&
      publishers_tree_ != nullptr &&
      publishers_tree_->isPublisherRootSelected(index) &&
      channel_combo_->currentText().trimmed().compare(
          config_.publishers.at(index).channel, Qt::CaseInsensitive) == 0) {
    return;
  }

  cancelLatestMessageFill();
  suppress_draft_sync_ = true;
  const PublishEntry& entry = config_.publishers.at(index);

  channel_combo_->blockSignals(true);
  message_type_combo_->blockSignals(true);
  publish_rate_spin_->blockSignals(true);
  channel_combo_->setCurrentText(entry.channel);
  setMessageTypeField(entry.message_type);
  publish_rate_spin_->setValue(entry.publish_rate_hz);
  channel_combo_->blockSignals(false);
  message_type_combo_->blockSignals(false);
  publish_rate_spin_->blockSignals(false);

  message_user_edited_ = !entry.message_json.trimmed().isEmpty();
  fillMessageJson(entry.message_json, message_user_edited_);
  fields_dirty_ = true;

  if (publishers_tree_ != nullptr) {
    publishers_tree_->blockSignals(true);
    publishers_tree_->selectPublisher(index);
    publishers_tree_->blockSignals(false);
  }

  suppress_draft_sync_ = false;
  updateMetadataPanel();
  updatePublishButtonState();
}

void PublishEditorWidget::updatePublishButtonState() {
  if (!suppress_draft_sync_) {
    ensureDraftExpression();
  }
  const bool has_node =
      manager_ != nullptr && manager_->autolinkNode() != nullptr;
  const QString draft_channel = channel_combo_->currentText().trimmed();
  const bool has_channel = !draft_channel.isEmpty();
  const bool has_type = !resolvedMessageType().isEmpty();
  const bool has_message = !message_edit_->toPlainText().trimmed().isEmpty();
  const bool channel_exists = findPublisherIndexByChannel(draft_channel) >= 0;
  const bool can_add = has_channel && has_type && !channel_exists;
  add_publisher_button_->setEnabled(can_add);
  if (channel_exists) {
    add_publisher_button_->setToolTip(
        tr("Publisher for this channel is already in the list"));
  } else {
    add_publisher_button_->setToolTip(
        can_add ? tr("Add publisher to list (rqt style)")
                : tr("Set channel and type first"));
  }
  const bool can_publish_once =
      has_node &&
      (selectedPublisherCanPublish() || (has_channel && has_type && has_message));
  publish_once_button_->setEnabled(can_publish_once);
  remove_publisher_button_->setEnabled(resolveActivePublisherRow() >= 0);
}

int PublishEditorWidget::resolveActivePublisherRow() const {
  const int combo_row =
      findPublisherIndexByChannel(channel_combo_->currentText().trimmed());
  const int tree_row = selectedPublisherRow();

  if (tree_row >= 0 && tree_row < config_.publishers.size()) {
    if (combo_row < 0 || tree_row == combo_row) {
      return tree_row;
    }
    if (publishers_tree_ != nullptr &&
        publishers_tree_->isPublisherRootSelected(tree_row)) {
      return tree_row;
    }
  }
  return combo_row;
}

void PublishEditorWidget::showResult(bool success, const QString& summary,
                                     const QString& details) {
  result_status_label_->setText(summary);
  result_status_label_->setStyleSheet(
      success ? QStringLiteral("color: #49cc90; font-weight: 600;")
              : QStringLiteral("color: #f93e3e; font-weight: 600;"));
  if (!details.isEmpty()) {
    result_edit_->setPlainText(details);
  }
}

QString PublishEditorWidget::resolvedMessageType() const {
  const QVariant type_data = message_type_combo_->currentData();
  const QString raw = type_data.isValid() && !type_data.toString().isEmpty()
                          ? type_data.toString()
                          : message_type_combo_->currentText().trimmed();
  if (raw.isEmpty()) {
    return {};
  }
  return QString::fromStdString(
      commsgs::NormalizeMessageType(raw.toStdString()));
}

void PublishEditorWidget::rememberCustomChannel(const QString& channel) {
  const QString trimmed = channel.trimmed();
  if (trimmed.isEmpty()) {
    return;
  }
  for (const QString& existing : custom_channels_) {
    if (existing.compare(trimmed, Qt::CaseInsensitive) == 0) {
      return;
    }
  }
  custom_channels_.push_back(trimmed);
}

QString PublishEditorWidget::messageTypeForChannel(const QString& channel) const {
  if (channel.isEmpty()) {
    return {};
  }
  const std::string type = plot::MessageTypeForChannel(manager_, channel);
  return type.empty() ? QString() : QString::fromStdString(type);
}

void PublishEditorWidget::setMessageTypeField(const QString& message_type) {
  const int type_index = message_type_combo_->findData(message_type);
  if (type_index >= 0) {
    message_type_combo_->setCurrentIndex(type_index);
    return;
  }
  const int text_index = message_type_combo_->findText(message_type);
  if (text_index >= 0) {
    message_type_combo_->setCurrentIndex(text_index);
    return;
  }
  message_type_combo_->setEditText(message_type);
}

void PublishEditorWidget::fillMessageJson(const QString& json, bool user_edited) {
  suppress_template_update_ = true;
  message_edit_->setPlainText(json);
  config_.message_json = json;
  message_user_edited_ = user_edited;
  fields_dirty_ = true;
  suppress_template_update_ = false;
}

void PublishEditorWidget::syncFieldsFromJson() {
  const QString message_type = resolvedMessageType();
  if (message_type.isEmpty()) {
    field_tree_->clearMessage();
    fields_dirty_ = false;
    return;
  }
  if (message_tabs_->currentIndex() == 1 || fields_dirty_) {
    field_tree_->loadFromJson(message_type.toStdString(),
                              message_edit_->toPlainText());
    fields_dirty_ = false;
  }
}

void PublishEditorWidget::syncJsonFromFields() {
  if (!field_tree_->hasMessage()) {
    return;
  }
  const QString json = field_tree_->toJson();
  if (json.isEmpty()) {
    return;
  }
  suppress_template_update_ = true;
  message_edit_->setPlainText(json);
  config_.message_json = json;
  suppress_template_update_ = false;
}

void PublishEditorWidget::onRefreshTopics() {
  refreshChannels();
  rebuildMessageTypeList();
  updateMetadataPanel();
  emitConfigChanged();
}

void PublishEditorWidget::onPublishRateChanged(double rate) {
  config_.publish_rate_hz = rate;
  const int row = selectedPublisherRow();
  if (row >= 0 && row < config_.publishers.size()) {
    config_.publishers[row].publish_rate_hz = rate;
    rebuildPublishersTree();
    if (config_.publishers.at(row).publishing) {
      startPublisherTimer(config_.publishers.at(row));
    }
  }
  emitConfigChanged();
}

void PublishEditorWidget::onMessageTabChanged(int index) {
  if (index == 1) {
    syncFieldsFromJson();
    return;
  }
  if (field_tree_->hasMessage()) {
    syncJsonFromFields();
    updatePublishButtonState();
    emitConfigChanged();
  }
  if (index == 2) {
    updateMetadataPanel();
  }
}

void PublishEditorWidget::updateMetadataPanel() {
  if (metadata_edit_ == nullptr) {
    return;
  }
  const QString channel = channel_combo_->currentText().trimmed();
  const QString schema = resolvedMessageType();
  const QString inferred = messageTypeForChannel(channel);

  QStringList lines;
  lines << tr("Endpoint: %1").arg(channel.isEmpty() ? tr("(not set)") : channel);
  lines << tr("Schema (selected): %1")
               .arg(schema.isEmpty() ? tr("(not set)") : ShortTypeLabel(schema.toStdString()));
  if (!inferred.isEmpty() && inferred.compare(schema, Qt::CaseInsensitive) != 0) {
    lines << tr("Schema (discovered): %1").arg(ShortTypeLabel(inferred.toStdString()));
  }

  if (manager_ != nullptr && !channel.isEmpty()) {
    bool found = false;
    for (const integration::ChannelInfo& info : manager_->channels()) {
      if (QString::fromStdString(info.channel_name).compare(channel, Qt::CaseInsensitive) ==
          0) {
        found = true;
        lines << tr("Discovered type: %1")
                     .arg(info.message_type.empty()
                              ? tr("(unknown)")
                              : ShortTypeLabel(info.message_type));
        lines << tr("Has writer: %1").arg(info.has_writer ? tr("yes") : tr("no"));
        break;
      }
    }
    if (!found) {
      lines << tr("Discovery: custom / not listed");
    }
  }

  const QString json = message_edit_->toPlainText().trimmed();
  lines << tr("Payload size: %1 chars").arg(json.size());
  lines << tr("Active publishers: %1").arg(config_.publishers.size());
  if (!config_.active_preset_name.isEmpty()) {
    lines << tr("Active preset: %1").arg(config_.active_preset_name);
  }

  metadata_edit_->setPlainText(lines.join(QLatin1Char('\n')));
}

void PublishEditorWidget::onFieldsEdited() {
  syncJsonFromFields();
  message_user_edited_ = true;
  fields_dirty_ = false;
  updatePublishButtonState();
  emitConfigChanged();
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
  fillMessageJson(*template_json, false);
  updatePublishButtonState();
}

void PublishEditorWidget::cancelLatestMessageFill() {
  if (latest_fill_subscription_ != 0) {
    integration::ChannelReaderRegistry::instance().unsubscribe(
        latest_fill_subscription_);
    latest_fill_subscription_ = 0;
  }
  if (latest_fill_timer_ != nullptr) {
    latest_fill_timer_->stop();
  }
  latest_fill_channel_.clear();
}

void PublishEditorWidget::requestAutoFillMessage(const QString& channel,
                                                 const QString& message_type) {
  cancelLatestMessageFill();
  if (channel.isEmpty() || message_type.isEmpty()) {
    maybeFillTemplateForType(message_type);
    return;
  }

  latest_fill_channel_ = channel;
  latest_fill_timer_->start(250);

  const QString captured_channel = channel;
  const QString captured_type = message_type;
  QTimer::singleShot(0, this, [this, captured_channel, captured_type]() {
    if (latest_fill_channel_.compare(captured_channel, Qt::CaseInsensitive) != 0) {
      return;
    }
    latest_fill_subscription_ =
        integration::ChannelReaderRegistry::instance().subscribe(
            captured_channel.toStdString(),
            [this, captured_channel, captured_type](const std::string& payload) {
              QTimer::singleShot(0, this, [this, captured_channel, captured_type,
                                             payload]() {
                if (latest_fill_channel_.compare(captured_channel,
                                                 Qt::CaseInsensitive) != 0) {
                  return;
                }
                const std::string decoded =
                    integration::DecodeChannelPayload(payload);
                const CodecResult decoded_json =
                    PublishMessageCodec::instance().decodePayloadToJson(
                        captured_type.toStdString(), decoded);
                if (decoded_json.ok) {
                  fillMessageJson(decoded_json.text, false);
                } else if (message_edit_->toPlainText().trimmed().isEmpty()) {
                  maybeFillTemplateForType(captured_type);
                }
                cancelLatestMessageFill();
                updatePublishButtonState();
                emitConfigChanged();
              });
            });
  });
}

void PublishEditorWidget::applyPreset(const PublishPreset& preset) {
  suppress_template_update_ = true;
  config_.active_preset_name = preset.name;
  config_.button_label = preset.button_label.isEmpty() ? config_.button_label
                                                       : preset.button_label;
  config_.button_tooltip = preset.button_tooltip;
  if (preset.button_color.isValid()) {
    config_.button_color = preset.button_color;
  }

  rememberCustomChannel(preset.channel);
  refreshChannels();
  channel_combo_->setCurrentText(preset.channel);
  setMessageTypeField(preset.message_type);
  fillMessageJson(preset.message_json, !preset.message_json.trimmed().isEmpty());

  publish_rate_spin_->setValue(preset.publish_rate_hz);
  config_.publish_rate_hz = preset.publish_rate_hz;

  suppress_template_update_ = false;
  updatePublishButtonState();
}

void PublishEditorWidget::emitConfigChanged() { emit configChanged(); }

void PublishEditorWidget::onEditingModeToggled(bool enabled) {
  config_.editing_mode = enabled;
  applyEditingModeUi();
  emitConfigChanged();
}

void PublishEditorWidget::onChannelChanged(const QString& text) {
  if (suppress_draft_sync_) {
    return;
  }
  const QString channel = text.trimmed();
  config_.channel = channel;

  const int existing_index = findPublisherIndexByChannel(channel);
  if (existing_index >= 0) {
    QTimer::singleShot(0, this, [this, existing_index]() {
      if (suppress_draft_sync_) {
        return;
      }
      loadPublisherAt(existing_index);
      emitConfigChanged();
    });
    return;
  }

  enterDraftMode();
  message_type_combo_->blockSignals(true);
  const QString inferred = messageTypeForChannel(channel);
  if (!inferred.isEmpty()) {
    setMessageTypeField(inferred);
  }
  message_type_combo_->blockSignals(false);
  const QString message_type = resolvedMessageType();

  if (preset_combo_ != nullptr && preset_combo_->currentIndex() > 0) {
    preset_combo_->blockSignals(true);
    preset_combo_->setCurrentIndex(0);
    preset_combo_->blockSignals(false);
    config_.active_preset_name.clear();
  }

  message_user_edited_ = false;
  fillMessageJson(QString(), false);
  fields_dirty_ = true;

  const QString captured_channel = channel;
  const QString captured_type = message_type;
  QTimer::singleShot(0, this, [this, captured_channel, captured_type]() {
    if (config_.channel.compare(captured_channel, Qt::CaseInsensitive) != 0) {
      return;
    }
    maybeFillTemplateForType(captured_type);
    requestAutoFillMessage(captured_channel, captured_type);
    updateMetadataPanel();
    updatePublishButtonState();
  });
  emitConfigChanged();
}

void PublishEditorWidget::onMessageTypeChanged(const QString& /*text*/) {
  config_.message_type = resolvedMessageType();
  if (suppress_draft_sync_) {
    return;
  }
  if (message_edit_->toPlainText().trimmed().isEmpty()) {
    message_user_edited_ = false;
    requestAutoFillMessage(config_.channel, config_.message_type);
  }
  updatePublishButtonState();
  emitConfigChanged();
}

void PublishEditorWidget::onResetTemplate() {
  const QString message_type = resolvedMessageType();
  if (message_type.isEmpty()) {
    return;
  }
  message_user_edited_ = false;
  maybeFillTemplateForType(message_type);
  updatePublishButtonState();
  emitConfigChanged();
}

void PublishEditorWidget::onFillFromLatest() {
  const QString channel = channel_combo_->currentText().trimmed();
  const QString message_type = resolvedMessageType();
  if (channel.isEmpty() || message_type.isEmpty()) {
    showResult(false, tr("Missing input"),
               tr("Configure channel endpoint and schema first."));
    return;
  }
  message_user_edited_ = false;
  requestAutoFillMessage(channel, message_type);
}

void PublishEditorWidget::onFieldEdited() {
  if (!suppress_template_update_) {
    message_user_edited_ = true;
    if (preset_combo_ != nullptr && preset_combo_->currentIndex() > 0) {
      preset_combo_->blockSignals(true);
      preset_combo_->setCurrentIndex(0);
      preset_combo_->blockSignals(false);
      config_.active_preset_name.clear();
    }
  }
  config_.message_json = message_edit_->toPlainText();
  fields_dirty_ = true;
  updatePublishButtonState();
  emitConfigChanged();
}

void PublishEditorWidget::onPresetSelected(int index) {
  if (index <= 0) {
    config_.active_preset_name.clear();
    emitConfigChanged();
    return;
  }
  const QString preset_name = preset_combo_->currentData().toString();
  const int preset_index = FindPresetIndex(config_.saved_presets, preset_name);
  if (preset_index < 0) {
    return;
  }
  applyPreset(config_.saved_presets.at(preset_index));
  emitConfigChanged();
}

void PublishEditorWidget::onSavePreset() {
  const PublishPanelConfig current = config();
  if (current.channel.isEmpty() || current.message_type.isEmpty() ||
      current.message_json.trimmed().isEmpty()) {
    showResult(false, tr("Cannot save"),
               tr("Configure channel, schema, and message before saving."));
    return;
  }

  bool ok = false;
  const QString default_name = current.active_preset_name.isEmpty()
                                   ? current.channel
                                   : current.active_preset_name;
  const QString name = QInputDialog::getText(
      this, tr("Save to collection"), tr("Request name:"), QLineEdit::Normal,
      default_name, &ok);
  if (!ok) {
    return;
  }
  const QString trimmed_name = name.trimmed();
  if (trimmed_name.isEmpty()) {
    return;
  }

  PublishPreset preset;
  preset.name = trimmed_name;
  preset.channel = current.channel;
  preset.message_type = current.message_type;
  preset.message_json = current.message_json;
  preset.publish_rate_hz = current.publish_rate_hz;
  preset.button_label = current.button_label;
  preset.button_tooltip = current.button_tooltip;
  preset.button_color = current.button_color;

  const int existing_index = FindPresetIndex(config_.saved_presets, trimmed_name);
  if (existing_index >= 0) {
    config_.saved_presets[existing_index] = preset;
  } else {
    config_.saved_presets.push_back(preset);
  }
  config_.active_preset_name = trimmed_name;
  rebuildPresetCombo();
  const int preset_index = preset_combo_->findData(trimmed_name);
  if (preset_index >= 0) {
    preset_combo_->setCurrentIndex(preset_index);
  }
  showResult(true, tr("Saved to collection"),
             tr("Request \"%1\" saved.").arg(trimmed_name));
  emitConfigChanged();
}

void PublishEditorWidget::onDeletePreset() {
  if (preset_combo_ == nullptr || preset_combo_->currentIndex() <= 0) {
    return;
  }
  const QString preset_name = preset_combo_->currentData().toString();
  const int preset_index = FindPresetIndex(config_.saved_presets, preset_name);
  if (preset_index < 0) {
    return;
  }
  const auto answer = QMessageBox::question(
      this, tr("Delete request"),
      tr("Delete saved request \"%1\"?").arg(preset_name));
  if (answer != QMessageBox::Yes) {
    return;
  }
  config_.saved_presets.removeAt(preset_index);
  if (config_.active_preset_name == preset_name) {
    config_.active_preset_name.clear();
  }
  rebuildPresetCombo();
  showResult(true, tr("Deleted"), tr("Removed request \"%1\".").arg(preset_name));
  emitConfigChanged();
}

void PublishEditorWidget::onPublishOnceClicked() { publishDraft(false); }

void PublishEditorWidget::onAddPublisher() {
  captureDraftFromUi();
  const int existing_index = findPublisherIndexByChannel(config_.channel);
  if (existing_index >= 0) {
    loadPublisherAt(existing_index);
    showResult(false, tr("Already in list"),
               tr("Publisher for %1 is already in the list.")
                   .arg(config_.channel));
    updatePublishButtonState();
    return;
  }

  PublishEntry entry;
  QString error;
  if (!prepareDraftPublisher(&entry, &error)) {
    QMessageBox::warning(this, tr("Cannot add publisher"), error);
    return;
  }

  config_.publishers.push_back(entry);

  rebuildPublishersTree();
  if (publishers_tree_ != nullptr) {
    publishers_tree_->selectPublisher(config_.publishers.size() - 1);
  }
  enterDraftMode();
  startPublisherTimer(entry);
  showResult(true, tr("Publisher added"),
             tr("Added %1 at %2 Hz.")
                 .arg(entry.channel)
                 .arg(entry.publish_rate_hz));
  updatePublishButtonState();
  emitConfigChanged();
}

void PublishEditorWidget::onRemovePublisher() {
  syncConfigFromPublishersTree();
  int row = resolveActivePublisherRow();
  if (row < 0 || row >= config_.publishers.size()) {
    return;
  }
  const QString removed_channel = config_.publishers.at(row).channel;
  stopPublisherTimer(config_.publishers.at(row).id);
  config_.publishers.removeAt(row);
  rebuildPublishersTree();

  if (config_.publishers.isEmpty()) {
    enterDraftMode();
    channel_combo_->setCurrentText(QString());
    fillMessageJson(QString(), false);
  } else {
    const int next_row =
        std::min(row, static_cast<int>(config_.publishers.size()) - 1);
    loadPublisherAt(next_row);
  }

  showResult(true, tr("Publisher removed"),
             tr("Removed %1 from the list.").arg(removed_channel));
  updatePublishButtonState();
  emitConfigChanged();
}

void PublishEditorWidget::syncPublisherJsonFromTree(int index) {
  if (publishers_tree_ == nullptr || index < 0 ||
      index >= config_.publishers.size()) {
    return;
  }
  const QString json = publishers_tree_->publisherJsonAt(index);
  if (!json.isEmpty()) {
    config_.publishers[index].message_json = json;
  }
}

void PublishEditorWidget::syncConfigFromPublishersTree() {
  if (suppress_tree_update_ || publishers_tree_ == nullptr) {
    return;
  }
  config_.publishers = publishers_tree_->publishers();
}

void PublishEditorWidget::onPublisherEdited(int index) {
  if (suppress_tree_update_ || index < 0 || index >= config_.publishers.size()) {
    return;
  }
  syncPublisherJsonFromTree(index);
  emitConfigChanged();
}

void PublishEditorWidget::onPublisherPublishingChanged(int index, bool publishing) {
  if (index < 0 || index >= config_.publishers.size()) {
    return;
  }
  config_.publishers[index].publishing = publishing;
  PublishEntry& entry = config_.publishers[index];
  if (publishing) {
    syncPublisherJsonFromTree(index);
    startPublisherTimer(entry);
    publishEntry(index, true);
  } else {
    stopPublisherTimer(entry.id);
  }
  emitConfigChanged();
}

void PublishEditorWidget::onPublisherRateChanged(int index, double rate_hz) {
  if (index < 0 || index >= config_.publishers.size()) {
    return;
  }
  config_.publishers[index].publish_rate_hz = rate_hz;
  startPublisherTimer(config_.publishers[index]);
  emitConfigChanged();
}

void PublishEditorWidget::rebuildPublishersTree() {
  if (publishers_tree_ == nullptr) {
    return;
  }
  suppress_tree_update_ = true;
  publishers_tree_->setPublishers(config_.publishers);
  suppress_tree_update_ = false;
}

int PublishEditorWidget::selectedPublisherRow() const {
  return publishers_tree_ == nullptr ? -1 : publishers_tree_->selectedPublisherIndex();
}

bool PublishEditorWidget::selectedPublisherCanPublish() const {
  const int row = selectedPublisherRow();
  if (row < 0 || row >= config_.publishers.size()) {
    return false;
  }
  const PublishEntry& entry = config_.publishers.at(row);
  return !entry.channel.trimmed().isEmpty() &&
         !entry.message_type.trimmed().isEmpty() &&
         !entry.message_json.trimmed().isEmpty();
}

int PublishEditorWidget::findPublisherIndexById(const QString& id) const {
  for (int i = 0; i < config_.publishers.size(); ++i) {
    if (config_.publishers.at(i).id == id) {
      return i;
    }
  }
  return -1;
}

int PublishEditorWidget::findPublisherIndexByChannel(const QString& channel) const {
  const QString trimmed = channel.trimmed();
  if (trimmed.isEmpty()) {
    return -1;
  }
  for (int i = 0; i < config_.publishers.size(); ++i) {
    if (config_.publishers.at(i).channel.compare(trimmed, Qt::CaseInsensitive) == 0) {
      return i;
    }
  }
  return -1;
}

void PublishEditorWidget::captureDraftFromUi() {
  config_.channel = channel_combo_->currentText().trimmed();
  config_.message_type = resolvedMessageType();
  if (message_tabs_->currentIndex() == 1) {
    syncJsonFromFields();
  }
  config_.message_json = message_edit_->toPlainText();
  config_.publish_rate_hz = publish_rate_spin_->value();
}

void PublishEditorWidget::startPublisherTimer(const PublishEntry& entry) {
  const int index = findPublisherIndexById(entry.id);
  if (index < 0 || index >= config_.publishers.size()) {
    stopPublisherTimer(entry.id);
    return;
  }
  const PublishEntry& latest = config_.publishers.at(index);
  if (!latest.publishing || latest.channel.isEmpty() ||
      latest.message_type.isEmpty()) {
    stopPublisherTimer(latest.id);
    return;
  }
  QTimer* timer = publisher_timers_.value(latest.id, nullptr);
  if (timer == nullptr) {
    timer = new QTimer(this);
    timer->setTimerType(Qt::CoarseTimer);
    connect(timer, &QTimer::timeout, this, [this, id = latest.id]() {
      const int row = findPublisherIndexById(id);
      if (row >= 0) {
        publishEntry(row, true);
      }
    });
    publisher_timers_.insert(latest.id, timer);
  }
  const int interval_ms = std::max(
      10, static_cast<int>(1000.0 / std::max(latest.publish_rate_hz, 0.1)));
  timer->setInterval(interval_ms);
  if (!timer->isActive()) {
    timer->start();
  }
}

void PublishEditorWidget::stopPublisherTimer(const QString& entry_id) {
  QTimer* timer = publisher_timers_.take(entry_id);
  if (timer != nullptr) {
    timer->stop();
    timer->deleteLater();
  }
}

void PublishEditorWidget::stopAllPublisherTimers() {
  for (QTimer* timer : publisher_timers_) {
    if (timer != nullptr) {
      timer->stop();
      timer->deleteLater();
    }
  }
  publisher_timers_.clear();
}

void PublishEditorWidget::restorePublisherTimers() {
  stopAllPublisherTimers();
  for (const PublishEntry& entry : config_.publishers) {
    if (entry.publishing) {
      startPublisherTimer(entry);
    }
  }
}

bool PublishEditorWidget::publishDraft(bool from_loop) {
  syncConfigFromPublishersTree();
  const int row = selectedPublisherRow();
  if (row >= 0 && row < config_.publishers.size()) {
    return publishEntry(row, from_loop);
  }

  if (message_tabs_ != nullptr && message_tabs_->currentIndex() == 1) {
    syncJsonFromFields();
  }
  captureDraftFromUi();
  if (config_.message_json.trimmed().isEmpty() && !config_.message_type.isEmpty()) {
    ensureDraftExpression();
    captureDraftFromUi();
  }

  PublishEntry draft;
  draft.channel = config_.channel;
  draft.message_type = config_.message_type;
  draft.message_json = config_.message_json;
  draft.publish_rate_hz = config_.publish_rate_hz;

  const CodecResult encoded = PublishMessageCodec::instance().encodeJson(
      draft.message_type.toStdString(), draft.message_json);
  if (!encoded.ok) {
    if (!from_loop) {
      showResult(false, tr("Validation failed"), encoded.error);
    }
    return false;
  }
  rememberCustomChannel(draft.channel);
  if (!integration::ChannelWriterRegistry::instance().publish(
          draft.channel.toStdString(), encoded.payload,
          draft.message_type.toStdString())) {
    if (!from_loop) {
      showResult(false, tr("Publish failed"),
                 tr("Could not publish to %1.").arg(draft.channel));
    }
    return false;
  }
  if (!from_loop) {
    showResult(true, tr("Published once"),
               tr("Channel: %1\nPayload: %2 bytes")
                   .arg(draft.channel)
                   .arg(static_cast<qulonglong>(encoded.payload.size())));
  }
  emit publishRequested();
  return true;
}

bool PublishEditorWidget::publishEntry(int index, bool from_loop) {
  if (!from_loop) {
    syncConfigFromPublishersTree();
  }
  if (index < 0 || index >= config_.publishers.size()) {
    return false;
  }
  const PublishEntry& entry = config_.publishers.at(index);
  const CodecResult encoded = PublishMessageCodec::instance().encodeJson(
      entry.message_type.toStdString(), entry.message_json);
  if (!encoded.ok) {
    if (!from_loop) {
      showResult(false, tr("Validation failed"), encoded.error);
    } else {
      showResult(false, tr("Loop publish stopped"),
                 tr("Validation failed for %1: %2")
                     .arg(entry.channel, encoded.error));
    }
    stopPublisherTimer(entry.id);
    return false;
  }
  rememberCustomChannel(entry.channel);
  if (from_loop) {
    integration::ChannelWriterRegistry::instance().publishLoop(
        entry.channel.toStdString(), encoded.payload,
        entry.message_type.toStdString());
    return true;
  }
  if (!integration::ChannelWriterRegistry::instance().publish(
          entry.channel.toStdString(), encoded.payload,
          entry.message_type.toStdString())) {
    const integration::PublishDiagnostics diag =
        integration::ChannelWriterRegistry::instance().lastDiagnostics();
    QString details =
        tr("Could not publish to %1.\n"
           "Run autoviz and `autolink channel echo %1` in the same "
           "environment (e.g. same Docker container), then retry.\n"
           "Message type: %2")
            .arg(entry.channel, entry.message_type);
    if (!diag.detail.empty()) {
      details += tr("\nDetail: %1")
                     .arg(QString::fromStdString(diag.detail));
    }
    details += tr("\nTopology reader: %1  Writer peer: %2")
                   .arg(diag.topology_has_reader ? tr("yes") : tr("no"))
                   .arg(diag.writer_has_reader ? tr("yes") : tr("no"));
    if (!from_loop) {
      showResult(false, tr("Publish failed"), details);
    } else {
      showResult(false, tr("Publish failed (will retry)"), details);
    }
    return false;
  }
  if (!from_loop) {
    showResult(true, tr("Published"),
               tr("Channel: %1\nPayload: %2 bytes")
                   .arg(entry.channel)
                   .arg(static_cast<qulonglong>(encoded.payload.size())));
  }
  emit publishRequested();
  return true;
}

}  // namespace publish_panel
}  // namespace autoviz
