/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/service/service_editor_widget.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QCompleter>
#include <QFont>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QTreeWidgetItem>
#include <QLabel>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSplitter>
#include <QTimer>
#include <QVBoxLayout>

#include <chrono>
#include <thread>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/service_client_registry.hpp"
#include "autoviz/integration/service_discovery.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/ui/publish/publish_field_tree.hpp"
#include "autoviz/ui/service/service_message_codec.hpp"

namespace autoviz {
namespace service_panel {
namespace {

QString ShortTypeLabel(const std::string& type) {
  QString label = QString::fromStdString(type);
  static const QString kPrefix = QStringLiteral("automsgs.msgs.");
  if (label.startsWith(kPrefix)) {
    label = label.mid(kPrefix.size());
  }
  return label;
}

}  // namespace

ServiceEditorWidget::ServiceEditorWidget(common::VisualizationManager* manager,
                                         QWidget* parent)
    : manager_(manager), config_(DefaultServiceCallPanelConfig()), QWidget(parent) {
  ApplyPanelShell(this);
  setFocusPolicy(Qt::StrongFocus);

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin,
                           PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin);
  root->setSpacing(6);

  rqt_top_bar_ = new QWidget(this);
  auto* top_layout = new QHBoxLayout(rqt_top_bar_);
  top_layout->setContentsMargins(0, 0, 0, 0);
  top_layout->setSpacing(6);
  top_layout->addWidget(new QLabel(tr("Service:"), rqt_top_bar_));
  service_combo_ = new QComboBox(rqt_top_bar_);
  service_combo_->setEditable(true);
  service_combo_->setInsertPolicy(QComboBox::NoInsert);
  service_combo_->setMinimumWidth(180);
  service_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  service_combo_->setCompleter(new QCompleter(service_combo_));
  service_combo_->completer()->setCaseSensitivity(Qt::CaseInsensitive);
  service_combo_->completer()->setFilterMode(Qt::MatchContains);
  top_layout->addWidget(service_combo_, 2);

  refresh_services_button_ = new QPushButton(QStringLiteral("\u21bb"), rqt_top_bar_);
  refresh_services_button_->setFlat(true);
  refresh_services_button_->setFixedSize(32, 32);
  refresh_services_button_->setToolTip(tr("Refresh service list"));
  top_layout->addWidget(refresh_services_button_);

  editing_mode_check_ = new QCheckBox(tr("Advanced"), rqt_top_bar_);
  editing_mode_check_->setChecked(config_.editing_mode);
  editing_mode_check_->setToolTip(tr("Show JSON editor and request/response types"));
  top_layout->addWidget(editing_mode_check_);

  auto* action_separator = new QFrame(rqt_top_bar_);
  action_separator->setFrameShape(QFrame::VLine);
  action_separator->setFrameShadow(QFrame::Sunken);
  action_separator->setFixedWidth(2);
  top_layout->addWidget(action_separator);

  call_button_ = new QPushButton(tr("Call"), rqt_top_bar_);
  call_button_->setMinimumSize(88, 32);
  call_button_->setDefault(true);
  call_button_->setStyleSheet(
      QStringLiteral("QPushButton { font-weight: 600; padding: 4px 16px; }"));
  top_layout->addWidget(call_button_);
  root->addWidget(rqt_top_bar_);

  status_label_ = new QLabel(this);
  StylePanelStatusLabel(status_label_);
  status_label_->setWordWrap(true);
  root->addWidget(status_label_);

  payload_splitter_ = new QSplitter(config_.vertical_layout ? Qt::Vertical : Qt::Horizontal,
                                    this);
  payload_splitter_->setChildrenCollapsible(false);

  request_group_ = new QGroupBox(tr("Request"), payload_splitter_);
  auto* request_layout = new QVBoxLayout(request_group_);
  request_layout->setContentsMargins(8, 12, 8, 8);
  auto* request_toolbar = new QHBoxLayout();
  request_toolbar->setContentsMargins(0, 0, 0, 0);
  auto* request_expand = new QPushButton(tr("Expand all"), request_group_);
  request_expand->setFlat(true);
  auto* request_collapse = new QPushButton(tr("Collapse all"), request_group_);
  request_collapse->setFlat(true);
  auto* request_add_array = new QPushButton(QStringLiteral("+"), request_group_);
  request_add_array->setFlat(true);
  auto* request_remove_array = new QPushButton(QStringLiteral("-"), request_group_);
  request_remove_array->setFlat(true);
  request_toolbar->addWidget(request_expand);
  request_toolbar->addWidget(request_collapse);
  request_toolbar->addStretch();
  request_toolbar->addWidget(request_add_array);
  request_toolbar->addWidget(request_remove_array);
  request_tree_ = new publish_panel::PublishFieldTreeWidget(request_group_);
  request_tree_->setDisplayMode(publish_panel::PublishFieldTreeWidget::DisplayMode::kFieldsEditor);
  request_tree_->setValueColumnTitle(tr("Expression"));
  request_tree_->setMinimumHeight(140);
  request_layout->addLayout(request_toolbar);
  request_layout->addWidget(request_tree_, 1);
  payload_splitter_->addWidget(request_group_);

  response_group_ = new QGroupBox(tr("Response"), payload_splitter_);
  auto* response_layout = new QVBoxLayout(response_group_);
  response_layout->setContentsMargins(8, 12, 8, 8);
  auto* response_toolbar = new QHBoxLayout();
  response_toolbar->setContentsMargins(0, 0, 0, 0);
  auto* response_expand = new QPushButton(tr("Expand all"), response_group_);
  response_expand->setFlat(true);
  auto* response_collapse = new QPushButton(tr("Collapse all"), response_group_);
  response_collapse->setFlat(true);
  response_toolbar->addWidget(response_expand);
  response_toolbar->addWidget(response_collapse);
  response_toolbar->addStretch();
  response_tree_ = new publish_panel::PublishFieldTreeWidget(response_group_);
  response_tree_->setDisplayMode(publish_panel::PublishFieldTreeWidget::DisplayMode::kFieldsEditor);
  response_tree_->setValueColumnTitle(tr("Value"));
  response_tree_->setReadOnly(true);
  response_tree_->setMinimumHeight(140);
  response_layout->addLayout(response_toolbar);
  response_layout->addWidget(response_tree_, 1);
  payload_splitter_->addWidget(response_group_);
  payload_splitter_->setStretchFactor(0, 1);
  payload_splitter_->setStretchFactor(1, 1);
  root->addWidget(payload_splitter_, 1);

  advanced_body_ = new QWidget(this);
  auto* advanced_layout = new QVBoxLayout(advanced_body_);
  advanced_layout->setContentsMargins(0, 0, 0, 0);
  advanced_layout->setSpacing(6);
  auto* type_form = new QFormLayout();
  type_form->setContentsMargins(0, 0, 0, 0);
  request_type_combo_ = new QComboBox(advanced_body_);
  request_type_combo_->setEditable(true);
  request_type_combo_->setInsertPolicy(QComboBox::NoInsert);
  response_type_combo_ = new QComboBox(advanced_body_);
  response_type_combo_->setEditable(true);
  response_type_combo_->setInsertPolicy(QComboBox::NoInsert);
  reset_template_button_ = new QPushButton(tr("Use template"), advanced_body_);
  reset_template_button_->setFlat(true);
  auto* request_type_row = new QHBoxLayout();
  request_type_row->addWidget(request_type_combo_, 1);
  request_type_row->addWidget(reset_template_button_);
  type_form->addRow(tr("Request type"), request_type_row);
  type_form->addRow(tr("Response type"), response_type_combo_);
  advanced_layout->addLayout(type_form);

  auto* json_splitter = new QSplitter(Qt::Vertical, advanced_body_);
  json_splitter->setChildrenCollapsible(false);
  auto* request_json_group = new QGroupBox(tr("Request JSON"), advanced_body_);
  auto* request_json_layout = new QVBoxLayout(request_json_group);
  request_edit_ = makeJsonEditor(request_json_group, tr("Service request JSON"));
  request_json_layout->addWidget(request_edit_);
  json_splitter->addWidget(request_json_group);
  auto* response_json_group = new QGroupBox(tr("Response JSON"), advanced_body_);
  auto* response_json_layout = new QVBoxLayout(response_json_group);
  response_edit_ = makeJsonEditor(response_json_group, tr("Service response JSON"), true);
  response_json_layout->addWidget(response_edit_);
  json_splitter->addWidget(response_json_group);
  advanced_layout->addWidget(json_splitter, 1);
  root->addWidget(advanced_body_);

  connect(editing_mode_check_, &QCheckBox::toggled, this,
          &ServiceEditorWidget::onEditingModeToggled);
  connect(service_combo_, &QComboBox::currentTextChanged, this,
          &ServiceEditorWidget::onServiceChanged);
  connect(request_type_combo_, &QComboBox::currentTextChanged, this,
          &ServiceEditorWidget::onRequestTypeChanged);
  connect(response_type_combo_, &QComboBox::currentTextChanged, this,
          &ServiceEditorWidget::onFieldEdited);
  connect(reset_template_button_, &QPushButton::clicked, this,
          &ServiceEditorWidget::onResetTemplate);
  connect(refresh_services_button_, &QPushButton::clicked, this,
          &ServiceEditorWidget::onRefreshServices);
  connect(request_edit_, &QPlainTextEdit::textChanged, this,
          &ServiceEditorWidget::onFieldEdited);
  connect(call_button_, &QPushButton::clicked, this, &ServiceEditorWidget::onCallClicked);
  connect(request_tree_, &publish_panel::PublishFieldTreeWidget::messageEdited, this,
          &ServiceEditorWidget::onRequestTreeEdited);
  connect(request_expand, &QPushButton::clicked, request_tree_,
          &publish_panel::PublishFieldTreeWidget::expandAllFields);
  connect(request_collapse, &QPushButton::clicked, request_tree_,
          &publish_panel::PublishFieldTreeWidget::collapseAllFields);
  connect(request_add_array, &QPushButton::clicked, request_tree_,
          &publish_panel::PublishFieldTreeWidget::addArrayElement);
  connect(request_remove_array, &QPushButton::clicked, request_tree_,
          &publish_panel::PublishFieldTreeWidget::removeArrayElement);
  connect(response_expand, &QPushButton::clicked, response_tree_,
          &publish_panel::PublishFieldTreeWidget::expandAllFields);
  connect(response_collapse, &QPushButton::clicked, response_tree_,
          &publish_panel::PublishFieldTreeWidget::collapseAllFields);

  service_timer_ = new QTimer(this);
  service_timer_->setInterval(5000);
  connect(service_timer_, &QTimer::timeout, this, &ServiceEditorWidget::onRefreshServices);
  service_timer_->start();

  refreshServices();
  setConfig(config_);
}

QPlainTextEdit* ServiceEditorWidget::makeJsonEditor(QWidget* parent,
                                                    const QString& placeholder,
                                                    bool read_only) {
  auto* editor = new QPlainTextEdit(parent);
  editor->setPlaceholderText(placeholder);
  editor->setReadOnly(read_only);
  editor->setMinimumHeight(100);
  QFont mono = editor->font();
  mono.setFamily(QStringLiteral("Monospace"));
  mono.setPointSizeF(std::max(9.0, mono.pointSizeF() - 1.0));
  editor->setFont(mono);
  if (read_only) {
    editor->setStyleSheet(QStringLiteral("QPlainTextEdit { background: palette(alternate-base); }"));
  }
  return editor;
}

void ServiceEditorWidget::keyPressEvent(QKeyEvent* event) {
  if (event->matches(QKeySequence::InsertLineSeparator)) {
    onCallClicked();
    event->accept();
    return;
  }
  QWidget::keyPressEvent(event);
}

ServiceCallPanelConfig ServiceEditorWidget::config() const {
  ServiceCallPanelConfig out = config_;
  out.service_name = service_combo_->currentText().trimmed();
  out.request_type = request_type_combo_->currentText().trimmed();
  out.response_type = response_type_combo_->currentText().trimmed();
  out.request_json = request_edit_->toPlainText();
  out.response_json = response_edit_->toPlainText();
  out.editing_mode = editing_mode_check_->isChecked();
  return out;
}

void ServiceEditorWidget::setConfig(const ServiceCallPanelConfig& config) {
  config_ = config;
  suppress_template_update_ = true;

  editing_mode_check_->setChecked(config_.editing_mode);
  call_button_->setText(config_.button_label.isEmpty() ? tr("Call")
                                                       : config_.button_label);
  call_button_->setToolTip(config_.button_tooltip);
  applyButtonStyle();
  applyLayoutOrientation(config_.vertical_layout);

  const int service_index = service_combo_->findText(config_.service_name);
  if (service_index >= 0) {
    service_combo_->setCurrentIndex(service_index);
  } else {
    service_combo_->setEditText(config_.service_name);
  }

  request_type_combo_->setEditText(config_.request_type);
  response_type_combo_->setEditText(config_.response_type);
  request_edit_->setPlainText(config_.request_json);
  response_edit_->setPlainText(config_.response_json);

  if (config_.request_json.trimmed().isEmpty() &&
      !config_.request_type.trimmed().isEmpty()) {
    maybeFillTemplateForType(config_.request_type);
  } else {
    syncRequestTreeFromJson();
  }
  syncResponseTreeFromJson(config_.response_json);

  suppress_template_update_ = false;
  applyEditingModeUi();
  updateCallButtonState();
}

void ServiceEditorWidget::refreshServices() {
  const QString current = service_combo_->currentText();
  service_combo_->blockSignals(true);
  service_combo_->clear();
  for (const std::string& service : integration::ListServices()) {
    service_combo_->addItem(QString::fromStdString(service));
  }
  const int index = service_combo_->findText(current);
  if (index >= 0) {
    service_combo_->setCurrentIndex(index);
  } else if (!current.isEmpty()) {
    service_combo_->setEditText(current);
  }
  service_combo_->blockSignals(false);
}

void ServiceEditorWidget::applyLayoutOrientation(bool vertical) {
  if (payload_splitter_ != nullptr) {
    payload_splitter_->setOrientation(vertical ? Qt::Vertical : Qt::Horizontal);
  }
}

void ServiceEditorWidget::applyEditingModeUi() {
  const bool advanced = editing_mode_check_->isChecked();
  if (advanced_body_ != nullptr) {
    advanced_body_->setVisible(advanced);
  }
}

void ServiceEditorWidget::applyButtonStyle() {
  if (config_.button_color.isValid()) {
    call_button_->setStyleSheet(
        QStringLiteral(
            "QPushButton { background: %1; color: palette(button-text); border: none;"
            " border-radius: 4px; padding: 6px 12px; font-weight: 600; }"
            "QPushButton:hover { background: %2; }"
            "QPushButton:disabled { background: palette(mid); color: palette(midlight); }")
            .arg(config_.button_color.name(), config_.button_color.lighter(110).name()));
  } else {
    call_button_->setStyleSheet(
        QStringLiteral(
            "QPushButton { background: palette(highlight); color: palette(highlighted-text);"
            " border: none; border-radius: 4px; padding: 6px 12px; font-weight: 600; }"
            "QPushButton:hover { background: palette(highlight); }"
            "QPushButton:disabled { background: palette(mid); color: palette(midlight); }"));
  }
}

void ServiceEditorWidget::updateCallButtonState() {
  if (request_tree_->hasMessage()) {
    syncRequestJsonFromTree();
  }
  const ServiceCallPanelConfig current = config();
  const bool has_node =
      manager_ != nullptr && manager_->autolinkNode() != nullptr;
  const bool ready =
      !current.service_name.isEmpty() && !current.request_type.isEmpty() &&
      !current.request_json.trimmed().isEmpty();
  call_button_->setEnabled(has_node && ready && !call_in_progress_.load());

  if (call_in_progress_.load()) {
    updateStatus(tr("Waiting for service response…"), false);
  } else if (!has_node) {
    updateStatus(tr("Autolink node unavailable"), true);
  } else if (!ready) {
    updateStatus(tr("Select a service and configure the request"), false);
  } else {
    updateStatus(QString(), false);
  }
}

void ServiceEditorWidget::updateStatus(const QString& text, bool is_error) {
  status_label_->setText(text);
  StylePanelStatusLabel(status_label_, is_error);
}

void ServiceEditorWidget::resolveTypesForService(const QString& service_name) {
  if (service_name.isEmpty()) {
    return;
  }
  const std::string service = service_name.toStdString();
  std::string request_type;
  std::string response_type;
  integration::ResolveServiceMessageType(service, true, &request_type);
  integration::ResolveServiceMessageType(service, false, &response_type);

  suppress_template_update_ = true;
  if (!request_type.empty()) {
    request_type_combo_->setEditText(QString::fromStdString(request_type));
    config_.request_type = request_type_combo_->currentText().trimmed();
  }
  if (!response_type.empty()) {
    response_type_combo_->setEditText(QString::fromStdString(response_type));
    config_.response_type = response_type_combo_->currentText().trimmed();
  }
  suppress_template_update_ = false;

  if (request_edit_->toPlainText().trimmed().isEmpty()) {
    maybeFillTemplateForType(request_type_combo_->currentText());
  } else {
    syncRequestTreeFromJson();
  }
}

void ServiceEditorWidget::maybeFillTemplateForType(const QString& message_type) {
  if (suppress_template_update_ || message_type.trimmed().isEmpty()) {
    return;
  }
  const auto template_json =
      ServiceMessageCodec::instance().defaultJsonTemplate(
          message_type.trimmed().toStdString());
  if (!template_json.has_value()) {
    return;
  }
  request_edit_->setPlainText(*template_json);
  config_.request_json = *template_json;
  request_fields_dirty_ = true;
  syncRequestTreeFromJson();
}

void ServiceEditorWidget::syncRequestJsonFromTree() {
  if (!request_tree_->hasMessage()) {
    return;
  }
  const QString json = request_tree_->toJson();
  if (json.isEmpty()) {
    return;
  }
  suppress_template_update_ = true;
  request_edit_->setPlainText(json);
  config_.request_json = json;
  suppress_template_update_ = false;
  request_fields_dirty_ = false;
}

void ServiceEditorWidget::syncRequestTreeFromJson() {
  const QString message_type = request_type_combo_->currentText().trimmed();
  if (message_type.isEmpty()) {
    request_tree_->clearMessage();
    request_fields_dirty_ = false;
    return;
  }
  request_tree_->loadFromJson(message_type.toStdString(),
                              request_edit_->toPlainText());
  applyRequestServiceRootLabel();
  request_fields_dirty_ = false;
}

void ServiceEditorWidget::syncResponseTreeFromJson(const QString& json) {
  const QString response_type = response_type_combo_->currentText().trimmed();
  if (response_type.isEmpty() || json.trimmed().isEmpty()) {
    response_tree_->clearMessage();
    return;
  }
  response_tree_->loadFromJson(response_type.toStdString(), json);
  if (response_tree_->topLevelItemCount() > 0) {
    QTreeWidgetItem* root = response_tree_->topLevelItem(0);
    root->setText(0, service_combo_->currentText().trimmed());
    root->setText(1, ShortTypeLabel(response_type.toStdString()));
  }
  response_tree_->expandToDepth(1);
}

void ServiceEditorWidget::applyRequestServiceRootLabel() {
  if (request_tree_->topLevelItemCount() <= 0) {
    return;
  }
  QTreeWidgetItem* root = request_tree_->topLevelItem(0);
  const QString service_name = service_combo_->currentText().trimmed();
  const QString request_type = request_type_combo_->currentText().trimmed();
  if (!service_name.isEmpty()) {
    root->setText(0, service_name);
  }
  if (!request_type.isEmpty()) {
    root->setText(1, ShortTypeLabel(request_type.toStdString()));
  }
}

void ServiceEditorWidget::emitConfigChanged() { emit configChanged(); }

void ServiceEditorWidget::onEditingModeToggled(bool enabled) {
  config_.editing_mode = enabled;
  applyEditingModeUi();
  emitConfigChanged();
}

void ServiceEditorWidget::onServiceChanged(const QString& text) {
  config_.service_name = text.trimmed();
  resolveTypesForService(config_.service_name);
  applyRequestServiceRootLabel();
  updateCallButtonState();
  emitConfigChanged();
}

void ServiceEditorWidget::onRequestTypeChanged(const QString& text) {
  config_.request_type = text.trimmed();
  if (request_edit_->toPlainText().trimmed().isEmpty()) {
    maybeFillTemplateForType(config_.request_type);
  } else {
    syncRequestTreeFromJson();
  }
  applyRequestServiceRootLabel();
  updateCallButtonState();
  emitConfigChanged();
}

void ServiceEditorWidget::onResetTemplate() {
  maybeFillTemplateForType(request_type_combo_->currentText().trimmed());
  updateCallButtonState();
  emitConfigChanged();
}

void ServiceEditorWidget::onFieldEdited() {
  config_.request_json = request_edit_->toPlainText();
  config_.response_type = response_type_combo_->currentText().trimmed();
  request_fields_dirty_ = true;
  updateCallButtonState();
  emitConfigChanged();
}

void ServiceEditorWidget::onRequestTreeEdited() {
  syncRequestJsonFromTree();
  updateCallButtonState();
  emitConfigChanged();
}

void ServiceEditorWidget::onRefreshServices() { refreshServices(); }

void ServiceEditorWidget::finishCall(const ServiceCallPanelConfig& snapshot,
                                     bool ok, const QString& response_text,
                                     const QString& error_text) {
  call_in_progress_.store(false);
  if (ok) {
    response_edit_->setPlainText(response_text);
    config_.response_json = response_text;
    syncResponseTreeFromJson(response_text);
    updateStatus(tr("Service call succeeded"), false);
  } else {
    updateStatus(error_text, true);
  }
  updateCallButtonState();
  emit callFinished();
  emitConfigChanged();
}

void ServiceEditorWidget::onCallClicked() {
  if (call_in_progress_.load()) {
    return;
  }

  syncRequestJsonFromTree();
  const ServiceCallPanelConfig snapshot = config();
  const CodecResult encoded = ServiceMessageCodec::instance().encodeJson(
      snapshot.request_type.toStdString(), snapshot.request_json);
  if (!encoded.ok) {
    updateStatus(encoded.error, true);
    return;
  }

  call_in_progress_.store(true);
  updateCallButtonState();

  const std::chrono::seconds timeout(std::max(1, snapshot.timeout_sec));
  std::thread([this, snapshot, encoded, timeout]() {
    const integration::ServiceCallResult result =
        integration::ServiceClientRegistry::instance().call(
            snapshot.service_name.toStdString(), encoded.payload, timeout);

    QString response_text;
    QString error_text;
    bool call_ok = false;
    if (result.ok) {
      const std::string response_type = snapshot.response_type.isEmpty()
                                            ? snapshot.request_type.toStdString()
                                            : snapshot.response_type.toStdString();
      const CodecResult decoded = ServiceMessageCodec::instance().decodeToJson(
          response_type, result.response_bytes);
      if (decoded.ok) {
        response_text = decoded.text;
        call_ok = true;
      } else {
        error_text = decoded.error;
      }
    } else {
      error_text = QString::fromStdString(result.error);
    }

    QMetaObject::invokeMethod(
        this,
        [this, snapshot, call_ok, response_text, error_text]() {
          finishCall(snapshot, call_ok, response_text, error_text);
        },
        Qt::QueuedConnection);
  }).detach();
}

}  // namespace service_panel
}  // namespace autoviz
