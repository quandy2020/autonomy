/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/service/service_editor_widget.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QFont>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QTimer>
#include <QVBoxLayout>

#include <chrono>
#include <thread>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/service_client_registry.hpp"
#include "autoviz/integration/service_discovery.hpp"
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

QPlainTextEdit* MakeJsonEditor(QWidget* parent, const QString& placeholder) {
  auto* editor = new QPlainTextEdit(parent);
  editor->setPlaceholderText(placeholder);
  editor->setMinimumHeight(120);
  QFont mono = editor->font();
  mono.setFamily(QStringLiteral("Monospace"));
  mono.setPointSizeF(std::max(9.0, mono.pointSizeF() - 1.0));
  editor->setFont(mono);
  return editor;
}

}  // namespace

ServiceEditorWidget::ServiceEditorWidget(common::VisualizationManager* manager,
                                         QWidget* parent)
    : manager_(manager), config_(DefaultServiceCallPanelConfig()), QWidget(parent) {
  setAttribute(Qt::WA_StyledBackground, true);
  setStyleSheet(QStringLiteral("ServiceEditorWidget { background: palette(window); }"));

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(8, 8, 8, 8);
  root->setSpacing(8);

  auto* header = new QHBoxLayout();
  editing_mode_check_ = new QCheckBox(tr("Editing mode"), this);
  editing_mode_check_->setChecked(config_.editing_mode);
  header->addWidget(editing_mode_check_);
  header->addStretch();
  status_label_ = new QLabel(this);
  status_label_->setStyleSheet(QStringLiteral("color: palette(mid); font-size: 10px;"));
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

  service_combo_ = new QComboBox(editor_body_);
  service_combo_->setEditable(true);
  service_combo_->setInsertPolicy(QComboBox::NoInsert);
  form->addRow(tr("Service name"), service_combo_);

  auto* request_type_row = new QHBoxLayout();
  request_type_combo_ = new QComboBox(editor_body_);
  request_type_combo_->setEditable(true);
  request_type_combo_->setInsertPolicy(QComboBox::NoInsert);
  reset_template_button_ = new QPushButton(tr("Reset template"), editor_body_);
  reset_template_button_->setFlat(true);
  request_type_row->addWidget(request_type_combo_, 1);
  request_type_row->addWidget(reset_template_button_);
  form->addRow(tr("Request type"), request_type_row);

  response_type_combo_ = new QComboBox(editor_body_);
  response_type_combo_->setEditable(true);
  response_type_combo_->setInsertPolicy(QComboBox::NoInsert);
  form->addRow(tr("Response type"), response_type_combo_);

  payload_splitter_ = new QSplitter(config_.vertical_layout ? Qt::Vertical
                                                            : Qt::Horizontal,
                                    editor_body_);
  payload_splitter_->setChildrenCollapsible(false);

  auto* request_group = new QGroupBox(tr("Request"), editor_body_);
  auto* request_layout = new QVBoxLayout(request_group);
  request_edit_ = MakeJsonEditor(request_group, tr("Service request JSON"));
  request_layout->addWidget(request_edit_);
  payload_splitter_->addWidget(request_group);

  auto* response_group = new QGroupBox(tr("Response"), editor_body_);
  auto* response_layout = new QVBoxLayout(response_group);
  response_edit_ = MakeJsonEditor(response_group, tr("Service response JSON"));
  response_edit_->setReadOnly(true);
  response_layout->addWidget(response_edit_);
  payload_splitter_->addWidget(response_group);
  payload_splitter_->setStretchFactor(0, 1);
  payload_splitter_->setStretchFactor(1, 1);

  form->addRow(payload_splitter_);

  scroll->setWidget(editor_body_);
  root->addWidget(scroll, 1);

  call_button_ = new QPushButton(config_.button_label, this);
  call_button_->setCursor(Qt::PointingHandCursor);
  call_button_->setMinimumHeight(32);
  root->addWidget(call_button_);

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
  connect(request_edit_, &QPlainTextEdit::textChanged, this,
          &ServiceEditorWidget::onFieldEdited);
  connect(call_button_, &QPushButton::clicked, this,
          &ServiceEditorWidget::onCallClicked);

  service_timer_ = new QTimer(this);
  service_timer_->setInterval(2000);
  connect(service_timer_, &QTimer::timeout, this,
          &ServiceEditorWidget::onRefreshServicesTimer);
  service_timer_->start();

  refreshServices();
  setConfig(config_);
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
  call_button_->setText(config_.button_label.isEmpty() ? tr("Call service")
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
  }

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
  if (payload_splitter_ == nullptr) {
    return;
  }
  payload_splitter_->setOrientation(vertical ? Qt::Vertical : Qt::Horizontal);
}

void ServiceEditorWidget::applyEditingModeUi() {
  const bool editing = editing_mode_check_->isChecked();
  editor_body_->setVisible(editing);
  response_edit_->setVisible(editing);
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
    updateStatus(tr("Configure service name, type, and request JSON"), false);
  } else {
    updateStatus(QString(), false);
  }
}

void ServiceEditorWidget::updateStatus(const QString& text, bool is_error) {
  status_label_->setText(text);
  status_label_->setStyleSheet(is_error
                                   ? QStringLiteral("color: #c07070; font-size: 10px;")
                                   : QStringLiteral("color: palette(mid); font-size: 10px;"));
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
  if (!request_type.empty() && request_type_combo_->currentText().trimmed().isEmpty()) {
    request_type_combo_->setEditText(QString::fromStdString(request_type));
  }
  if (!response_type.empty() &&
      response_type_combo_->currentText().trimmed().isEmpty()) {
    response_type_combo_->setEditText(QString::fromStdString(response_type));
  }
  suppress_template_update_ = false;

  if (request_edit_->toPlainText().trimmed().isEmpty()) {
    maybeFillTemplateForType(request_type_combo_->currentText());
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
  updateCallButtonState();
  emitConfigChanged();
}

void ServiceEditorWidget::onRequestTypeChanged(const QString& text) {
  config_.request_type = text.trimmed();
  if (request_edit_->toPlainText().trimmed().isEmpty()) {
    maybeFillTemplateForType(config_.request_type);
  }
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
  updateCallButtonState();
  emitConfigChanged();
}

void ServiceEditorWidget::onRefreshServicesTimer() { refreshServices(); }

void ServiceEditorWidget::finishCall(const ServiceCallPanelConfig& snapshot,
                                     bool ok, const QString& response_text,
                                     const QString& error_text) {
  call_in_progress_.store(false);
  if (ok) {
    response_edit_->setPlainText(response_text);
    config_.response_json = response_text;
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

  const ServiceCallPanelConfig snapshot = config();
  const CodecResult encoded = ServiceMessageCodec::instance().encodeJson(
      snapshot.request_type.toStdString(), snapshot.request_json);
  if (!encoded.ok) {
    updateStatus(encoded.error, true);
    return;
  }

  call_in_progress_.store(true);
  updateCallButtonState();

  const std::chrono::seconds timeout(
      std::max(1, snapshot.timeout_sec));
  std::thread([this, snapshot, encoded, timeout]() {
    const integration::ServiceCallResult result =
        integration::ServiceClientRegistry::instance().call(
            snapshot.service_name.toStdString(), encoded.payload, timeout);

    QString response_text;
    QString error_text;
    bool ok = false;
    if (result.ok) {
      const std::string response_type = snapshot.response_type.isEmpty()
                                            ? snapshot.request_type.toStdString()
                                            : snapshot.response_type.toStdString();
      const CodecResult decoded = ServiceMessageCodec::instance().decodeToJson(
          response_type, result.response_bytes);
      if (decoded.ok) {
        response_text = decoded.text;
        ok = true;
      } else {
        error_text = decoded.error;
      }
    } else {
      error_text = QString::fromStdString(result.error);
    }

    QMetaObject::invokeMethod(
        this,
        [this, snapshot, ok, response_text, error_text]() {
          finishCall(snapshot, ok, response_text, error_text);
        },
        Qt::QueuedConnection);
  }).detach();
}

}  // namespace service_panel
}  // namespace autoviz
