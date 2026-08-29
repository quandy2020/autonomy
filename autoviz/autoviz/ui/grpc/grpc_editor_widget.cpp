/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/grpc/grpc_editor_widget.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFont>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QThread>
#include <QVBoxLayout>

#include <algorithm>
#include <string>
#include <vector>

#include "autoviz/integration/grpc/grpc_session.hpp"
#include "autoviz/integration/grpc/grpc_types.hpp"
#include "autoviz/integration/grpc/grpc_worker.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {
namespace grpc_panel {
namespace {

using integration::grpc_client::MethodInfo;
using integration::grpc_client::MethodType;

QPlainTextEdit* MakeJsonEditor(QWidget* parent, const QString& placeholder,
                               bool read_only = false) {
  auto* editor = new QPlainTextEdit(parent);
  editor->setPlaceholderText(placeholder);
  editor->setReadOnly(read_only);
  editor->setMinimumHeight(120);
  QFont mono = editor->font();
  mono.setFamily(QStringLiteral("Monospace"));
  mono.setPointSizeF(std::max(9.0, mono.pointSizeF() - 1.0));
  editor->setFont(mono);
  if (read_only) {
    editor->setStyleSheet(
        QStringLiteral("QPlainTextEdit { background: palette(alternate-base); }"));
  }
  return editor;
}

}  // namespace

GrpcEditorWidget::GrpcEditorWidget(QWidget* parent)
    : config_(DefaultGrpcPanelPersistConfig()), QWidget(parent) {
  ApplyPanelShell(this);
  setFocusPolicy(Qt::StrongFocus);

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin,
                           PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin);
  root->setSpacing(6);

#if !AUTOVIZ_ENABLE_GRPC
  disabled_banner_ = new QLabel(
      tr("gRPC support was not enabled at build time"), this);
  disabled_banner_->setWordWrap(true);
  disabled_banner_->setAlignment(Qt::AlignCenter);
  disabled_banner_->setStyleSheet(
      QStringLiteral("QLabel { background: palette(mid); color: palette(window-text);"
                     " border-radius: 4px; padding: 10px; font-weight: 600; }"));
  root->addWidget(disabled_banner_);
#endif

  auto* top_bar = new QWidget(this);
  auto* top_layout = new QHBoxLayout(top_bar);
  top_layout->setContentsMargins(0, 0, 0, 0);
  top_layout->setSpacing(6);

  tls_check_ = new QCheckBox(tr("TLS"), top_bar);
  tls_check_->setChecked(config_.tls);
  top_layout->addWidget(tls_check_);

  top_layout->addWidget(new QLabel(tr("URL:"), top_bar));
  url_combo_ = new QComboBox(top_bar);
  url_combo_->setEditable(true);
  url_combo_->setInsertPolicy(QComboBox::NoInsert);
  url_combo_->setMinimumWidth(180);
  url_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  url_combo_->setPlaceholderText(tr("host:port"));
  top_layout->addWidget(url_combo_, 2);

  top_layout->addWidget(new QLabel(tr("Method:"), top_bar));
  method_combo_ = new QComboBox(top_bar);
  method_combo_->setEditable(true);
  method_combo_->setInsertPolicy(QComboBox::NoInsert);
  method_combo_->setMinimumWidth(160);
  method_combo_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  method_combo_->setPlaceholderText(tr("package.Service/Method"));
  top_layout->addWidget(method_combo_, 2);

  invoke_button_ = new QPushButton(tr("Invoke"), top_bar);
  invoke_button_->setMinimumSize(88, 32);
  invoke_button_->setDefault(true);
  invoke_button_->setStyleSheet(
      QStringLiteral("QPushButton { font-weight: 600; padding: 4px 16px; }"));
  top_layout->addWidget(invoke_button_);
  root->addWidget(top_bar);

  status_label_ = new QLabel(this);
  StylePanelStatusLabel(status_label_);
  status_label_->setWordWrap(true);
  root->addWidget(status_label_);

  request_tabs_ = new QTabWidget(this);
  message_edit_ = MakeJsonEditor(request_tabs_, tr("gRPC request JSON"));
  request_tabs_->addTab(message_edit_, tr("Message"));

  auto* metadata_page = new QWidget(request_tabs_);
  auto* metadata_layout = new QVBoxLayout(metadata_page);
  metadata_placeholder_ =
      new QLabel(tr("Metadata editor coming in a later milestone"), metadata_page);
  metadata_placeholder_->setAlignment(Qt::AlignCenter);
  metadata_layout->addWidget(metadata_placeholder_);
  request_tabs_->addTab(metadata_page, tr("Metadata"));

  auto* definition_page = new QWidget(request_tabs_);
  auto* definition_layout = new QVBoxLayout(definition_page);
  definition_layout->setAlignment(Qt::AlignTop);
  load_proto_button_ = new QPushButton(tr("Load .proto…"), definition_page);
  load_automsgs_button_ = new QPushButton(tr("Load automsgs RPCs"), definition_page);
  refresh_reflection_button_ =
      new QPushButton(tr("Refresh reflection"), definition_page);
#if AUTOVIZ_ENABLE_GRPC
  load_proto_button_->setEnabled(true);
  load_automsgs_button_->setEnabled(true);
#else
  load_proto_button_->setEnabled(true);  // still clickable → error message
  load_automsgs_button_->setEnabled(true);
#endif
  refresh_reflection_button_->setEnabled(true);
  definition_layout->addWidget(load_proto_button_);
  definition_layout->addWidget(load_automsgs_button_);
  definition_layout->addWidget(refresh_reflection_button_);
  definition_layout->addStretch();
  request_tabs_->addTab(definition_page, tr("Service definition"));

  auto* response_page = new QWidget(request_tabs_);
  auto* response_layout = new QVBoxLayout(response_page);
  response_layout->setContentsMargins(0, 0, 0, 0);
  response_edit_ =
      MakeJsonEditor(response_page, tr("gRPC response JSON"), /*read_only=*/true);
  response_layout->addWidget(response_edit_);
  request_tabs_->addTab(response_page, tr("Response"));

  root->addWidget(request_tabs_, 1);

  worker_thread_ = new QThread(this);
  worker_ = new integration::grpc_client::GrpcWorker();
  worker_->setStore(&store_);
  worker_->setSession(&session_);
  worker_->moveToThread(worker_thread_);
  connect(worker_, &integration::grpc_client::GrpcWorker::unaryFinished, this,
          &GrpcEditorWidget::onUnaryFinished);
  connect(worker_, &integration::grpc_client::GrpcWorker::errorOccurred, this,
          &GrpcEditorWidget::onWorkerError);
  connect(this, &GrpcEditorWidget::requestUnary, worker_,
          &integration::grpc_client::GrpcWorker::runUnary);
  connect(worker_thread_, &QThread::finished, worker_, &QObject::deleteLater);
  worker_thread_->start();

  connect(tls_check_, &QCheckBox::toggled, this, &GrpcEditorWidget::onFieldEdited);
  connect(url_combo_, &QComboBox::currentTextChanged, this,
          &GrpcEditorWidget::onFieldEdited);
  connect(method_combo_, &QComboBox::currentTextChanged, this,
          &GrpcEditorWidget::onFieldEdited);
  connect(message_edit_, &QPlainTextEdit::textChanged, this,
          &GrpcEditorWidget::onFieldEdited);
  connect(invoke_button_, &QPushButton::clicked, this,
          &GrpcEditorWidget::onInvokeClicked);
  connect(load_proto_button_, &QPushButton::clicked, this,
          &GrpcEditorWidget::onLoadProtoClicked);
  connect(load_automsgs_button_, &QPushButton::clicked, this,
          &GrpcEditorWidget::onLoadAutomsgsClicked);
  connect(refresh_reflection_button_, &QPushButton::clicked, this,
          &GrpcEditorWidget::onRefreshReflectionClicked);

  setConfig(config_);
#if !AUTOVIZ_ENABLE_GRPC
  updateStatus(tr("gRPC support was not enabled at build time"), true);
#else
  updateStatus(tr("Ready"), false);
#endif
}

GrpcEditorWidget::~GrpcEditorWidget() {
  session_.cancel();
  if (worker_thread_ != nullptr) {
    worker_thread_->quit();
    worker_thread_->wait(5000);
  }
}

GrpcPanelPersistConfig GrpcEditorWidget::config() const {
  GrpcPanelPersistConfig out = config_;
  out.tls = tls_check_->isChecked();
  out.url = url_combo_->currentText().trimmed();
  out.method_full_name = selectedMethodFullName();
  out.message_json = message_edit_->toPlainText();
  return out;
}

void GrpcEditorWidget::setConfig(const GrpcPanelPersistConfig& config) {
  config_ = config;
  tls_check_->setChecked(config_.tls);

  const int url_index = url_combo_->findText(config_.url);
  if (url_index >= 0) {
    url_combo_->setCurrentIndex(url_index);
  } else {
    url_combo_->setEditText(config_.url);
  }

  refreshMethodCombo(config_.method_full_name);
  if (method_combo_->currentIndex() < 0 && !config_.method_full_name.isEmpty()) {
    method_combo_->setEditText(config_.method_full_name);
  }

  message_edit_->setPlainText(config_.message_json);
}

void GrpcEditorWidget::onInvokeClicked() {
#if !AUTOVIZ_ENABLE_GRPC
  updateStatus(tr("gRPC support was not enabled at build time"), true);
  return;
#else
  if (invoke_in_flight_) {
    return;
  }

  const QString method_full = selectedMethodFullName();
  if (method_full.isEmpty()) {
    updateStatus(tr("Select or enter a method"), true);
    return;
  }
  const QString url = url_combo_->currentText().trimmed();
  if (url.isEmpty()) {
    updateStatus(tr("Enter a server URL (host:port)"), true);
    return;
  }

  const std::string normalized =
      integration::grpc_client::NormalizeMethodFullName(method_full.toStdString());
  const auto* md = store_.findMethod(normalized);
  if (md != nullptr) {
    const MethodType type = store_.methodType(md);
    if (type != MethodType::kUnary) {
      updateStatus(tr("Streaming not implemented yet (Task 7)"), true);
      return;
    }
  }

  session_.cancel();
  setInvokeInFlight(true);
  updateStatus(tr("Invoking…"), false);

  emit requestUnary(url, tls_check_->isChecked(), config_.verify_cert,
                    config_.ssl_override, method_full, message_edit_->toPlainText(),
                    config_.include_defaults, config_.timeout_ms);
#endif
}

void GrpcEditorWidget::onFieldEdited() {
  config_.tls = tls_check_->isChecked();
  config_.url = url_combo_->currentText().trimmed();
  config_.method_full_name = selectedMethodFullName();
  config_.message_json = message_edit_->toPlainText();
  emitConfigChanged();
}

void GrpcEditorWidget::onLoadProtoClicked() {
#if !AUTOVIZ_ENABLE_GRPC
  updateStatus(tr("gRPC disabled"), true);
  return;
#else
  const QString path = QFileDialog::getOpenFileName(
      this, tr("Load .proto"), QString(),
      tr("Protocol Buffers (*.proto);;All Files (*)"));
  if (path.isEmpty()) {
    return;
  }

  const QFileInfo info(path);
  std::vector<std::string> includes;
  includes.push_back(info.absolutePath().toStdString());
  {
    QDir dir = info.dir();
    if (dir.cdUp()) {
      includes.push_back(dir.absolutePath().toStdString());
    }
  }

  std::string err;
  if (!store_.loadProtoFile(path.toStdString(), includes, &err)) {
    updateStatus(QString::fromStdString(err.empty() ? "Failed to load proto"
                                                    : err),
                 true);
    return;
  }
  refreshMethodCombo(selectedMethodFullName());
  updateStatus(tr("Loaded %1 method(s) from %2")
                   .arg(store_.listMethods().size())
                   .arg(info.fileName()),
               false);
  emitConfigChanged();
#endif
}

void GrpcEditorWidget::onLoadAutomsgsClicked() {
#if !AUTOVIZ_ENABLE_GRPC
  updateStatus(tr("gRPC disabled"), true);
  return;
#else
  std::string err;
  if (!store_.loadAutomsgsRpcs(&err)) {
    updateStatus(QString::fromStdString(err.empty() ? "Failed to load automsgs RPCs"
                                                    : err),
                 true);
    return;
  }
  refreshMethodCombo(selectedMethodFullName());
  updateStatus(tr("Loaded %1 automsgs RPC method(s)")
                   .arg(store_.listMethods().size()),
               false);
  emitConfigChanged();
#endif
}

void GrpcEditorWidget::onRefreshReflectionClicked() {
  updateStatus(tr("Reflection comes in Task 6"), false);
}

void GrpcEditorWidget::onUnaryFinished(const QString& response_json,
                                       int status_code,
                                       const QString& status_message,
                                       qint64 latency_ms) {
  setInvokeInFlight(false);
  response_edit_->setPlainText(response_json);
  const QString name = statusCodeName(status_code);
  QString text =
      tr("%1 %2 · %3ms").arg(status_code).arg(name).arg(latency_ms);
  if (!status_message.isEmpty() && status_code != 0) {
    text += QStringLiteral(" — ") + status_message;
  }
  updateStatus(text, status_code != 0);
  if (request_tabs_ != nullptr) {
    request_tabs_->setCurrentIndex(request_tabs_->count() - 1);  // Response
  }
}

void GrpcEditorWidget::onWorkerError(const QString& message) {
  setInvokeInFlight(false);
  updateStatus(message, true);
}

void GrpcEditorWidget::updateStatus(const QString& text, bool is_error) {
  status_label_->setText(text);
  StylePanelStatusLabel(status_label_, is_error);
}

void GrpcEditorWidget::emitConfigChanged() { emit configChanged(); }

void GrpcEditorWidget::refreshMethodCombo(const QString& preferred_full_name) {
  const QString preferred =
      preferred_full_name.isEmpty() ? selectedMethodFullName()
                                    : preferred_full_name;

  method_combo_->blockSignals(true);
  method_combo_->clear();

  std::vector<MethodInfo> methods = store_.listMethods();
  std::sort(methods.begin(), methods.end(),
            [](const MethodInfo& a, const MethodInfo& b) {
              return a.full_name < b.full_name;
            });

  int select_index = -1;
  for (const MethodInfo& info : methods) {
    const QString display =
        QString::fromStdString(info.service_name + " / " + info.method_name);
    const QString full = QString::fromStdString(info.full_name);
    method_combo_->addItem(display, full);
    if (select_index < 0 &&
        (full == preferred ||
         QString::fromStdString(info.service_name + "/" + info.method_name) ==
             preferred ||
         display == preferred)) {
      select_index = method_combo_->count() - 1;
    }
  }

  if (select_index >= 0) {
    method_combo_->setCurrentIndex(select_index);
  } else if (!preferred.isEmpty()) {
    method_combo_->setEditText(preferred);
  }
  method_combo_->blockSignals(false);
}

void GrpcEditorWidget::setInvokeInFlight(bool in_flight) {
  invoke_in_flight_ = in_flight;
  invoke_button_->setEnabled(!in_flight);
}

QString GrpcEditorWidget::selectedMethodFullName() const {
  const QVariant data = method_combo_->currentData();
  if (data.isValid() && data.canConvert<QString>() &&
      !data.toString().isEmpty()) {
    return data.toString().trimmed();
  }
  return method_combo_->currentText().trimmed();
}

QString GrpcEditorWidget::statusCodeName(int code) {
  switch (code) {
    case 0:
      return QStringLiteral("OK");
    case 1:
      return QStringLiteral("CANCELLED");
    case 2:
      return QStringLiteral("UNKNOWN");
    case 3:
      return QStringLiteral("INVALID_ARGUMENT");
    case 4:
      return QStringLiteral("DEADLINE_EXCEEDED");
    case 5:
      return QStringLiteral("NOT_FOUND");
    case 6:
      return QStringLiteral("ALREADY_EXISTS");
    case 7:
      return QStringLiteral("PERMISSION_DENIED");
    case 8:
      return QStringLiteral("RESOURCE_EXHAUSTED");
    case 9:
      return QStringLiteral("FAILED_PRECONDITION");
    case 10:
      return QStringLiteral("ABORTED");
    case 11:
      return QStringLiteral("OUT_OF_RANGE");
    case 12:
      return QStringLiteral("UNIMPLEMENTED");
    case 13:
      return QStringLiteral("INTERNAL");
    case 14:
      return QStringLiteral("UNAVAILABLE");
    case 15:
      return QStringLiteral("DATA_LOSS");
    case 16:
      return QStringLiteral("UNAUTHENTICATED");
    default:
      return QStringLiteral("CODE_%1").arg(code);
  }
}

}  // namespace grpc_panel
}  // namespace autoviz
