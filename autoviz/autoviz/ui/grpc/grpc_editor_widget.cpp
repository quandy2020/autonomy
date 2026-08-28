/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/grpc/grpc_editor_widget.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QFont>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QVBoxLayout>

#include <algorithm>

#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {
namespace grpc_panel {
namespace {

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
  load_proto_button_->setEnabled(false);
  load_automsgs_button_->setEnabled(false);
  refresh_reflection_button_->setEnabled(false);
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

  connect(tls_check_, &QCheckBox::toggled, this, &GrpcEditorWidget::onFieldEdited);
  connect(url_combo_, &QComboBox::currentTextChanged, this,
          &GrpcEditorWidget::onFieldEdited);
  connect(method_combo_, &QComboBox::currentTextChanged, this,
          &GrpcEditorWidget::onFieldEdited);
  connect(message_edit_, &QPlainTextEdit::textChanged, this,
          &GrpcEditorWidget::onFieldEdited);
  connect(invoke_button_, &QPushButton::clicked, this,
          &GrpcEditorWidget::onInvokeClicked);

  setConfig(config_);
#if !AUTOVIZ_ENABLE_GRPC
  updateStatus(tr("gRPC support was not enabled at build time"), true);
#else
  updateStatus(tr("Not connected (scaffold)"), false);
#endif
}

GrpcPanelPersistConfig GrpcEditorWidget::config() const {
  GrpcPanelPersistConfig out = config_;
  out.tls = tls_check_->isChecked();
  out.url = url_combo_->currentText().trimmed();
  out.method_full_name = method_combo_->currentText().trimmed();
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

  const int method_index = method_combo_->findText(config_.method_full_name);
  if (method_index >= 0) {
    method_combo_->setCurrentIndex(method_index);
  } else {
    method_combo_->setEditText(config_.method_full_name);
  }

  message_edit_->setPlainText(config_.message_json);
}

void GrpcEditorWidget::onInvokeClicked() {
  updateStatus(tr("Not connected (scaffold)"), false);
}

void GrpcEditorWidget::onFieldEdited() {
  config_.tls = tls_check_->isChecked();
  config_.url = url_combo_->currentText().trimmed();
  config_.method_full_name = method_combo_->currentText().trimmed();
  config_.message_json = message_edit_->toPlainText();
  emitConfigChanged();
}

void GrpcEditorWidget::updateStatus(const QString& text, bool is_error) {
  status_label_->setText(text);
  StylePanelStatusLabel(status_label_, is_error);
}

void GrpcEditorWidget::emitConfigChanged() { emit configChanged(); }

}  // namespace grpc_panel
}  // namespace autoviz
