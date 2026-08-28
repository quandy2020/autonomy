/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/grpc/grpc_settings_widget.hpp"

#include <QCheckBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLineEdit>
#include <QSpinBox>
#include <QVBoxLayout>

#include <algorithm>

#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {
namespace grpc_panel {

GrpcSettingsWidget::GrpcSettingsWidget(QWidget* parent)
    : config_(DefaultGrpcPanelPersistConfig()), QWidget(parent) {
  ApplyCompactSettingsShell(this);
  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin,
                            PanelSettingsLayout::kOuterMargin, PanelSettingsLayout::kOuterMargin);
  outer->setSpacing(PanelSettingsLayout::kOuterSpacing);
  outer->setAlignment(Qt::AlignTop);

  auto* general = new QGroupBox(tr("General"), this);
  StyleSettingsGroupBox(general);
  auto* general_form = new QFormLayout(general);
  ApplyCompactForm(general_form);
  title_edit_ = new QLineEdit(config_.title, general);
  timeout_spin_ = new QSpinBox(general);
  timeout_spin_->setRange(100, 600000);
  timeout_spin_->setSingleStep(100);
  timeout_spin_->setSuffix(tr(" ms"));
  timeout_spin_->setValue(config_.timeout_ms);
  general_form->addRow(tr("Title"), title_edit_);
  general_form->addRow(tr("Timeout"), timeout_spin_);
  outer->addWidget(general);

  auto* tls_group = new QGroupBox(tr("TLS"), this);
  StyleSettingsGroupBox(tls_group);
  auto* tls_form = new QFormLayout(tls_group);
  ApplyCompactForm(tls_form);
  verify_cert_check_ = new QCheckBox(tr("Verify certificate"), tls_group);
  verify_cert_check_->setChecked(config_.verify_cert);
  ssl_override_edit_ = new QLineEdit(config_.ssl_override, tls_group);
  ssl_override_edit_->setPlaceholderText(tr("SNI / server name override"));
  tls_form->addRow(QString(), verify_cert_check_);
  tls_form->addRow(tr("SSL override"), ssl_override_edit_);
  outer->addWidget(tls_group);

  auto* response_group = new QGroupBox(tr("Response"), this);
  StyleSettingsGroupBox(response_group);
  auto* response_form = new QFormLayout(response_group);
  ApplyCompactForm(response_form);
  include_defaults_check_ = new QCheckBox(tr("Include default fields"), response_group);
  include_defaults_check_->setChecked(config_.include_defaults);
  max_response_spin_ = new QSpinBox(response_group);
  max_response_spin_->setRange(1, 512);
  max_response_spin_->setSuffix(tr(" MB"));
  max_response_spin_->setValue(config_.max_response_mb);
  response_form->addRow(QString(), include_defaults_check_);
  response_form->addRow(tr("Max response"), max_response_spin_);
  outer->addWidget(response_group);
  outer->addStretch();

  connect(title_edit_, &QLineEdit::textChanged, this,
          &GrpcSettingsWidget::emitConfigChanged);
  connect(timeout_spin_, qOverload<int>(&QSpinBox::valueChanged), this,
          &GrpcSettingsWidget::emitConfigChanged);
  connect(verify_cert_check_, &QCheckBox::toggled, this,
          &GrpcSettingsWidget::emitConfigChanged);
  connect(ssl_override_edit_, &QLineEdit::textChanged, this,
          &GrpcSettingsWidget::emitConfigChanged);
  connect(include_defaults_check_, &QCheckBox::toggled, this,
          &GrpcSettingsWidget::emitConfigChanged);
  connect(max_response_spin_, qOverload<int>(&QSpinBox::valueChanged), this,
          &GrpcSettingsWidget::emitConfigChanged);

  setConfig(config_);
}

GrpcPanelPersistConfig GrpcSettingsWidget::config() const {
  GrpcPanelPersistConfig out = config_;
  out.title = title_edit_->text().trimmed();
  out.timeout_ms = timeout_spin_->value();
  out.verify_cert = verify_cert_check_->isChecked();
  out.ssl_override = ssl_override_edit_->text().trimmed();
  out.include_defaults = include_defaults_check_->isChecked();
  out.max_response_mb = max_response_spin_->value();
  return out;
}

void GrpcSettingsWidget::setConfig(const GrpcPanelPersistConfig& config) {
  config_ = config;
  title_edit_->setText(config_.title);
  timeout_spin_->setValue(std::max(100, config_.timeout_ms));
  verify_cert_check_->setChecked(config_.verify_cert);
  ssl_override_edit_->setText(config_.ssl_override);
  include_defaults_check_->setChecked(config_.include_defaults);
  max_response_spin_->setValue(std::max(1, config_.max_response_mb));
}

void GrpcSettingsWidget::emitConfigChanged() { emit configChanged(); }

}  // namespace grpc_panel
}  // namespace autoviz
