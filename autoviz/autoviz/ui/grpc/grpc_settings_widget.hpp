/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>

#include "autoviz/ui/grpc/grpc_types.hpp"

class QCheckBox;
class QLineEdit;
class QSpinBox;

namespace autoviz {
namespace grpc_panel {

class GrpcSettingsWidget : public QWidget {
  Q_OBJECT

 public:
  explicit GrpcSettingsWidget(QWidget* parent = nullptr);

  GrpcPanelPersistConfig config() const;
  void setConfig(const GrpcPanelPersistConfig& config);

 signals:
  void configChanged();

 private:
  void emitConfigChanged();

  GrpcPanelPersistConfig config_;
  QLineEdit* title_edit_ = nullptr;
  QSpinBox* timeout_spin_ = nullptr;
  QCheckBox* verify_cert_check_ = nullptr;
  QLineEdit* ssl_override_edit_ = nullptr;
  QCheckBox* include_defaults_check_ = nullptr;
  QSpinBox* max_response_spin_ = nullptr;
};

}  // namespace grpc_panel
}  // namespace autoviz
