/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifndef AUTOVIZ_ENABLE_GRPC
#define AUTOVIZ_ENABLE_GRPC 0
#endif

#include <QWidget>

#include "autoviz/ui/grpc/grpc_types.hpp"

class QCheckBox;
class QComboBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QTabWidget;

namespace autoviz {
namespace grpc_panel {

class GrpcEditorWidget : public QWidget {
  Q_OBJECT

 public:
  explicit GrpcEditorWidget(QWidget* parent = nullptr);

  GrpcPanelPersistConfig config() const;
  void setConfig(const GrpcPanelPersistConfig& config);

 signals:
  void configChanged();

 private slots:
  void onInvokeClicked();
  void onFieldEdited();

 private:
  void updateStatus(const QString& text, bool is_error = false);
  void emitConfigChanged();

  GrpcPanelPersistConfig config_;

  QLabel* disabled_banner_ = nullptr;
  QCheckBox* tls_check_ = nullptr;
  QComboBox* url_combo_ = nullptr;
  QComboBox* method_combo_ = nullptr;
  QPushButton* invoke_button_ = nullptr;
  QTabWidget* request_tabs_ = nullptr;
  QPlainTextEdit* message_edit_ = nullptr;
  QLabel* metadata_placeholder_ = nullptr;
  QPushButton* load_proto_button_ = nullptr;
  QPushButton* load_automsgs_button_ = nullptr;
  QPushButton* refresh_reflection_button_ = nullptr;
  QPlainTextEdit* response_edit_ = nullptr;
  QLabel* status_label_ = nullptr;
};

}  // namespace grpc_panel
}  // namespace autoviz
