/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifndef AUTOVIZ_ENABLE_GRPC
#define AUTOVIZ_ENABLE_GRPC 0
#endif

#include <memory>

#include <QWidget>

#include "autoviz/integration/grpc/grpc_descriptor_store.hpp"
#include "autoviz/integration/grpc/grpc_session.hpp"
#include "autoviz/ui/grpc/grpc_types.hpp"

class QCheckBox;
class QComboBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QTabWidget;
class QThread;

namespace autoviz {

namespace integration {
namespace grpc_client {
class GrpcWorker;
}  // namespace grpc_client
}  // namespace integration

namespace grpc_panel {

class GrpcEditorWidget : public QWidget {
  Q_OBJECT

 public:
  explicit GrpcEditorWidget(QWidget* parent = nullptr);
  ~GrpcEditorWidget() override;

  GrpcPanelPersistConfig config() const;
  void setConfig(const GrpcPanelPersistConfig& config);

 signals:
  void configChanged();
  void requestUnary(const QString& target, bool use_tls, bool verify_cert,
                    const QString& ssl_override, const QString& method_full_name,
                    const QString& request_json, bool include_defaults,
                    int timeout_ms);

 private slots:
  void onInvokeClicked();
  void onFieldEdited();
  void onLoadProtoClicked();
  void onLoadAutomsgsClicked();
  void onRefreshReflectionClicked();
  void onUnaryFinished(const QString& response_json, int status_code,
                        const QString& status_message, qint64 latency_ms);
  void onWorkerError(const QString& message);

 private:
  void updateStatus(const QString& text, bool is_error = false);
  void emitConfigChanged();
  void refreshMethodCombo(const QString& preferred_full_name = QString());
  void setInvokeInFlight(bool in_flight);
  QString selectedMethodFullName() const;
  static QString statusCodeName(int code);

  GrpcPanelPersistConfig config_;

  integration::grpc_client::GrpcDescriptorStore store_;
  integration::grpc_client::GrpcSession session_;
  QThread* worker_thread_ = nullptr;
  integration::grpc_client::GrpcWorker* worker_ = nullptr;
  bool invoke_in_flight_ = false;

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
