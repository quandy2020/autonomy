/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifndef AUTOVIZ_ENABLE_GRPC
#define AUTOVIZ_ENABLE_GRPC 0
#endif

#include <QObject>
#include <QString>

namespace autoviz {
namespace integration {
namespace grpc_client {

class GrpcDescriptorStore;
class GrpcSession;

/** Runs unary RPCs on a worker QThread; results via queued signals. */
class GrpcWorker : public QObject {
  Q_OBJECT

 public:
  explicit GrpcWorker(QObject* parent = nullptr);

  void setStore(GrpcDescriptorStore* store) { store_ = store; }
  void setSession(GrpcSession* session) { session_ = session; }

 public slots:
  void runUnary(const QString& target, bool use_tls, bool verify_cert,
                const QString& ssl_override, const QString& method_full_name,
                const QString& request_json, bool include_defaults,
                int timeout_ms);

 signals:
  void unaryFinished(const QString& response_json, int status_code,
                     const QString& status_message, qint64 latency_ms);
  void errorOccurred(const QString& message);

 private:
  GrpcDescriptorStore* store_ = nullptr;  // non-owning
  GrpcSession* session_ = nullptr;        // non-owning
};

}  // namespace grpc_client
}  // namespace integration
}  // namespace autoviz
