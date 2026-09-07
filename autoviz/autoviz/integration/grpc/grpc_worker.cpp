/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/integration/grpc/grpc_worker.hpp"

#include "autoviz/integration/grpc/grpc_invoker.hpp"
#include "autoviz/integration/grpc/grpc_session.hpp"

namespace autoviz {
namespace integration {
namespace grpc_client {

GrpcWorker::GrpcWorker(QObject* parent) : QObject(parent) {}

void GrpcWorker::runUnary(const QString& target, bool use_tls, bool verify_cert,
                          const QString& ssl_override,
                          const QString& method_full_name,
                          const QString& request_json, bool include_defaults,
                          int timeout_ms) {
#if !AUTOVIZ_ENABLE_GRPC
  Q_UNUSED(target);
  Q_UNUSED(use_tls);
  Q_UNUSED(verify_cert);
  Q_UNUSED(ssl_override);
  Q_UNUSED(method_full_name);
  Q_UNUSED(request_json);
  Q_UNUSED(include_defaults);
  Q_UNUSED(timeout_ms);
  emit errorOccurred(QStringLiteral("gRPC disabled"));
  return;
#else
  if (store_ == nullptr || session_ == nullptr) {
    emit errorOccurred(QStringLiteral("gRPC worker not initialized"));
    return;
  }

  std::string err;
  if (!session_->connect(target.toStdString(), use_tls, verify_cert,
                         ssl_override.toStdString(), &err)) {
    emit errorOccurred(QString::fromStdString(
        err.empty() ? "Failed to connect" : err));
    return;
  }

  GrpcInvoker invoker(store_);
  const UnaryResult result = invoker.unaryCall(
      session_->channel(), method_full_name.toStdString(),
      request_json.toStdString(), include_defaults, timeout_ms, session_,
      &err);

  if (!err.empty() && result.response_json.empty() &&
      result.status.code == 0) {
    // Local failure before/after RPC (parse, codec, etc.).
    emit errorOccurred(QString::fromStdString(err));
    return;
  }

  emit unaryFinished(QString::fromStdString(result.response_json),
                     result.status.code,
                     QString::fromStdString(result.status.message),
                     static_cast<qint64>(result.status.latency_ms));
#endif
}

}  // namespace grpc_client
}  // namespace integration
}  // namespace autoviz
