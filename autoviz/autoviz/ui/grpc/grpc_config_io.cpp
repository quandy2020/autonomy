/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/grpc/grpc_config_io.hpp"

#include <QPair>

namespace autoviz {
namespace grpc_panel {

common::GrpcPanelPersistConfig ToPersistConfig(
    const QString& object_name, const GrpcPanelPersistConfig& config) {
  common::GrpcPanelPersistConfig persist;
  persist.object_name = object_name.toStdString();
  persist.title = config.title.toStdString();
  persist.url = config.url.toStdString();
  persist.tls = config.tls;
  persist.method_full_name = config.method_full_name.toStdString();
  persist.message_json = config.message_json.toStdString();
  persist.timeout_ms = config.timeout_ms;
  persist.verify_cert = config.verify_cert;
  persist.ssl_override = config.ssl_override.toStdString();
  persist.include_defaults = config.include_defaults;
  persist.max_response_mb = config.max_response_mb;
  persist.definition_mode = config.definition_mode.toStdString();
  persist.settings_visible = config.settings_visible;
  persist.metadata.reserve(static_cast<std::size_t>(config.metadata.size()));
  for (const auto& kv : config.metadata) {
    persist.metadata.emplace_back(kv.first.toStdString(),
                                  kv.second.toStdString());
  }
  persist.proto_paths.reserve(
      static_cast<std::size_t>(config.proto_paths.size()));
  for (const QString& path : config.proto_paths) {
    persist.proto_paths.push_back(path.toStdString());
  }
  persist.url_history.reserve(
      static_cast<std::size_t>(config.url_history.size()));
  for (const QString& url : config.url_history) {
    persist.url_history.push_back(url.toStdString());
  }
  return persist;
}

GrpcPanelPersistConfig FromPersistConfig(
    const common::GrpcPanelPersistConfig& persist) {
  GrpcPanelPersistConfig config = DefaultGrpcPanelPersistConfig();
  config.title = QString::fromStdString(persist.title);
  config.url = QString::fromStdString(persist.url);
  config.tls = persist.tls;
  config.method_full_name = QString::fromStdString(persist.method_full_name);
  config.message_json = QString::fromStdString(persist.message_json);
  if (config.message_json.isEmpty()) {
    config.message_json = QStringLiteral("{}");
  }
  config.timeout_ms = persist.timeout_ms;
  config.verify_cert = persist.verify_cert;
  config.ssl_override = QString::fromStdString(persist.ssl_override);
  config.include_defaults = persist.include_defaults;
  config.max_response_mb = persist.max_response_mb;
  config.definition_mode = QString::fromStdString(persist.definition_mode);
  if (config.definition_mode.isEmpty()) {
    config.definition_mode = QStringLiteral("none");
  }
  config.settings_visible = persist.settings_visible;
  config.metadata.reserve(static_cast<int>(persist.metadata.size()));
  for (const auto& kv : persist.metadata) {
    config.metadata.push_back(
        qMakePair(QString::fromStdString(kv.first),
                  QString::fromStdString(kv.second)));
  }
  for (const std::string& path : persist.proto_paths) {
    config.proto_paths.push_back(QString::fromStdString(path));
  }
  for (const std::string& url : persist.url_history) {
    config.url_history.push_back(QString::fromStdString(url));
  }
  return config;
}

}  // namespace grpc_panel
}  // namespace autoviz
