/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <optional>
#include <string>

#include <QString>

namespace autoviz {
namespace service_panel {

struct CodecResult {
  bool ok = false;
  QString error;
  std::string payload;
  QString text;
};

class ServiceMessageCodec {
 public:
  static ServiceMessageCodec& instance();

  std::optional<QString> defaultJsonTemplate(const std::string& message_type) const;
  CodecResult encodeJson(const std::string& message_type,
                         const QString& message_json) const;
  CodecResult decodeToJson(const std::string& message_type,
                           const std::string& bytes) const;

 private:
  ServiceMessageCodec() = default;
};

}  // namespace service_panel
}  // namespace autoviz
