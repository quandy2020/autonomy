/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <optional>
#include <string>
#include <vector>

#include <QString>

namespace autoviz {
namespace publish_panel {

struct CodecResult {
  bool ok = false;
  QString error;
  std::string payload;
};

class PublishMessageCodec {
 public:
  static PublishMessageCodec& instance();

  std::vector<std::string> listMessageTypes() const;
  std::optional<QString> defaultJsonTemplate(const std::string& message_type) const;
  CodecResult encodeJson(const std::string& message_type,
                         const QString& message_json) const;

 private:
  PublishMessageCodec() = default;
};

}  // namespace publish_panel
}  // namespace autoviz
