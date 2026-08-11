/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <QString>

namespace google {
namespace protobuf {
class Message;
}  // namespace protobuf
}  // namespace google

namespace autoviz {
namespace publish_panel {

struct CodecResult {
  bool ok = false;
  QString error;
  QString text;
  std::string payload;
};

/** Create a protobuf message instance (generated pool first, then DynamicFactory). */
std::unique_ptr<google::protobuf::Message> CreatePublishMessage(
    const std::string& message_type);

class PublishMessageCodec {
 public:
  static PublishMessageCodec& instance();

  std::vector<std::string> listMessageTypes() const;
  std::optional<QString> defaultJsonTemplate(const std::string& message_type) const;
  CodecResult encodeJson(const std::string& message_type,
                         const QString& message_json) const;
  CodecResult decodePayloadToJson(const std::string& message_type,
                                  const std::string& bytes) const;

 private:
  PublishMessageCodec() = default;
};

}  // namespace publish_panel
}  // namespace autoviz
