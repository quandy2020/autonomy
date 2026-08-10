/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/publish/publish_message_codec.hpp"

#include <automsgs/msgs/DynamicFactory.hh>

#include <google/protobuf/util/json_util.h>

#include <algorithm>

#include "autoviz/common/protobuf_json_compat.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"

namespace autoviz {
namespace publish_panel {
namespace {

using automsgs::msgs::DynamicFactory;

std::string StripPackagePrefix(const std::string& type_name) {
  static const char* kPrefix = "automsgs.msgs.";
  if (type_name.rfind(kPrefix, 0) == 0) {
    return type_name.substr(std::char_traits<char>::length(kPrefix));
  }
  return type_name;
}

DynamicFactory::MessagePtr CreateMessage(const std::string& message_type) {
  static DynamicFactory factory;
  const std::string normalized = commsgs::NormalizeMessageType(message_type);
  DynamicFactory::MessagePtr message = factory.New(normalized);
  if (message == nullptr) {
    message = factory.New(StripPackagePrefix(normalized));
  }
  return message;
}

}  // namespace

PublishMessageCodec& PublishMessageCodec::instance() {
  static PublishMessageCodec codec;
  return codec;
}

std::vector<std::string> PublishMessageCodec::listMessageTypes() const {
  static DynamicFactory factory;
  std::vector<std::string> types;
  factory.Types(types);
  std::sort(types.begin(), types.end());
  return types;
}

std::optional<QString> PublishMessageCodec::defaultJsonTemplate(
    const std::string& message_type) const {
  DynamicFactory::MessagePtr message = CreateMessage(message_type);
  if (message == nullptr) {
    return std::nullopt;
  }

  google::protobuf::util::JsonPrintOptions options;
  options.add_whitespace = true;
  SetAlwaysPrintPrimitiveFields(&options);
  options.preserve_proto_field_names = true;

  std::string json;
  const auto status =
      google::protobuf::util::MessageToJsonString(*message, &json, options);
  if (!status.ok()) {
    return std::nullopt;
  }
  return QString::fromStdString(json);
}

CodecResult PublishMessageCodec::encodeJson(const std::string& message_type,
                                            const QString& message_json) const {
  CodecResult result;
  if (message_type.empty()) {
    result.error = QStringLiteral("Message type is required");
    return result;
  }
  const QString trimmed = message_json.trimmed();
  if (trimmed.isEmpty()) {
    result.error = QStringLiteral("Message JSON is empty");
    return result;
  }

  DynamicFactory::MessagePtr message = CreateMessage(message_type);
  if (message == nullptr) {
    result.error = QStringLiteral("Unknown message type");
    return result;
  }

  google::protobuf::util::JsonParseOptions options;
  options.ignore_unknown_fields = true;
  const auto status = google::protobuf::util::JsonStringToMessage(
      trimmed.toStdString(), message.get(), options);
  if (!status.ok()) {
    result.error = StatusMessageToQString(status);
    return result;
  }

  if (!message->SerializeToString(&result.payload)) {
    result.error = QStringLiteral("Failed to serialize message");
    return result;
  }
  result.ok = true;
  return result;
}

}  // namespace publish_panel
}  // namespace autoviz
