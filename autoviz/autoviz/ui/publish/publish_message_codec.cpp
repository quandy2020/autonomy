/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/publish/publish_message_codec.hpp"

#include <automsgs/msgs/DynamicFactory.hh>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>
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

std::unique_ptr<google::protobuf::Message> CreateFromGeneratedPool(
    const std::string& type_name) {
  if (type_name.empty()) {
    return nullptr;
  }
  const google::protobuf::DescriptorPool* pool =
      google::protobuf::DescriptorPool::generated_pool();
  if (pool == nullptr) {
    return nullptr;
  }
  const google::protobuf::Descriptor* desc = pool->FindMessageTypeByName(type_name);
  if (desc == nullptr) {
    return nullptr;
  }
  const google::protobuf::Message* prototype =
      google::protobuf::MessageFactory::generated_factory()->GetPrototype(desc);
  if (prototype == nullptr) {
    return nullptr;
  }
  return std::unique_ptr<google::protobuf::Message>(prototype->New());
}

}  // namespace

std::unique_ptr<google::protobuf::Message> CreatePublishMessage(
    const std::string& message_type) {
  if (message_type.empty()) {
    return nullptr;
  }
  const std::string normalized = commsgs::NormalizeMessageType(message_type);
  if (auto message = CreateFromGeneratedPool(normalized)) {
    return message;
  }
  if (const std::string short_name = StripPackagePrefix(normalized);
      short_name != normalized) {
    if (auto message = CreateFromGeneratedPool(short_name)) {
      return message;
    }
  }

  static DynamicFactory factory;
  DynamicFactory::MessagePtr message = factory.New(normalized);
  if (message == nullptr && !normalized.empty()) {
    message = factory.New(StripPackagePrefix(normalized));
  }
  return message;
}

namespace {

DynamicFactory::MessagePtr CreateMessage(const std::string& message_type) {
  std::unique_ptr<google::protobuf::Message> message =
      CreatePublishMessage(message_type);
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

CodecResult PublishMessageCodec::decodePayloadToJson(
    const std::string& message_type, const std::string& bytes) const {
  CodecResult result;
  if (message_type.empty()) {
    result.error = QStringLiteral("Message type is required");
    return result;
  }
  if (bytes.empty()) {
    result.error = QStringLiteral("Payload is empty");
    return result;
  }

  DynamicFactory::MessagePtr message = CreateMessage(message_type);
  if (message == nullptr) {
    result.error = QStringLiteral("Unknown message type");
    return result;
  }
  if (!message->ParseFromString(bytes)) {
    result.error = QStringLiteral("Failed to parse message payload");
    return result;
  }

  google::protobuf::util::JsonPrintOptions options;
  options.add_whitespace = true;
  SetAlwaysPrintPrimitiveFields(&options);
  options.preserve_proto_field_names = true;

  std::string json;
  const auto status =
      google::protobuf::util::MessageToJsonString(*message, &json, options);
  if (!status.ok()) {
    result.text = QString::fromStdString(message->DebugString());
    result.ok = true;
    return result;
  }
  result.text = QString::fromStdString(json);
  result.ok = true;
  return result;
}

}  // namespace publish_panel
}  // namespace autoviz
