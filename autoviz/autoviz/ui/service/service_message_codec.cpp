/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/service/service_message_codec.hpp"

#include <automsgs/msgs/DynamicFactory.hh>

#include <google/protobuf/util/json_util.h>

#include "autolink/message/protobuf_factory.hpp"
#include "autoviz/common/protobuf_json_compat.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"

namespace autoviz {
namespace service_panel {
namespace {

using automsgs::msgs::DynamicFactory;

std::string StripPackagePrefix(const std::string& type_name) {
  static const char* kPrefix = "automsgs.msgs.";
  if (type_name.rfind(kPrefix, 0) == 0) {
    return type_name.substr(std::char_traits<char>::length(kPrefix));
  }
  return type_name;
}

DynamicFactory::MessagePtr CreateAutomsgMessage(const std::string& message_type) {
  static DynamicFactory factory;
  const std::string normalized = commsgs::NormalizeMessageType(message_type);
  DynamicFactory::MessagePtr message = factory.New(normalized);
  if (message == nullptr) {
    message = factory.New(StripPackagePrefix(normalized));
  }
  return message;
}

std::unique_ptr<google::protobuf::Message> CreateProtobufMessage(
    const std::string& message_type) {
  auto* factory = autolink::message::ProtobufFactory::Instance();
  return std::unique_ptr<google::protobuf::Message>(
      factory->GenerateMessageByType(message_type));
}

}  // namespace

ServiceMessageCodec& ServiceMessageCodec::instance() {
  static ServiceMessageCodec codec;
  return codec;
}

std::optional<QString> ServiceMessageCodec::defaultJsonTemplate(
    const std::string& message_type) const {
  if (message_type.empty()) {
    return std::nullopt;
  }

  google::protobuf::util::JsonPrintOptions options;
  options.add_whitespace = true;
  SetAlwaysPrintPrimitiveFields(&options);
  options.preserve_proto_field_names = true;

  if (DynamicFactory::MessagePtr message = CreateAutomsgMessage(message_type)) {
    std::string json;
    const auto status =
        google::protobuf::util::MessageToJsonString(*message, &json, options);
    if (status.ok()) {
      return QString::fromStdString(json);
    }
  }

  if (std::unique_ptr<google::protobuf::Message> message =
          CreateProtobufMessage(message_type)) {
    std::string json;
    const auto status =
        google::protobuf::util::MessageToJsonString(*message, &json, options);
    if (status.ok()) {
      return QString::fromStdString(json);
    }
  }

  return std::nullopt;
}

CodecResult ServiceMessageCodec::encodeJson(const std::string& message_type,
                                            const QString& message_json) const {
  CodecResult result;
  if (message_type.empty()) {
    result.error = QStringLiteral("Message type is required");
    return result;
  }
  const QString trimmed = message_json.trimmed();
  if (trimmed.isEmpty()) {
    result.error = QStringLiteral("Request JSON is empty");
    return result;
  }

  google::protobuf::util::JsonParseOptions options;
  options.ignore_unknown_fields = true;

  if (DynamicFactory::MessagePtr message = CreateAutomsgMessage(message_type)) {
    const auto status = google::protobuf::util::JsonStringToMessage(
        trimmed.toStdString(), message.get(), options);
    if (status.ok() && message->SerializeToString(&result.payload)) {
      result.ok = true;
      return result;
    }
    if (!status.ok()) {
      result.error = StatusMessageToQString(status);
      return result;
    }
  }

  if (std::unique_ptr<google::protobuf::Message> message =
          CreateProtobufMessage(message_type)) {
    const auto status = google::protobuf::util::JsonStringToMessage(
        trimmed.toStdString(), message.get(), options);
    if (!status.ok()) {
      result.error = StatusMessageToQString(status);
      return result;
    }
    if (message->SerializeToString(&result.payload)) {
      result.ok = true;
      return result;
    }
  }

  result.error = QStringLiteral("Unknown message type");
  return result;
}

CodecResult ServiceMessageCodec::decodeToJson(
    const std::string& message_type, const std::string& bytes) const {
  CodecResult result;
  if (message_type.empty()) {
    result.error = QStringLiteral("Response type is required");
    return result;
  }

  google::protobuf::util::JsonPrintOptions options;
  options.add_whitespace = true;
  SetAlwaysPrintPrimitiveFields(&options);
  options.preserve_proto_field_names = true;

  if (DynamicFactory::MessagePtr message = CreateAutomsgMessage(message_type)) {
    if (message->ParseFromString(bytes)) {
      std::string json;
      const auto status =
          google::protobuf::util::MessageToJsonString(*message, &json, options);
      if (status.ok()) {
        result.text = QString::fromStdString(json);
        result.ok = true;
        return result;
      }
      result.text = QString::fromStdString(message->DebugString());
      result.ok = true;
      return result;
    }
  }

  if (std::unique_ptr<google::protobuf::Message> message =
          CreateProtobufMessage(message_type)) {
    if (message->ParseFromString(bytes)) {
      std::string json;
      const auto status =
          google::protobuf::util::MessageToJsonString(*message, &json, options);
      if (status.ok()) {
        result.text = QString::fromStdString(json);
        result.ok = true;
        return result;
      }
      result.text = QString::fromStdString(message->DebugString());
      result.ok = true;
      return result;
    }
  }

  result.text = QStringLiteral("(%1 bytes binary response)")
                    .arg(static_cast<qulonglong>(bytes.size()));
  result.ok = true;
  return result;
}

}  // namespace service_panel
}  // namespace autoviz
