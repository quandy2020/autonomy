/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/integration/grpc/grpc_json_codec.hpp"

#include <google/protobuf/util/json_util.h>

#include "autoviz/common/protobuf_json_compat.hpp"

namespace autoviz {
namespace integration {
namespace grpc_client {
namespace {

void SetError(std::string* error, const std::string& message) {
  if (error != nullptr) {
    *error = message;
  }
}

}  // namespace

std::unique_ptr<google::protobuf::Message> JsonToDynamicMessage(
    const std::string& json, const google::protobuf::Descriptor* descriptor,
    google::protobuf::DynamicMessageFactory& factory, std::string* error) {
  if (descriptor == nullptr) {
    SetError(error, "Descriptor is null");
    return nullptr;
  }

  const google::protobuf::Message* prototype = factory.GetPrototype(descriptor);
  if (prototype == nullptr) {
    SetError(error, "Failed to get DynamicMessage prototype");
    return nullptr;
  }
  std::unique_ptr<google::protobuf::Message> message(prototype->New());
  if (message == nullptr) {
    SetError(error, "Failed to create DynamicMessage");
    return nullptr;
  }

  google::protobuf::util::JsonParseOptions options;
  options.ignore_unknown_fields = true;
  const auto status =
      google::protobuf::util::JsonStringToMessage(json, message.get(), options);
  if (!status.ok()) {
    SetError(error, StatusMessageToStdString(status));
    return nullptr;
  }
  return message;
}

std::unique_ptr<google::protobuf::Message> JsonToDynamicMessage(
    const std::string& json, const google::protobuf::Descriptor* descriptor,
    std::string* error) {
  // Factory must outlive messages created from its prototypes (TypeInfo).
  static google::protobuf::DynamicMessageFactory factory;
  return JsonToDynamicMessage(json, descriptor, factory, error);
}

bool DynamicMessageToJson(const google::protobuf::Message& message,
                          bool include_defaults, std::string* json_out,
                          std::string* error) {
  if (json_out == nullptr) {
    SetError(error, "json_out is null");
    return false;
  }

  google::protobuf::util::JsonPrintOptions options;
  options.preserve_proto_field_names = true;
  if (include_defaults) {
    SetAlwaysPrintPrimitiveFields(&options);
  }

  std::string json;
  const auto status =
      google::protobuf::util::MessageToJsonString(message, &json, options);
  if (!status.ok()) {
    SetError(error, StatusMessageToStdString(status));
    return false;
  }
  *json_out = std::move(json);
  return true;
}

}  // namespace grpc_client
}  // namespace integration
}  // namespace autoviz
