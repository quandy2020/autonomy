/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/plot_field_extractor.hpp"

#include <automsgs/msgs/DynamicFactory.hh>

#include "autoviz/ui/plot/message_path_navigation.hpp"

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#include <cmath>
#include <sstream>

#include "autoviz/commsgs/message_type_utils.hpp"

namespace autoviz {
namespace plot {
namespace {

using automsgs::msgs::DynamicFactory;

std::string NormalizeTypeName(const std::string& message_type) {
  return commsgs::NormalizeMessageType(message_type);
}

std::string StripPackagePrefix(const std::string& type_name) {
  static const char* kPrefix = "automsgs.msgs.";
  if (type_name.rfind(kPrefix, 0) == 0) {
    return type_name.substr(std::char_traits<char>::length(kPrefix));
  }
  return type_name;
}

google::protobuf::Message* ParseMessageWithGeneratedPool(
    const std::string& normalized, const std::string& payload,
    DynamicFactory::MessagePtr* out) {
  if (out == nullptr || normalized.empty() || payload.empty()) {
    return nullptr;
  }
  const google::protobuf::DescriptorPool* pool =
      google::protobuf::DescriptorPool::generated_pool();
  if (pool == nullptr) {
    return nullptr;
  }
  const google::protobuf::Descriptor* desc = pool->FindMessageTypeByName(normalized);
  if (desc == nullptr) {
    desc = pool->FindMessageTypeByName(StripPackagePrefix(normalized));
  }
  if (desc == nullptr) {
    return nullptr;
  }
  const google::protobuf::Message* prototype =
      google::protobuf::MessageFactory::generated_factory()->GetPrototype(desc);
  if (prototype == nullptr) {
    return nullptr;
  }
  *out = DynamicFactory::MessagePtr(prototype->New());
  if (*out == nullptr || !(*out)->ParseFromString(payload)) {
    out->reset();
    return nullptr;
  }
  return out->get();
}

google::protobuf::Message* ParseMessage(const std::string& message_type,
                                        const std::string& payload,
                                        DynamicFactory::MessagePtr* out) {
  if (out == nullptr || message_type.empty() || payload.empty()) {
    return nullptr;
  }
  const std::string normalized = NormalizeTypeName(message_type);
  if (google::protobuf::Message* parsed =
          ParseMessageWithGeneratedPool(normalized, payload, out)) {
    return parsed;
  }

  static DynamicFactory factory;
  *out = factory.New(normalized);
  if (*out == nullptr) {
    *out = factory.New(StripPackagePrefix(normalized));
  }
  if (*out == nullptr || !(*out)->ParseFromString(payload)) {
    out->reset();
    return nullptr;
  }
  return out->get();
}

double ReadNumericField(const google::protobuf::Message& message,
                        const google::protobuf::FieldDescriptor* field) {
  const google::protobuf::Reflection* reflection = message.GetReflection();
  switch (field->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
      return reflection->GetDouble(message, field);
    case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
      return reflection->GetFloat(message, field);
    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
      return reflection->GetInt32(message, field);
    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
      return static_cast<double>(reflection->GetInt64(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
      return reflection->GetUInt32(message, field);
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
      return static_cast<double>(reflection->GetUInt64(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
      return reflection->GetBool(message, field) ? 1.0 : 0.0;
    default:
      return 0.0;
  }
}

bool ExtractTimestampFromMessage(const google::protobuf::Message& message,
                                 double* timestamp_sec) {
  const google::protobuf::Descriptor* desc = message.GetDescriptor();
  const google::protobuf::FieldDescriptor* header_field =
      desc->FindFieldByName("header");
  if (header_field == nullptr ||
      header_field->type() !=
          google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    return false;
  }
  const google::protobuf::Message& header =
      message.GetReflection()->GetMessage(message, header_field);
  const google::protobuf::Descriptor* header_desc = header.GetDescriptor();
  const google::protobuf::FieldDescriptor* stamp_field =
      header_desc->FindFieldByName("stamp");
  if (stamp_field == nullptr ||
      stamp_field->type() !=
          google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    return false;
  }
  const google::protobuf::Message& stamp =
      header.GetReflection()->GetMessage(header, stamp_field);
  const google::protobuf::Descriptor* stamp_desc = stamp.GetDescriptor();
  const google::protobuf::FieldDescriptor* sec_field =
      stamp_desc->FindFieldByName("sec");
  const google::protobuf::FieldDescriptor* nanosec_field =
      stamp_desc->FindFieldByName("nanosec");
  if (sec_field == nullptr) {
    return false;
  }
  const double sec = ReadNumericField(stamp, sec_field);
  double nanosec = 0.0;
  if (nanosec_field != nullptr) {
    nanosec = ReadNumericField(stamp, nanosec_field);
  }
  *timestamp_sec = sec + nanosec * 1e-9;
  return std::isfinite(*timestamp_sec);
}

bool ExtractNumericByPath(const google::protobuf::Message& message,
                          const std::string& field_path, double* value) {
  if (field_path.empty()) {
    return false;
  }
  std::istringstream stream(field_path);
  std::string segment;
  const google::protobuf::Message* current = &message;
  const google::protobuf::FieldDescriptor* leaf_field = nullptr;

  while (std::getline(stream, segment, '.')) {
    if (segment.empty()) {
      return false;
    }
    const google::protobuf::Descriptor* desc = current->GetDescriptor();
    const google::protobuf::FieldDescriptor* field =
        desc->FindFieldByName(segment);
    if (field == nullptr) {
      return false;
    }
    if (field->is_repeated()) {
      if (field->type() != google::protobuf::FieldDescriptor::TYPE_MESSAGE ||
          current->GetReflection()->FieldSize(*current, field) <= 0) {
        return false;
      }
      current = &current->GetReflection()->GetRepeatedMessage(*current, field, 0);
      continue;
    }
    if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      current = &current->GetReflection()->GetMessage(*current, field);
      continue;
    }
    leaf_field = field;
    break;
  }

  if (leaf_field == nullptr) {
    return false;
  }
  *value = ReadNumericField(*current, leaf_field);
  return std::isfinite(*value);
}

const google::protobuf::Message* ResolveMessageAtPath(
    const google::protobuf::Message& message, const std::string& field_path) {
  if (field_path.empty()) {
    return &message;
  }
  std::istringstream stream(field_path);
  std::string segment;
  const google::protobuf::Message* current = &message;
  while (std::getline(stream, segment, '.')) {
    if (segment.empty()) {
      return nullptr;
    }
    const google::protobuf::Descriptor* desc = current->GetDescriptor();
    const google::protobuf::FieldDescriptor* field =
        desc->FindFieldByName(segment);
    if (field == nullptr || field->is_repeated()) {
      return nullptr;
    }
    if (field->type() != google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      return current;
    }
    current = &current->GetReflection()->GetMessage(*current, field);
  }
  return current;
}

bool ExtractTimestampAtPath(const google::protobuf::Message& message,
                            const std::string& timestamp_path,
                            double* timestamp_sec) {
  if (timestamp_path.empty()) {
    return false;
  }
  const google::protobuf::Message* resolved =
      ResolveMessageAtPath(message, timestamp_path);
  if (resolved == nullptr) {
    return false;
  }
  if (ExtractTimestampFromMessage(*resolved, timestamp_sec)) {
    return true;
  }
  return ExtractNumericByPath(message, timestamp_path, timestamp_sec);
}

}  // namespace

PlotFieldExtractor& PlotFieldExtractor::instance() {
  static PlotFieldExtractor extractor;
  return extractor;
}

std::optional<double> PlotFieldExtractor::extractNumeric(
    const std::string& message_type, const std::string& payload,
    const std::string& field_path) const {
  DynamicFactory::MessagePtr message;
  if (ParseMessage(message_type, payload, &message) == nullptr) {
    return std::nullopt;
  }
  double value = 0.0;
  if (!ExtractNumericByMessagePath(*message, field_path, &value) &&
      !ExtractNumericByPath(*message, field_path, &value)) {
    return std::nullopt;
  }
  return value;
}

std::optional<double> PlotFieldExtractor::extractTimestamp(
    const std::string& message_type, const std::string& payload,
    const std::string& timestamp_path, double fallback_timestamp_sec) const {
  DynamicFactory::MessagePtr message;
  if (ParseMessage(message_type, payload, &message) == nullptr) {
    return std::nullopt;
  }
  double timestamp_sec = fallback_timestamp_sec;
  if (timestamp_path.empty()) {
    if (!ExtractTimestampFromMessage(*message, &timestamp_sec)) {
      timestamp_sec = fallback_timestamp_sec;
    }
  } else if (!ExtractTimestampAtPath(*message, timestamp_path, &timestamp_sec)) {
    return std::nullopt;
  }
  return timestamp_sec;
}

std::optional<PlotSample> PlotFieldExtractor::extract(
    const std::string& message_type, const std::string& payload,
    const std::string& field_path, double fallback_timestamp_sec) const {
  if (message_type.empty() || payload.empty() || field_path.empty()) {
    return std::nullopt;
  }

  DynamicFactory::MessagePtr message;
  if (ParseMessage(message_type, payload, &message) == nullptr) {
    return std::nullopt;
  }

  double value = 0.0;
  if (!ExtractNumericByMessagePath(*message, field_path, &value) &&
      !ExtractNumericByPath(*message, field_path, &value)) {
    return std::nullopt;
  }

  PlotSample sample;
  sample.value = value;
  if (!ExtractTimestampFromMessage(*message, &sample.timestamp_sec)) {
    sample.timestamp_sec = fallback_timestamp_sec;
  }
  return sample;
}

}  // namespace plot
}  // namespace autoviz
