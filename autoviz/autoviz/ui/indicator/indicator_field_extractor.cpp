/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/indicator/indicator_field_extractor.hpp"

#include <automsgs/msgs/DynamicFactory.hh>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#include <cmath>
#include <sstream>

#include "autoviz/commsgs/message_type_utils.hpp"

namespace autoviz {
namespace indicator {
namespace {

using automsgs::msgs::DynamicFactory;

google::protobuf::Message* ParseMessage(const std::string& message_type,
                                        const std::string& payload,
                                        DynamicFactory::MessagePtr* out) {
  if (out == nullptr || message_type.empty() || payload.empty()) {
    return nullptr;
  }
  static DynamicFactory factory;
  const std::string normalized = commsgs::NormalizeMessageType(message_type);
  *out = factory.New(normalized);
  if (*out == nullptr) {
    static const char* kPrefix = "automsgs.msgs.";
    if (normalized.rfind(kPrefix, 0) == 0) {
      *out = factory.New(normalized.substr(std::char_traits<char>::length(kPrefix)));
    }
  }
  if (*out == nullptr || !(*out)->ParseFromString(payload)) {
    out->reset();
    return nullptr;
  }
  return out->get();
}

bool ResolveLeafField(const google::protobuf::Message& message,
                      const std::string& field_path,
                      const google::protobuf::Message** container,
                      const google::protobuf::FieldDescriptor** leaf_field) {
  if (field_path.empty() || container == nullptr || leaf_field == nullptr) {
    return false;
  }
  std::istringstream stream(field_path);
  std::string segment;
  const google::protobuf::Message* current = &message;

  while (std::getline(stream, segment, '.')) {
    if (segment.empty()) {
      return false;
    }
    const google::protobuf::Descriptor* desc = current->GetDescriptor();
    const google::protobuf::FieldDescriptor* field = desc->FindFieldByName(segment);
    if (field == nullptr || field->is_repeated()) {
      return false;
    }
    if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      current = &current->GetReflection()->GetMessage(*current, field);
      continue;
    }
    *container = current;
    *leaf_field = field;
    return true;
  }
  return false;
}

std::optional<IndicatorFieldValue> ReadScalarField(
    const google::protobuf::Message& message,
    const google::protobuf::FieldDescriptor* field) {
  if (field == nullptr) {
    return std::nullopt;
  }
  const google::protobuf::Reflection* reflection = message.GetReflection();
  IndicatorFieldValue value;

  switch (field->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
      value.kind = IndicatorValueKind::kNumeric;
      value.number = reflection->GetDouble(message, field);
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
      value.kind = IndicatorValueKind::kNumeric;
      value.number = reflection->GetFloat(message, field);
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
      value.kind = IndicatorValueKind::kNumeric;
      value.number = reflection->GetInt32(message, field);
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
      value.kind = IndicatorValueKind::kNumeric;
      value.number = static_cast<double>(reflection->GetInt64(message, field));
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
      value.kind = IndicatorValueKind::kNumeric;
      value.number = reflection->GetUInt32(message, field);
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
      value.kind = IndicatorValueKind::kNumeric;
      value.number = static_cast<double>(reflection->GetUInt64(message, field));
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
      value.kind = IndicatorValueKind::kBoolean;
      value.boolean = reflection->GetBool(message, field);
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
      value.kind = IndicatorValueKind::kString;
      value.text = QString::fromStdString(reflection->GetString(message, field));
      break;
    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM: {
      const int enum_value = reflection->GetEnumValue(message, field);
      value.kind = IndicatorValueKind::kNumeric;
      value.number = enum_value;
      if (const google::protobuf::EnumValueDescriptor* enum_desc =
              field->enum_type()->FindValueByNumber(enum_value)) {
        value.text = QString::fromStdString(enum_desc->name());
      }
      break;
    }
    default:
      return std::nullopt;
  }

  if (value.kind == IndicatorValueKind::kNumeric && !std::isfinite(value.number)) {
    return std::nullopt;
  }
  return value;
}

}  // namespace

IndicatorFieldExtractor& IndicatorFieldExtractor::instance() {
  static IndicatorFieldExtractor extractor;
  return extractor;
}

std::optional<IndicatorFieldValue> IndicatorFieldExtractor::extract(
    const std::string& message_type, const std::string& payload,
    const std::string& field_path) const {
  DynamicFactory::MessagePtr message;
  if (ParseMessage(message_type, payload, &message) == nullptr) {
    return std::nullopt;
  }
  const google::protobuf::Message* container = nullptr;
  const google::protobuf::FieldDescriptor* leaf_field = nullptr;
  if (!ResolveLeafField(*message, field_path, &container, &leaf_field)) {
    return std::nullopt;
  }
  return ReadScalarField(*container, leaf_field);
}

QString FormatIndicatorFieldValue(const IndicatorFieldValue& value) {
  switch (value.kind) {
    case IndicatorValueKind::kNumeric:
      return QString::number(value.number, 'g', 8);
    case IndicatorValueKind::kString:
      return value.text;
    case IndicatorValueKind::kBoolean:
      return value.boolean ? QStringLiteral("true") : QStringLiteral("false");
  }
  return {};
}

}  // namespace indicator
}  // namespace autoviz
