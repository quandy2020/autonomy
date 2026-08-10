/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/table/table_field_extractor.hpp"

#include <automsgs/msgs/DynamicFactory.hh>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#include <sstream>

#include "autoviz/common/protobuf_qt_string.hpp"
#include "autoviz/commsgs/message_type_utils.hpp"
#include "autoviz/ui/plot/message_path_navigation.hpp"

namespace autoviz {
namespace table {
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

google::protobuf::Message* ParseMessage(const std::string& message_type,
                                        const std::string& payload,
                                        DynamicFactory::MessagePtr* out) {
  if (out == nullptr || message_type.empty() || payload.empty()) {
    return nullptr;
  }
  static DynamicFactory factory;
  const std::string normalized = NormalizeTypeName(message_type);
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

const google::protobuf::Descriptor* ResolveDescriptor(
    const std::string& message_type) {
  if (message_type.empty()) {
    return nullptr;
  }
  static DynamicFactory factory;
  const std::string normalized = NormalizeTypeName(message_type);
  DynamicFactory::MessagePtr message = factory.New(normalized);
  if (message == nullptr) {
    message = factory.New(StripPackagePrefix(normalized));
  }
  if (message == nullptr) {
    return nullptr;
  }
  return message->GetDescriptor();
}

bool IsScalarField(const google::protobuf::FieldDescriptor* field) {
  if (field == nullptr || field->is_repeated()) {
    return false;
  }
  switch (field->type()) {
    case google::protobuf::FieldDescriptor::TYPE_DOUBLE:
    case google::protobuf::FieldDescriptor::TYPE_FLOAT:
    case google::protobuf::FieldDescriptor::TYPE_INT64:
    case google::protobuf::FieldDescriptor::TYPE_UINT64:
    case google::protobuf::FieldDescriptor::TYPE_INT32:
    case google::protobuf::FieldDescriptor::TYPE_FIXED64:
    case google::protobuf::FieldDescriptor::TYPE_FIXED32:
    case google::protobuf::FieldDescriptor::TYPE_BOOL:
    case google::protobuf::FieldDescriptor::TYPE_STRING:
    case google::protobuf::FieldDescriptor::TYPE_BYTES:
    case google::protobuf::FieldDescriptor::TYPE_UINT32:
    case google::protobuf::FieldDescriptor::TYPE_SFIXED32:
    case google::protobuf::FieldDescriptor::TYPE_SFIXED64:
    case google::protobuf::FieldDescriptor::TYPE_SINT32:
    case google::protobuf::FieldDescriptor::TYPE_SINT64:
    case google::protobuf::FieldDescriptor::TYPE_ENUM:
      return true;
    default:
      return false;
  }
}

QString ScalarFieldToString(const google::protobuf::Message& message,
                            const google::protobuf::FieldDescriptor* field) {
  const google::protobuf::Reflection* reflection = message.GetReflection();
  switch (field->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
      return QString::number(reflection->GetDouble(message, field), 'g', 8);
    case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
      return QString::number(reflection->GetFloat(message, field), 'g', 6);
    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
      return QString::number(reflection->GetInt32(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
      return QString::number(reflection->GetInt64(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
      return QString::number(reflection->GetUInt32(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
      return QString::number(reflection->GetUInt64(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
      return reflection->GetBool(message, field) ? QStringLiteral("true")
                                                 : QStringLiteral("false");
    case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
      return QString::fromStdString(reflection->GetString(message, field));
    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM:
      if (const google::protobuf::EnumValueDescriptor* value =
              reflection->GetEnum(message, field)) {
        return ProtobufToQString(value->name());
      }
      return QString();
    default:
      return QString();
  }
}

QString RepeatedPrimitiveToString(
    const google::protobuf::Message& message,
    const google::protobuf::FieldDescriptor* field, int index) {
  const google::protobuf::Reflection* reflection = message.GetReflection();
  switch (field->cpp_type()) {
    case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
      return QString::number(reflection->GetRepeatedDouble(message, field, index),
                             'g', 8);
    case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
      return QString::number(reflection->GetRepeatedFloat(message, field, index),
                             'g', 6);
    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
      return QString::number(reflection->GetRepeatedInt32(message, field, index));
    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
      return QString::number(reflection->GetRepeatedInt64(message, field, index));
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
      return QString::number(reflection->GetRepeatedUInt32(message, field, index));
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
      return QString::number(reflection->GetRepeatedUInt64(message, field, index));
    case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
      return reflection->GetRepeatedBool(message, field, index)
                 ? QStringLiteral("true")
                 : QStringLiteral("false");
    case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
      return QString::fromStdString(
          reflection->GetRepeatedString(message, field, index));
    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM:
      if (const google::protobuf::EnumValueDescriptor* value =
              reflection->GetRepeatedEnum(message, field, index)) {
        return ProtobufToQString(value->name());
      }
      return QString();
    default:
      return QString();
  }
}

QString MessageFieldToString(const google::protobuf::Message& message,
                             const google::protobuf::FieldDescriptor* field) {
  if (field == nullptr) {
    return QString();
  }
  if (IsScalarField(field)) {
    return ScalarFieldToString(message, field);
  }
  if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    const google::protobuf::Message& sub =
        message.GetReflection()->GetMessage(message, field);
    QString text = QString::fromStdString(sub.ShortDebugString());
    text.replace(QLatin1Char('\n'), QLatin1Char(' '));
    return text.simplified();
  }
  return QString();
}

bool ResolveRepeatedField(const google::protobuf::Message& root,
                          const std::string& array_path,
                          const google::protobuf::Message** container,
                          const google::protobuf::FieldDescriptor** array_field) {
  if (array_path.empty() || container == nullptr || array_field == nullptr) {
    return false;
  }

  std::istringstream stream(array_path);
  std::string segment;
  std::vector<std::string> segments;
  while (std::getline(stream, segment, '.')) {
    if (!segment.empty()) {
      segments.push_back(segment);
    }
  }
  if (segments.empty()) {
    return false;
  }

  const google::protobuf::Message* current = &root;
  for (size_t i = 0; i + 1 < segments.size(); ++i) {
    const google::protobuf::FieldDescriptor* field =
        current->GetDescriptor()->FindFieldByName(segments[i]);
    if (field == nullptr || field->is_repeated() ||
        field->type() != google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      return false;
    }
    current = &current->GetReflection()->GetMessage(*current, field);
  }

  const google::protobuf::FieldDescriptor* repeated =
      current->GetDescriptor()->FindFieldByName(segments.back());
  if (repeated == nullptr || !repeated->is_repeated()) {
    return false;
  }
  *container = current;
  *array_field = repeated;
  return true;
}

std::optional<std::string> FindFirstRepeatedPathRecursive(
    const google::protobuf::Descriptor* desc, const std::string& prefix) {
  if (desc == nullptr) {
    return std::nullopt;
  }
  for (int i = 0; i < desc->field_count(); ++i) {
    const google::protobuf::FieldDescriptor* field = desc->field(i);
    if (field == nullptr) {
      continue;
    }
    const std::string field_name(field->name().data(), field->name().size());
    const std::string path =
        prefix.empty() ? field_name : prefix + '.' + field_name;
    if (field->is_repeated()) {
      return path;
    }
    if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      if (auto nested =
              FindFirstRepeatedPathRecursive(field->message_type(), path)) {
        return nested;
      }
    }
  }
  return std::nullopt;
}

TableData ExtractRepeatedMessageArray(
    const google::protobuf::Message& container,
    const google::protobuf::FieldDescriptor* array_field,
    const plot::ResolvedRepeatedField& context) {
  TableData data;
  if (array_field == nullptr || !array_field->is_repeated() ||
      array_field->type() != google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    return data;
  }

  const google::protobuf::Descriptor* element_desc = array_field->message_type();
  std::vector<const google::protobuf::FieldDescriptor*> column_fields;
  for (int i = 0; i < element_desc->field_count(); ++i) {
    const google::protobuf::FieldDescriptor* field = element_desc->field(i);
    if (field == nullptr || field->is_repeated()) {
      continue;
    }
    column_fields.push_back(field);
    data.columns.push_back(TableColumn{ProtobufToQString(field->name())});
  }

  const google::protobuf::Reflection* reflection = container.GetReflection();
  const int count = reflection->FieldSize(container, array_field);
  data.rows.reserve(static_cast<size_t>(count));
  for (int row_index = 0; row_index < count; ++row_index) {
    if (context.use_single_index && row_index != context.single_index) {
      continue;
    }
    const google::protobuf::Message& element =
        reflection->GetRepeatedMessage(container, array_field, row_index);
    if (context.element_filter.has_value() &&
        !plot::ElementMatchesPathFilter(element, *context.element_filter)) {
      continue;
    }
    std::vector<QString> row;
    row.reserve(column_fields.size());
    for (const google::protobuf::FieldDescriptor* field : column_fields) {
      row.push_back(MessageFieldToString(element, field));
    }
    data.rows.push_back(std::move(row));
  }
  return data;
}

TableData ExtractRepeatedPrimitiveArray(
    const google::protobuf::Message& container,
    const google::protobuf::FieldDescriptor* array_field,
    const plot::ResolvedRepeatedField& context) {
  TableData data;
  if (array_field == nullptr || !array_field->is_repeated() ||
      array_field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    return data;
  }

  data.columns.push_back(TableColumn{ProtobufToQString(array_field->name())});
  const google::protobuf::Reflection* reflection = container.GetReflection();
  const int count = reflection->FieldSize(container, array_field);
  data.rows.reserve(static_cast<size_t>(count));
  for (int row_index = 0; row_index < count; ++row_index) {
    if (context.use_single_index && row_index != context.single_index) {
      continue;
    }
    if (context.element_filter.has_value()) {
      continue;
    }
    data.rows.push_back(
        {RepeatedPrimitiveToString(container, array_field, row_index)});
  }
  return data;
}

}  // namespace

TableFieldExtractor& TableFieldExtractor::instance() {
  static TableFieldExtractor extractor;
  return extractor;
}

std::optional<std::string> TableFieldExtractor::findFirstArrayPath(
    const std::string& message_type) const {
  const google::protobuf::Descriptor* desc = ResolveDescriptor(message_type);
  if (desc == nullptr) {
    return std::nullopt;
  }
  return FindFirstRepeatedPathRecursive(desc, "");
}

std::optional<TableData> TableFieldExtractor::extract(
    const std::string& message_type, const std::string& payload,
    const std::string& array_path) const {
  DynamicFactory::MessagePtr message;
  if (ParseMessage(message_type, payload, &message) == nullptr) {
    return std::nullopt;
  }

  std::string resolved_path = array_path;
  if (resolved_path.empty()) {
    if (auto detected = findFirstArrayPath(message_type)) {
      resolved_path = *detected;
    } else {
      return std::nullopt;
    }
  }

  const google::protobuf::Message* container = nullptr;
  const google::protobuf::FieldDescriptor* array_field = nullptr;
  plot::ResolvedRepeatedField resolved;
  if (plot::ResolveRepeatedFieldPath(*message, resolved_path, &resolved)) {
    container = resolved.container;
    array_field = resolved.repeated_field;
  } else if (!ResolveRepeatedField(*message, resolved_path, &container,
                                   &array_field)) {
    return std::nullopt;
  } else {
    resolved.container = container;
    resolved.repeated_field = array_field;
  }

  TableData data;
  if (array_field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
    data = ExtractRepeatedMessageArray(*container, array_field, resolved);
  } else {
    data = ExtractRepeatedPrimitiveArray(*container, array_field, resolved);
  }
  if (data.columns.empty()) {
    return std::nullopt;
  }
  return data;
}

}  // namespace table
}  // namespace autoviz
