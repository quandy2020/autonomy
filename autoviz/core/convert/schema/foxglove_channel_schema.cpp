#include "autonomy/autoviz/core/convert/schema/foxglove_channel_schema.hpp"

#include <algorithm>
#include <unordered_set>

#include "autonomy/autoviz/core/convert/registry/automsgs_foxglove_registry.hpp"
#include "autonomy/autoviz/core/convert/schema/protobuf_ros_style_name.hpp"
#include "google/protobuf/descriptor.h"

namespace autoviz {
namespace converter {

bool SchemaConverter::AddFromDescriptor(const google::protobuf::Descriptor* descriptor) {
  return AddDescriptorSubtree(descriptor);
}

bool SchemaConverter::AddFromFullName(const std::string& proto_full_name,
                                      const google::protobuf::DescriptorPool* pool) {
  if (pool == nullptr) {
    return false;
  }
  const auto* descriptor = pool->FindMessageTypeByName(proto_full_name);
  if (descriptor == nullptr) {
    return false;
  }
  return AddFromDescriptor(descriptor);
}

const FoxgloveMessageSchema* SchemaConverter::FindSchema(const std::string& proto_full_name) const {
  const auto it = schemas_.find(proto_full_name);
  if (it == schemas_.end()) {
    return nullptr;
  }
  return &it->second;
}

std::vector<FoxgloveMessageSchema> SchemaConverter::ListSchemas() const {
  std::vector<FoxgloveMessageSchema> out;
  out.reserve(schemas_.size());
  for (const auto& [_, schema] : schemas_) {
    out.push_back(schema);
  }
  std::sort(
      out.begin(), out.end(),
      [](const FoxgloveMessageSchema& lhs, const FoxgloveMessageSchema& rhs) {
        return lhs.proto_full_name < rhs.proto_full_name;
      });
  return out;
}

bool SchemaConverter::AddDescriptorSubtree(const google::protobuf::Descriptor* descriptor) {
  if (descriptor == nullptr) {
    return false;
  }

  const std::string full_name = descriptor->full_name();
  if (schemas_.find(full_name) != schemas_.end()) {
    return true;
  }

  FoxgloveMessageSchema schema;
  schema.proto_full_name = full_name;
  schema.foxglove_type_name = RosStyleTypeName(full_name);
  schema.foxglove_canonical_name.clear();
  schema.foxglove_strategy = AutomsgsFoxgloveStrategy::kPassthroughProtobuf;
  schema.studio_visualization_hint.clear();
  schema.fields.reserve(descriptor->field_count());

  std::unordered_set<std::string> deps;
  for (int i = 0; i < descriptor->field_count(); ++i) {
    const auto* field = descriptor->field(i);
    if (field == nullptr) {
      continue;
    }

    const FoxgloveFieldSchema field_schema = BuildFieldSchema(*field);
    if (!field_schema.message_full_name.empty()) {
      deps.insert(field_schema.message_full_name);
      AddDescriptorSubtree(field->message_type());
    }

    schema.fields.push_back(field_schema);
  }

  schema.dependencies.assign(deps.begin(), deps.end());
  std::sort(schema.dependencies.begin(), schema.dependencies.end());
  schemas_[full_name] = std::move(schema);

  if (const AutomsgsFoxgloveEntry* reg = FindFoxgloveRule(full_name)) {
    auto& stored = schemas_[full_name];
    stored.foxglove_canonical_name = reg->foxglove_canonical_name;
    stored.foxglove_strategy = reg->strategy;
    stored.studio_visualization_hint = reg->studio_hint;
  }

  return true;
}

FoxgloveFieldSchema SchemaConverter::BuildFieldSchema(
    const google::protobuf::FieldDescriptor& field) const {
  FoxgloveFieldSchema out;
  out.name = field.name();
  out.type = MapValueType(field);
  out.is_repeated = field.is_repeated();
  out.is_map = field.is_map();
  if (field.cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE &&
      field.message_type() != nullptr) {
    out.message_full_name = field.message_type()->full_name();
  }
  if (field.cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_ENUM &&
      field.enum_type() != nullptr) {
    out.enum_full_name = field.enum_type()->full_name();
  }
  return out;
}

FoxgloveValueType SchemaConverter::MapValueType(const google::protobuf::FieldDescriptor& field) {
  using FD = google::protobuf::FieldDescriptor;
  switch (field.cpp_type()) {
    case FD::CPPTYPE_BOOL:
      return FoxgloveValueType::kBool;
    case FD::CPPTYPE_INT32:
      return FoxgloveValueType::kInt32;
    case FD::CPPTYPE_INT64:
      return FoxgloveValueType::kInt64;
    case FD::CPPTYPE_UINT32:
      return FoxgloveValueType::kUint32;
    case FD::CPPTYPE_UINT64:
      return FoxgloveValueType::kUint64;
    case FD::CPPTYPE_FLOAT:
      return FoxgloveValueType::kFloat32;
    case FD::CPPTYPE_DOUBLE:
      return FoxgloveValueType::kFloat64;
    case FD::CPPTYPE_STRING:
      return field.type() == FD::TYPE_BYTES ? FoxgloveValueType::kBytes
                                            : FoxgloveValueType::kString;
    case FD::CPPTYPE_ENUM:
      return FoxgloveValueType::kEnum;
    case FD::CPPTYPE_MESSAGE:
      return FoxgloveValueType::kMessage;
    default:
      return FoxgloveValueType::kUnknown;
  }
}

}  // namespace converter
}  // namespace autoviz
