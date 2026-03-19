// Generic protobuf descriptor -> foxglove schema converter framework.
#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace google {
namespace protobuf {
class Descriptor;
class DescriptorPool;
class FieldDescriptor;
}  // namespace protobuf
}  // namespace google

namespace autoviz {
namespace converter {

enum class FoxgloveValueType {
  kUnknown = 0,
  kBool,
  kInt32,
  kInt64,
  kUint32,
  kUint64,
  kFloat32,
  kFloat64,
  kString,
  kBytes,
  kMessage,
  kEnum,
};

struct FoxgloveFieldSchema {
  std::string name;
  FoxgloveValueType type{FoxgloveValueType::kUnknown};
  bool is_repeated{false};
  bool is_map{false};
  std::string message_full_name;
  std::string enum_full_name;
};

// Internal, transport-agnostic schema model.
// Later this can be translated to concrete foxglove-sdk channel/schema objects.
struct FoxgloveMessageSchema {
  std::string proto_full_name;
  std::string foxglove_type_name;
  std::vector<FoxgloveFieldSchema> fields;
  std::vector<std::string> dependencies;
};

class SchemaConverter {
 public:
  SchemaConverter() = default;

  // Register one message and all nested dependencies recursively.
  // Returns false when descriptor is null or conversion fails.
  bool RegisterMessage(const google::protobuf::Descriptor* descriptor);

  // Register from message full name using descriptor pool lookup.
  bool RegisterMessageByName(const std::string& proto_full_name,
                             const google::protobuf::DescriptorPool* pool);

  // Query schema by protobuf full name.
  const FoxgloveMessageSchema* FindByProtoFullName(
      const std::string& proto_full_name) const;

  // Export all currently registered schemas.
  std::vector<FoxgloveMessageSchema> ExportAll() const;

 private:
  bool ConvertMessageRecursive(const google::protobuf::Descriptor* descriptor);
  FoxgloveFieldSchema ConvertField(const google::protobuf::FieldDescriptor& field) const;
  static FoxgloveValueType ConvertFieldType(
      const google::protobuf::FieldDescriptor& field);

 private:
  std::unordered_map<std::string, FoxgloveMessageSchema> schemas_;
};

}  // namespace converter
}  // namespace autoviz

