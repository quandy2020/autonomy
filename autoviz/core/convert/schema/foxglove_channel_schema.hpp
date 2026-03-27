// 从 protobuf DescriptorPool 生成 Foxglove 通道用的 `FoxgloveMessageSchema`，并合并 registry 策略。
#pragma once

#include "autonomy/autoviz/core/convert/registry/automsgs_foxglove_registry.hpp"

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

struct FoxgloveMessageSchema {
  std::string proto_full_name;
  /// 由 `RosStyleTypeName` 得到的人类可读名（如 sensor_msgs/Imu）。
  std::string foxglove_type_name;
  std::string foxglove_canonical_name;
  AutomsgsFoxgloveStrategy foxglove_strategy{AutomsgsFoxgloveStrategy::kPassthroughProtobuf};
  std::string studio_visualization_hint;
  std::vector<FoxgloveFieldSchema> fields;
  std::vector<std::string> dependencies;
};

class SchemaConverter {
 public:
  SchemaConverter() = default;

  bool AddFromDescriptor(const google::protobuf::Descriptor* descriptor);
  bool AddFromFullName(const std::string& proto_full_name,
                       const google::protobuf::DescriptorPool* pool);

  const FoxgloveMessageSchema* FindSchema(const std::string& proto_full_name) const;

  std::vector<FoxgloveMessageSchema> ListSchemas() const;

 private:
  bool AddDescriptorSubtree(const google::protobuf::Descriptor* descriptor);
  FoxgloveFieldSchema BuildFieldSchema(const google::protobuf::FieldDescriptor& field) const;
  static FoxgloveValueType MapValueType(const google::protobuf::FieldDescriptor& field);

 private:
  std::unordered_map<std::string, FoxgloveMessageSchema> schemas_;
};

}  // namespace converter
}  // namespace autoviz
