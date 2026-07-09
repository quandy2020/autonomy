/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "autonomy/visualization/schema/schema_compatibility.hpp"

#include <array>
#include <unordered_set>
#include <utility>

#include <google/protobuf/descriptor.h>

#include "autonomy/visualization/common/visualization_schema_registry.hpp"

namespace autonomy {
namespace visualization {
namespace {

using Mapping = std::pair<const char*, const char*>;
constexpr std::array<Mapping, 14> kPackageMappings = {{
    {"autonomy.commsgs.proto.sensor_msgs", "sensor_msgs"},
    {"autonomy.commsgs.proto.sensor_msgs_foxglove", "sensor_msgs"},
    {"autonomy.commsgs.proto.std_msgs", "std_msgs"},
    {"autonomy.commsgs.proto.geometry_msgs", "geometry_msgs"},
    {"autonomy.commsgs.proto.visualization_msgs", "visualization_msgs"},
    {"autonomy.commsgs.proto.builtin_interfaces", "builtin_interfaces"},
    {"autonomy.commsgs.proto.planning_msgs", "nav_msgs"},
    {"autonomy.commsgs.proto.nav_msgs", "nav_msgs"},
    {"autonomy.commsgs.proto.map_msgs", "map_msgs"},
    {"autonomy.commsgs.proto.vision_msgs", "vision_msgs"},
    {"autonomy.commsgs.proto.diagnostic_msgs", "diagnostic_msgs"},
    {"autonomy.commsgs.proto.tf2_msgs", "tf2_msgs"},
    {"autonomy.commsgs.proto.vehicle_msgs", "vehicle_msgs"},
    {"autonomy.commsgs.proto.error_code", "error_code"},
}};

std::string ReplacePrefix(const std::string& value, const Mapping& mapping) {
  const std::string from = mapping.first;
  const std::string to = mapping.second;
  if (value == from) {
    return to;
  }
  if (value.rfind(from + ".", 0) == 0) {
    return to + value.substr(from.size());
  }
  if (value.rfind("." + from + ".", 0) == 0) {
    return "." + to + value.substr(from.size() + 1);
  }
  if (value.rfind("." + from, 0) == 0 && value.size() == from.size() + 1) {
    return "." + to;
  }
  return value;
}

std::string NormalizeTypeReferenceValue(const std::string& type_name) {
  std::string normalized = type_name;
  for (const auto& mapping : kPackageMappings) {
    normalized = ReplacePrefix(normalized, mapping);
  }
  return normalized;
}

void AppendGeneratedFileDescriptor(
    const google::protobuf::FileDescriptor* file_descriptor,
    google::protobuf::FileDescriptorSet* descriptor_set,
    std::unordered_set<std::string>* seen_files) {
  if (file_descriptor == nullptr || descriptor_set == nullptr ||
      seen_files == nullptr) {
    return;
  }

  if (!seen_files->insert(file_descriptor->name()).second) {
    return;
  }

  for (int i = 0; i < file_descriptor->dependency_count(); ++i) {
    AppendGeneratedFileDescriptor(file_descriptor->dependency(i), descriptor_set,
                                  seen_files);
  }

  file_descriptor->CopyTo(descriptor_set->add_file());
}

void NormalizeDescriptorMessage(google::protobuf::DescriptorProto* message) {
  if (message == nullptr) {
    return;
  }
  for (int i = 0; i < message->field_size(); ++i) {
    auto* field = message->mutable_field(i);
    field->set_type_name(NormalizeTypeReferenceValue(field->type_name()));
  }
  for (int i = 0; i < message->nested_type_size(); ++i) {
    NormalizeDescriptorMessage(message->mutable_nested_type(i));
  }
}

}  // namespace

bool SchemaCompatibility::IsDirectlyRenderableIn3D(
    const std::string& message_type) {
  return VisualizationSchemaRegistry::IsThreeDRenderable(message_type);
}

bool SchemaCompatibility::RequiresPayloadAdaptation(
    const std::string& message_type) {
  return VisualizationSchemaRegistry::RequiresPayloadAdaptation(message_type);
}

bool SchemaCompatibility::IsBridgeable(const std::string& message_type) {
  return VisualizationSchemaRegistry::IsBridgeable(message_type);
}

bool SchemaCompatibility::IsSupportedBy3D(const std::string& message_type) {
  return IsDirectlyRenderableIn3D(message_type) ||
         RequiresPayloadAdaptation(message_type);
}

void SchemaCompatibility::NormalizeFileDescriptorProto(
    google::protobuf::FileDescriptorProto* file_proto) {
  if (file_proto == nullptr) {
    return;
  }

  file_proto->set_name(NormalizeFileName(file_proto->name()));
  file_proto->set_package(NormalizePackageName(file_proto->package()));
  for (int i = 0; i < file_proto->dependency_size(); ++i) {
    file_proto->set_dependency(i, NormalizeFileName(file_proto->dependency(i)));
  }
  for (int i = 0; i < file_proto->message_type_size(); ++i) {
    NormalizeDescriptorMessage(file_proto->mutable_message_type(i));
  }
}

void SchemaCompatibility::NormalizeFileDescriptorSet(
    google::protobuf::FileDescriptorSet* file_set) {
  if (file_set == nullptr) {
    return;
  }
  for (int i = 0; i < file_set->file_size(); ++i) {
    NormalizeFileDescriptorProto(file_set->mutable_file(i));
  }
}

std::string SchemaCompatibility::NormalizeSchemaName(
    const std::string& message_type) {
  if (VisualizationSchemaRegistry::IsBridgeable(message_type)) {
    return VisualizationSchemaRegistry::ResolveTargetSchemaName(message_type);
  }

  std::string normalized = message_type;
  for (const auto& mapping : kPackageMappings) {
    normalized = ReplacePrefix(normalized, mapping);
  }
  return normalized;
}

std::string SchemaCompatibility::NormalizeTypeReference(
    const std::string& type_name) {
  return NormalizeTypeReferenceValue(type_name);
}

std::string SchemaCompatibility::NormalizePackageName(
    const std::string& package_name) {
  return NormalizeSchemaName(package_name);
}

std::string SchemaCompatibility::NormalizeFileName(const std::string& file_name) {
  std::string normalized = file_name;
  constexpr std::array<Mapping, 14> kFileMappings = {{
      {"autonomy/commsgs/proto/sensor_msgs.proto", "sensor_msgs.proto"},
      {"autonomy/commsgs/proto/sensor_msgs_foxglove.proto",
       "sensor_msgs_foxglove.proto"},
      {"autonomy/commsgs/proto/std_msgs.proto", "std_msgs.proto"},
      {"autonomy/commsgs/proto/geometry_msgs.proto", "geometry_msgs.proto"},
      {"autonomy/commsgs/proto/visualization_msgs.proto", "visualization_msgs.proto"},
      {"autonomy/commsgs/proto/builtin_interfaces.proto", "builtin_interfaces.proto"},
      {"autonomy/commsgs/proto/planning_msgs.proto", "nav_msgs.proto"},
      {"autonomy/commsgs/proto/nav_msgs.proto", "nav_msgs.proto"},
      {"autonomy/commsgs/proto/map_msgs.proto", "map_msgs.proto"},
      {"autonomy/commsgs/proto/vision_msgs.proto", "vision_msgs.proto"},
      {"autonomy/commsgs/proto/diagnostic_msgs.proto", "diagnostic_msgs.proto"},
      {"autonomy/commsgs/proto/tf2_msgs.proto", "tf2_msgs.proto"},
      {"autonomy/commsgs/proto/vehicle_msgs.proto", "vehicle_msgs.proto"},
      {"autonomy/commsgs/proto/error_code.proto", "error_code.proto"},
  }};
  for (const auto& mapping : kFileMappings) {
    if (normalized == mapping.first) {
      return mapping.second;
    }
  }
  return normalized;
}

bool SchemaCompatibility::BuildNormalizedFileDescriptorSet(
    const std::string& message_type, std::string* descriptor_set) {
  if (descriptor_set == nullptr) {
    return false;
  }

  const google::protobuf::Descriptor* descriptor =
      google::protobuf::DescriptorPool::generated_pool()->FindMessageTypeByName(
          message_type);
  if (descriptor == nullptr) {
    return false;
  }

  google::protobuf::FileDescriptorSet file_descriptor_set;
  std::unordered_set<std::string> seen_files;
  AppendGeneratedFileDescriptor(descriptor->file(), &file_descriptor_set,
                                &seen_files);
  NormalizeFileDescriptorSet(&file_descriptor_set);
  return file_descriptor_set.SerializeToString(descriptor_set);
}

}  // namespace visualization
}  // namespace autonomy
