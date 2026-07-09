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

#pragma once

#include <string>

#include <google/protobuf/descriptor.pb.h>

namespace autonomy {
namespace visualization {

class SchemaCompatibility {
 public:
  static bool IsDirectlyRenderableIn3D(const std::string& message_type);
  static bool RequiresPayloadAdaptation(const std::string& message_type);
  static bool IsBridgeable(const std::string& message_type);
  static bool IsSupportedBy3D(const std::string& message_type);
  static void NormalizeFileDescriptorProto(
      google::protobuf::FileDescriptorProto* file_proto);
  static void NormalizeFileDescriptorSet(
      google::protobuf::FileDescriptorSet* file_set);
  static std::string NormalizeSchemaName(const std::string& message_type);
  static std::string NormalizeTypeReference(const std::string& type_name);
  static std::string NormalizePackageName(const std::string& package_name);
  static std::string NormalizeFileName(const std::string& file_name);
  static bool BuildNormalizedFileDescriptorSet(
      const std::string& message_type, std::string* descriptor_set);
};

}  // namespace visualization
}  // namespace autonomy
