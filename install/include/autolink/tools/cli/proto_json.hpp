/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
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

namespace autolink {
namespace tools {

bool RegisterProtoType(const std::string& type, const std::string& desc = "");

// Load a FileDescriptorSet from `protoc --descriptor_set_out` (include imports).
bool RegisterDescriptorSetFile(const std::string& path,
                               std::string* err = nullptr);

bool JsonToProtobufBytes(const std::string& type, const std::string& json,
                         std::string* out_bytes, std::string* err);

bool ProtobufBytesToJson(const std::string& type, const std::string& bytes,
                         std::string* out_json, std::string* err);

bool ProtobufBytesToDebug(const std::string& type, const std::string& bytes,
                          std::string* out_debug, std::string* err);

bool ResolveChannelType(const std::string& channel_name, std::string* type,
                        std::string* proto_desc);

bool ResolveServiceType(const std::string& service_name, std::string* type,
                        std::string* proto_desc);

}  // namespace tools
}  // namespace autolink
