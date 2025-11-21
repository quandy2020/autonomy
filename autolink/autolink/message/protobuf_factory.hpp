/**
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include "absl/strings/string_view.h"

#include "google/protobuf/compiler/parser.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/dynamic_message.h"
#include "google/protobuf/io/tokenizer.h"

#include "autolink/common/macros.hpp"
#include "autolink/proto/proto_desc.pb.h"

namespace autolink {
namespace message {

using autolink::proto::ProtoDesc;
using google::protobuf::Descriptor;
using google::protobuf::DescriptorPool;
using google::protobuf::DynamicMessageFactory;
using google::protobuf::FileDescriptor;
using google::protobuf::FileDescriptorProto;

class ErrorCollector : public google::protobuf::DescriptorPool::ErrorCollector
{
public:
    using ErrorLocation =
        google::protobuf::DescriptorPool::ErrorCollector::ErrorLocation;
    // Deprecated methods - kept for compatibility but may not be called
    void AddError(const std::string& filename, const std::string& element_name,
                  const google::protobuf::Message* descriptor,
                  ErrorLocation location, const std::string& message);

    void AddWarning(const std::string& filename,
                    const std::string& element_name,
                    const google::protobuf::Message* descriptor,
                    ErrorLocation location, const std::string& message);

    // New API in protobuf 29.0+ (if available)
    // Note: Only override if the base class has this method
    // For protobuf < 29.0, this method may not exist in the base class
    void RecordError(absl::string_view filename, absl::string_view element_name,
                     const google::protobuf::Message* descriptor,
                     ErrorLocation location,
                     absl::string_view message);
};

class ProtobufFactory
{
public:
    ~ProtobufFactory();

    // Recursively register FileDescriptorProto and all its dependencies to
    // factory.
    bool RegisterMessage(const std::string& proto_desc_str);
    bool RegisterPythonMessage(const std::string& proto_str);

    // Convert the serialized FileDescriptorProto to real descriptors and place
    // them in factory.
    // It is an error if a FileDescriptorProto contains references to types or
    // other files that are not found in the Factory.
    bool RegisterMessage(const google::protobuf::Message& message);
    bool RegisterMessage(const Descriptor& desc);
    bool RegisterMessage(const FileDescriptorProto& file_desc_proto);

    // Serialize all descriptors of the given message to string.
    static void GetDescriptorString(const google::protobuf::Message& message,
                                    std::string* desc_str);

    // Serialize all descriptors of the descriptor to string.
    static void GetDescriptorString(const Descriptor* desc,
                                    std::string* desc_str);

    // Get Serialized descriptors of messages with the given type.
    void GetDescriptorString(const std::string& type, std::string* desc_str);

    // Get the storage location of proto.
    void GetProtoPath(const std::string& type, std::string& location);

    // Given a type name, constructs the default (prototype) Message of that
    // type. Returns nullptr if no such message exists.
    google::protobuf::Message* GenerateMessageByType(
        const std::string& type) const;

    // Find a descriptor by FileDescriptorProto. Returns nullptr if not found.
    const Descriptor* FindMessageTypeByFile(
        const FileDescriptorProto& file_desc_proto);

    // Find a top-level message type by name. Returns nullptr if not found.
    const Descriptor* FindMessageTypeByName(const std::string& type) const;

    // Find a service definition by name. Returns nullptr if not found.
    const google::protobuf::ServiceDescriptor* FindServiceByName(
        const std::string& name) const;

    void GetPythonDesc(const std::string& type, std::string* desc_str);

private:
    bool RegisterMessage(const ProtoDesc& proto_desc);
    google::protobuf::Message* GetMessageByGeneratedType(
        const std::string& type) const;
    static bool GetProtoDesc(const FileDescriptor* file_desc,
                             ProtoDesc* proto_desc);

    std::mutex register_mutex_;
    std::unique_ptr<DescriptorPool> pool_ = nullptr;
    std::unique_ptr<DynamicMessageFactory> factory_ = nullptr;

    DECLARE_SINGLETON(ProtobufFactory)
};

}  // namespace message
}  // namespace autolink