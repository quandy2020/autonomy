/*
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

#include "autoviz/core/bridge/schema_builder.hpp"

#include <unordered_set>

#include <google/protobuf/descriptor.pb.h>

#include "autolink/proto/proto_desc.pb.h"

namespace autoviz {
namespace bridge {
namespace {

void AppendProtoDescToFileDescriptorSet(const autolink::proto::ProtoDesc& proto_desc,
                                        google::protobuf::FileDescriptorSet* fd_set,
                                        std::unordered_set<std::string>* added_files) {
  if (fd_set == nullptr || added_files == nullptr) {
    return;
  }

  google::protobuf::FileDescriptorProto fd_proto;
  if (proto_desc.desc().empty() || !fd_proto.ParseFromString(proto_desc.desc())) {
    return;
  }

  for (const auto& dep : proto_desc.dependencies()) {
    AppendProtoDescToFileDescriptorSet(dep, fd_set, added_files);
  }

  if (!fd_proto.name().empty() && added_files->count(fd_proto.name()) > 0) {
    return;
  }
  *fd_set->add_file() = fd_proto;
  if (!fd_proto.name().empty()) {
    added_files->insert(fd_proto.name());
  }
}

}  // namespace

std::string BuildFoxgloveSchemaFromProtoDesc(const std::string& proto_desc_bin) {
  if (proto_desc_bin.empty()) {
    return "";
  }
  autolink::proto::ProtoDesc proto_desc;
  if (!proto_desc.ParseFromString(proto_desc_bin)) {
    return "";
  }
  google::protobuf::FileDescriptorSet fd_set;
  std::unordered_set<std::string> added_files;
  AppendProtoDescToFileDescriptorSet(proto_desc, &fd_set, &added_files);
  std::string serialized;
  if (!fd_set.SerializeToString(&serialized)) {
    return "";
  }
  return serialized;
}

}  // namespace bridge
}  // namespace autoviz

