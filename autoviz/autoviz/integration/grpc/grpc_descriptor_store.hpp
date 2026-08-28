/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/dynamic_message.h>

#include "autoviz/integration/grpc/grpc_types.hpp"

namespace autoviz {
namespace integration {
namespace grpc_client {

class GrpcDescriptorStore {
 public:
  GrpcDescriptorStore();
  ~GrpcDescriptorStore();

  GrpcDescriptorStore(const GrpcDescriptorStore&) = delete;
  GrpcDescriptorStore& operator=(const GrpcDescriptorStore&) = delete;

  bool loadProtoFile(const std::string& path,
                     const std::vector<std::string>& include_paths,
                     std::string* err);

  /** Load all .proto files under AUTOVIZ_AUTOMSGS_PROTO_ROOT/rpcs when set. */
  bool loadAutomsgsRpcs(std::string* err);

  bool loadFromFileDescriptorSet(
      const google::protobuf::FileDescriptorSet& fds, std::string* err);

  std::vector<MethodInfo> listMethods() const;

  const google::protobuf::MethodDescriptor* findMethod(
      const std::string& full_name) const;

  MethodType methodType(
      const google::protobuf::MethodDescriptor* method) const;

  /** Stub example JSON for |msg| (scalars default; nested messages {}). */
  std::string exampleJson(const google::protobuf::Descriptor* msg) const;

  google::protobuf::DynamicMessageFactory& factory() { return factory_; }

 private:
  bool buildFileProto(const google::protobuf::FileDescriptorProto& file_proto,
                      std::string* err);
  bool ingestFileDescriptor(const google::protobuf::FileDescriptor* file,
                            std::string* err);
  void indexFile(const google::protobuf::FileDescriptor* file);

  google::protobuf::DescriptorPool pool_;
  mutable google::protobuf::DynamicMessageFactory factory_;
  std::vector<MethodInfo> methods_;
};

}  // namespace grpc_client
}  // namespace integration
}  // namespace autoviz
