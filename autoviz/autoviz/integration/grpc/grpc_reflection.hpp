/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#ifndef AUTOVIZ_ENABLE_GRPC
#define AUTOVIZ_ENABLE_GRPC 0
#endif

#include <memory>
#include <string>

#include <google/protobuf/descriptor.pb.h>

#if AUTOVIZ_ENABLE_GRPC
#include <grpcpp/channel.h>
#endif

namespace autoviz {
namespace integration {
namespace grpc_client {

/**
 * Fetch service descriptors via gRPC Server Reflection (v1alpha).
 * Returns a FileDescriptorSet suitable for GrpcDescriptorStore::loadFromFileDescriptorSet.
 */
bool FetchServerReflectionFileDescriptorSet(
#if AUTOVIZ_ENABLE_GRPC
    const std::shared_ptr<grpc::Channel>& channel,
#endif
    google::protobuf::FileDescriptorSet* out, std::string* err);

}  // namespace grpc_client
}  // namespace integration
}  // namespace autoviz
