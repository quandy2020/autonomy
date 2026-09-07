/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/integration/grpc/grpc_invoker.hpp"

#include <chrono>
#include <vector>

#include "autoviz/integration/grpc/grpc_descriptor_store.hpp"
#include "autoviz/integration/grpc/grpc_json_codec.hpp"
#include "autoviz/integration/grpc/grpc_session.hpp"

#if AUTOVIZ_ENABLE_GRPC
#include <grpcpp/impl/client_unary_call.h>
#include <grpcpp/impl/rpc_method.h>
#include <grpcpp/support/byte_buffer.h>
#include <grpcpp/support/slice.h>
#endif

namespace autoviz {
namespace integration {
namespace grpc_client {
namespace {

void SetError(std::string* err, const std::string& message) {
  if (err != nullptr) {
    *err = message;
  }
}

#if AUTOVIZ_ENABLE_GRPC
std::string ByteBufferToString(const grpc::ByteBuffer& buffer) {
  std::vector<grpc::Slice> slices;
  const auto status = buffer.Dump(&slices);
  if (!status.ok()) {
    return {};
  }
  std::string out;
  size_t total = 0;
  for (const auto& slice : slices) {
    total += slice.size();
  }
  out.reserve(total);
  for (const auto& slice : slices) {
    out.append(reinterpret_cast<const char*>(slice.begin()), slice.size());
  }
  return out;
}

grpc::ByteBuffer StringToByteBuffer(const std::string& bytes) {
  grpc::Slice slice(bytes);
  return grpc::ByteBuffer(&slice, 1);
}
#endif

}  // namespace

GrpcInvoker::GrpcInvoker(GrpcDescriptorStore* store) : store_(store) {}

UnaryResult GrpcInvoker::unaryCall(
#if AUTOVIZ_ENABLE_GRPC
    const std::shared_ptr<grpc::Channel>& channel,
#endif
    const std::string& method_full_name, const std::string& request_json,
    bool include_defaults, int timeout_ms, GrpcSession* session_for_cancel,
    std::string* err) {
  UnaryResult result;
#if !AUTOVIZ_ENABLE_GRPC
  (void)method_full_name;
  (void)request_json;
  (void)include_defaults;
  (void)timeout_ms;
  (void)session_for_cancel;
  SetError(err, "gRPC disabled");
  result.status.code = static_cast<int>(14);  // UNAVAILABLE
  result.status.message = "gRPC disabled";
  return result;
#else
  if (store_ == nullptr) {
    SetError(err, "Descriptor store is null");
    return result;
  }
  if (!channel) {
    SetError(err, "gRPC channel is null");
    return result;
  }

  const std::string normalized = NormalizeMethodFullName(method_full_name);
  const google::protobuf::MethodDescriptor* method =
      store_->findMethod(normalized);
  if (method == nullptr) {
    SetError(err, "Method not found: " + normalized);
    return result;
  }

  std::string parse_err;
  auto request_msg = JsonToDynamicMessage(request_json, method->input_type(),
                                          store_->factory(), &parse_err);
  if (!request_msg) {
    SetError(err, parse_err.empty() ? "Failed to parse request JSON"
                                    : parse_err);
    return result;
  }

  std::string request_bytes;
  if (!request_msg->SerializeToString(&request_bytes)) {
    SetError(err, "Failed to serialize request message");
    return result;
  }

  const std::string method_path =
      std::string("/") + std::string(method->service()->full_name()) + "/" +
      std::string(method->name());

  grpc::ClientContext context;
  if (timeout_ms > 0) {
    const auto deadline =
        std::chrono::system_clock::now() + std::chrono::milliseconds(timeout_ms);
    context.set_deadline(deadline);
  }
  if (session_for_cancel != nullptr) {
    session_for_cancel->setActiveContext(&context);
  }

  grpc::ByteBuffer request_buffer = StringToByteBuffer(request_bytes);
  grpc::ByteBuffer response_buffer;

  const auto t0 = std::chrono::steady_clock::now();
  const grpc::internal::RpcMethod rpc_method(
      method_path.c_str(), /*suffix_for_stats=*/nullptr,
      grpc::internal::RpcMethod::NORMAL_RPC);
  const grpc::Status status = grpc::internal::BlockingUnaryCall(
      channel.get(), rpc_method, &context, request_buffer, &response_buffer);
  const auto t1 = std::chrono::steady_clock::now();

  if (session_for_cancel != nullptr) {
    session_for_cancel->setActiveContext(nullptr);
  }

  result.status.code = static_cast<int>(status.error_code());
  result.status.message = status.error_message();
  result.status.latency_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();

  if (!status.ok()) {
    // Still try to surface empty JSON; caller shows status.
    result.response_json.clear();
    return result;
  }

  const std::string response_bytes = ByteBufferToString(response_buffer);
  const google::protobuf::Message* prototype =
      store_->factory().GetPrototype(method->output_type());
  if (prototype == nullptr) {
    SetError(err, "Failed to get response prototype");
    return result;
  }
  std::unique_ptr<google::protobuf::Message> response_msg(prototype->New());
  if (response_msg == nullptr) {
    SetError(err, "Failed to create response message");
    return result;
  }
  if (!response_msg->ParseFromString(response_bytes)) {
    SetError(err, "Failed to parse response protobuf");
    return result;
  }

  std::string json_err;
  if (!DynamicMessageToJson(*response_msg, include_defaults,
                            &result.response_json, &json_err)) {
    SetError(err, json_err.empty() ? "Failed to encode response JSON"
                                   : json_err);
    result.response_json.clear();
    return result;
  }
  return result;
#endif
}

}  // namespace grpc_client
}  // namespace integration
}  // namespace autoviz
