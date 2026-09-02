/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/integration/grpc/grpc_reflection.hpp"

#include <filesystem>
#include <set>
#include <vector>

#include "autoviz/integration/grpc/grpc_descriptor_store.hpp"

#if AUTOVIZ_ENABLE_GRPC
#include <grpcpp/generic/generic_stub.h>
#include <grpcpp/support/async_stream.h>
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
  if (!buffer.Dump(&slices).ok()) {
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

bool WaitCq(grpc::CompletionQueue* cq, void* expected, std::string* err) {
  void* got = nullptr;
  bool ok = false;
  if (!cq->Next(&got, &ok)) {
    SetError(err, "Reflection CQ shut down unexpectedly");
    return false;
  }
  if (got != expected || !ok) {
    SetError(err, "Reflection RPC stream operation failed");
    return false;
  }
  return true;
}

std::string ResolveReflectionProtoPath() {
#ifdef AUTOVIZ_GRPC_REFLECTION_PROTO
  {
    const std::string configured(AUTOVIZ_GRPC_REFLECTION_PROTO);
    if (std::filesystem::exists(configured)) {
      return configured;
    }
  }
#endif
  const std::vector<std::string> candidates = {
      "autoviz/resources/grpc/reflection_v1alpha.proto",
      "resources/grpc/reflection_v1alpha.proto",
      "../resources/grpc/reflection_v1alpha.proto",
      "../../resources/grpc/reflection_v1alpha.proto",
  };
  for (const auto& path : candidates) {
    if (std::filesystem::exists(path)) {
      return std::filesystem::absolute(path).generic_string();
    }
  }
#ifndef AUTOVIZ_GRPC_REFLECTION_PROTO
  return "autoviz/resources/grpc/reflection_v1alpha.proto";
#else
  return std::string(AUTOVIZ_GRPC_REFLECTION_PROTO);
#endif
}

bool LoadReflectionDescriptors(GrpcDescriptorStore* store, std::string* err) {
  const std::string path = ResolveReflectionProtoPath();
  if (!std::filesystem::exists(path)) {
    SetError(err, "reflection_v1alpha.proto not found: " + path);
    return false;
  }
  std::vector<std::string> includes;
  includes.push_back(
      std::filesystem::path(path).parent_path().generic_string());
  return store->loadProtoFile(path, includes, err);
}

bool WriteReflectionRequest(
    grpc::ClientAsyncReaderWriterInterface<grpc::ByteBuffer, grpc::ByteBuffer>*
        stream,
    grpc::CompletionQueue* cq, const google::protobuf::Message& request,
    void* tag, std::string* err) {
  std::string bytes;
  if (!request.SerializeToString(&bytes)) {
    SetError(err, "Failed to serialize reflection request");
    return false;
  }
  grpc::ByteBuffer buffer = StringToByteBuffer(bytes);
  stream->Write(buffer, tag);
  return WaitCq(cq, tag, err);
}

bool ReadReflectionResponse(
    grpc::ClientAsyncReaderWriterInterface<grpc::ByteBuffer, grpc::ByteBuffer>*
        stream,
    grpc::CompletionQueue* cq, google::protobuf::Message* response, void* tag,
    std::string* err) {
  grpc::ByteBuffer buffer;
  stream->Read(&buffer, tag);
  if (!WaitCq(cq, tag, err)) {
    return false;
  }
  const std::string bytes = ByteBufferToString(buffer);
  if (!response->ParseFromString(bytes)) {
    SetError(err, "Failed to parse reflection response");
    return false;
  }
  return true;
}

void CollectFileDescriptors(
    const google::protobuf::Message& file_desc_response,
    google::protobuf::FileDescriptorSet* out,
    std::set<std::string>* seen_names) {
  const google::protobuf::Descriptor* desc = file_desc_response.GetDescriptor();
  const google::protobuf::Reflection* refl =
      file_desc_response.GetReflection();
  const google::protobuf::FieldDescriptor* field =
      desc->FindFieldByName("file_descriptor_proto");
  if (field == nullptr || !field->is_repeated()) {
    return;
  }
  const int n = refl->FieldSize(file_desc_response, field);
  for (int i = 0; i < n; ++i) {
    const std::string& bytes =
        refl->GetRepeatedString(file_desc_response, field, i);
    google::protobuf::FileDescriptorProto file_proto;
    if (!file_proto.ParseFromString(bytes)) {
      continue;
    }
    if (!seen_names->insert(file_proto.name()).second) {
      continue;
    }
    *out->add_file() = std::move(file_proto);
  }
}

#endif  // AUTOVIZ_ENABLE_GRPC

}  // namespace

bool FetchServerReflectionFileDescriptorSet(
#if AUTOVIZ_ENABLE_GRPC
    const std::shared_ptr<grpc::Channel>& channel,
#endif
    google::protobuf::FileDescriptorSet* out, std::string* err) {
#if !AUTOVIZ_ENABLE_GRPC
  (void)out;
  SetError(err, "gRPC disabled");
  return false;
#else
  if (!channel) {
    SetError(err, "gRPC channel is null");
    return false;
  }
  if (out == nullptr) {
    SetError(err, "FileDescriptorSet output is null");
    return false;
  }
  out->Clear();

  GrpcDescriptorStore reflection_store;
  if (!LoadReflectionDescriptors(&reflection_store, err)) {
    return false;
  }

  const google::protobuf::MethodDescriptor* method =
      reflection_store.findMethod(
          "grpc.reflection.v1alpha.ServerReflection.ServerReflectionInfo");
  if (method == nullptr) {
    SetError(err, "ServerReflectionInfo method not found in reflection proto");
    return false;
  }

  const google::protobuf::Message* req_proto =
      reflection_store.factory().GetPrototype(method->input_type());
  const google::protobuf::Message* resp_proto =
      reflection_store.factory().GetPrototype(method->output_type());
  if (req_proto == nullptr || resp_proto == nullptr) {
    SetError(err, "Failed to get reflection message prototypes");
    return false;
  }

  grpc::ClientContext context;
  grpc::CompletionQueue cq;
  grpc::GenericStub stub(channel);
  void* tag = reinterpret_cast<void*>(1);

  std::unique_ptr<
      grpc::ClientAsyncReaderWriterInterface<grpc::ByteBuffer, grpc::ByteBuffer>>
      stream = stub.PrepareCall(
          &context,
          "/grpc.reflection.v1alpha.ServerReflection/ServerReflectionInfo",
          &cq);
  if (!stream) {
    SetError(err, "Failed to prepare reflection call");
    return false;
  }
  stream->StartCall(tag);
  if (!WaitCq(&cq, tag, err)) {
    return false;
  }

  // list_services
  {
    std::unique_ptr<google::protobuf::Message> req(req_proto->New());
    const google::protobuf::FieldDescriptor* list_field =
        req->GetDescriptor()->FindFieldByName("list_services");
    if (list_field == nullptr) {
      SetError(err, "list_services field missing");
      return false;
    }
    req->GetReflection()->SetString(req.get(), list_field, "");
    if (!WriteReflectionRequest(stream.get(), &cq, *req, tag, err)) {
      return false;
    }
  }

  std::vector<std::string> service_names;
  {
    std::unique_ptr<google::protobuf::Message> resp(resp_proto->New());
    if (!ReadReflectionResponse(stream.get(), &cq, resp.get(), tag, err)) {
      return false;
    }
    const google::protobuf::Reflection* refl = resp->GetReflection();
    const google::protobuf::FieldDescriptor* error_field =
        resp->GetDescriptor()->FindFieldByName("error_response");
    if (error_field != nullptr && refl->HasField(*resp, error_field)) {
      const google::protobuf::Message& err_msg =
          refl->GetMessage(*resp, error_field);
      const auto* err_desc = err_msg.GetDescriptor();
      const auto* code_f = err_desc->FindFieldByName("error_code");
      const auto* msg_f = err_desc->FindFieldByName("error_message");
      std::string detail = "Reflection error";
      if (msg_f != nullptr) {
        detail = err_msg.GetReflection()->GetString(err_msg, msg_f);
      }
      if (code_f != nullptr) {
        detail += " (code " +
                  std::to_string(
                      err_msg.GetReflection()->GetInt32(err_msg, code_f)) +
                  ")";
      }
      SetError(err, detail);
      return false;
    }
    const google::protobuf::FieldDescriptor* list_resp_field =
        resp->GetDescriptor()->FindFieldByName("list_services_response");
    if (list_resp_field == nullptr ||
        !refl->HasField(*resp, list_resp_field)) {
      SetError(err, "Reflection response missing list_services_response");
      return false;
    }
    const google::protobuf::Message& list_msg =
        refl->GetMessage(*resp, list_resp_field);
    const google::protobuf::FieldDescriptor* service_field =
        list_msg.GetDescriptor()->FindFieldByName("service");
    if (service_field == nullptr) {
      SetError(err, "ListServiceResponse.service missing");
      return false;
    }
    const int n = list_msg.GetReflection()->FieldSize(list_msg, service_field);
    for (int i = 0; i < n; ++i) {
      const google::protobuf::Message& svc =
          list_msg.GetReflection()->GetRepeatedMessage(list_msg, service_field,
                                                       i);
      const auto* name_f = svc.GetDescriptor()->FindFieldByName("name");
      if (name_f == nullptr) {
        continue;
      }
      const std::string name =
          svc.GetReflection()->GetString(svc, name_f);
      if (name == "grpc.reflection.v1alpha.ServerReflection") {
        continue;
      }
      service_names.push_back(name);
    }
  }

  std::set<std::string> seen_names;
  for (const std::string& service_name : service_names) {
    std::unique_ptr<google::protobuf::Message> req(req_proto->New());
    const google::protobuf::FieldDescriptor* symbol_field =
        req->GetDescriptor()->FindFieldByName("file_containing_symbol");
    if (symbol_field == nullptr) {
      SetError(err, "file_containing_symbol field missing");
      return false;
    }
    req->GetReflection()->SetString(req.get(), symbol_field, service_name);
    if (!WriteReflectionRequest(stream.get(), &cq, *req, tag, err)) {
      return false;
    }

    std::unique_ptr<google::protobuf::Message> resp(resp_proto->New());
    if (!ReadReflectionResponse(stream.get(), &cq, resp.get(), tag, err)) {
      return false;
    }
    const google::protobuf::Reflection* refl = resp->GetReflection();
    const google::protobuf::FieldDescriptor* error_field =
        resp->GetDescriptor()->FindFieldByName("error_response");
    if (error_field != nullptr && refl->HasField(*resp, error_field)) {
      const google::protobuf::Message& err_msg =
          refl->GetMessage(*resp, error_field);
      const auto* msg_f =
          err_msg.GetDescriptor()->FindFieldByName("error_message");
      SetError(err, msg_f != nullptr
                        ? err_msg.GetReflection()->GetString(err_msg, msg_f)
                        : "Reflection file_containing_symbol failed");
      return false;
    }
    const google::protobuf::FieldDescriptor* fds_field =
        resp->GetDescriptor()->FindFieldByName("file_descriptor_response");
    if (fds_field == nullptr || !refl->HasField(*resp, fds_field)) {
      continue;
    }
    CollectFileDescriptors(refl->GetMessage(*resp, fds_field), out,
                            &seen_names);
  }

  stream->WritesDone(tag);
  if (!WaitCq(&cq, tag, err)) {
    return false;
  }
  grpc::Status status;
  stream->Finish(&status, tag);
  if (!WaitCq(&cq, tag, err)) {
    return false;
  }
  if (!status.ok() && out->file_size() == 0) {
    SetError(err, status.error_message().empty()
                      ? "Reflection stream finished with error"
                      : status.error_message());
    return false;
  }
  if (out->file_size() == 0) {
    SetError(err, service_names.empty()
                      ? "Reflection listed no services"
                      : "Reflection returned no file descriptors");
    return false;
  }
  return true;
#endif
}

}  // namespace grpc_client
}  // namespace integration
}  // namespace autoviz
