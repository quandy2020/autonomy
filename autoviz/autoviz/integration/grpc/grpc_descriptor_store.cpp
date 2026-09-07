/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/integration/grpc/grpc_descriptor_store.hpp"

#include <google/protobuf/compiler/importer.h>
#include <google/protobuf/util/json_util.h>

#include <filesystem>
#include <utility>
#include <vector>

#include "autoviz/common/protobuf_json_compat.hpp"

namespace autoviz {
namespace integration {
namespace grpc_client {
namespace {

void SetError(std::string* error, const std::string& message) {
  if (error != nullptr) {
    *error = message;
  }
}

class CollectingErrorCollector
    : public google::protobuf::compiler::MultiFileErrorCollector {
 public:
  void RecordError(absl::string_view filename, int line, int column,
                   absl::string_view message) override {
    if (!text.empty()) {
      text += '\n';
    }
    text += std::string(filename);
    text += ':';
    text += std::to_string(line);
    text += ':';
    text += std::to_string(column);
    text += ": ";
    text += std::string(message);
  }

  std::string text;
};

class PoolErrorCollector : public google::protobuf::DescriptorPool::ErrorCollector {
 public:
  void RecordError(absl::string_view filename,
                   absl::string_view element_name,
                   const google::protobuf::Message* /*descriptor*/,
                   ErrorLocation /*location*/, absl::string_view message) override {
    if (!text.empty()) {
      text += '\n';
    }
    text += std::string(filename);
    text += ": ";
    text += std::string(element_name);
    text += ": ";
    text += std::string(message);
  }

  std::string text;
};

MethodType ClassifyMethod(const google::protobuf::MethodDescriptor* method) {
  if (method == nullptr) {
    return MethodType::kUnary;
  }
  const bool client = method->client_streaming();
  const bool server = method->server_streaming();
  if (client && server) {
    return MethodType::kBidiStreaming;
  }
  if (client) {
    return MethodType::kClientStreaming;
  }
  if (server) {
    return MethodType::kServerStreaming;
  }
  return MethodType::kUnary;
}

std::string VirtualImportName(const std::string& path,
                              const std::vector<std::string>& include_paths) {
  namespace fs = std::filesystem;
  const fs::path abs = fs::weakly_canonical(fs::absolute(path));
  for (const auto& include : include_paths) {
    std::error_code ec;
    const fs::path root = fs::weakly_canonical(fs::absolute(include), ec);
    if (ec) {
      continue;
    }
    auto rel = fs::relative(abs, root, ec);
    if (!ec && !rel.empty() && *rel.begin() != "..") {
      return rel.generic_string();
    }
  }
  return abs.filename().generic_string();
}

}  // namespace

GrpcDescriptorStore::GrpcDescriptorStore() : factory_(&pool_) {}
GrpcDescriptorStore::~GrpcDescriptorStore() = default;

bool GrpcDescriptorStore::buildFileProto(
    const google::protobuf::FileDescriptorProto& file_proto, std::string* err) {
  if (pool_.FindFileByName(file_proto.name()) != nullptr) {
    return true;
  }
  PoolErrorCollector collector;
  const google::protobuf::FileDescriptor* built =
      pool_.BuildFileCollectingErrors(file_proto, &collector);
  if (built == nullptr) {
    SetError(err, collector.text.empty() ? "BuildFile failed" : collector.text);
    return false;
  }
  indexFile(built);
  return true;
}

bool GrpcDescriptorStore::ingestFileDescriptor(
    const google::protobuf::FileDescriptor* file, std::string* err) {
  if (file == nullptr) {
    SetError(err, "FileDescriptor is null");
    return false;
  }
  if (pool_.FindFileByName(file->name()) != nullptr) {
    return true;
  }

  for (int i = 0; i < file->dependency_count(); ++i) {
    if (!ingestFileDescriptor(file->dependency(i), err)) {
      return false;
    }
  }

  google::protobuf::FileDescriptorProto file_proto;
  file->CopyTo(&file_proto);
  return buildFileProto(file_proto, err);
}

void GrpcDescriptorStore::indexFile(const google::protobuf::FileDescriptor* file) {
  if (file == nullptr) {
    return;
  }
  for (int s = 0; s < file->service_count(); ++s) {
    const google::protobuf::ServiceDescriptor* service = file->service(s);
    for (int m = 0; m < service->method_count(); ++m) {
      const google::protobuf::MethodDescriptor* method = service->method(m);
      MethodInfo info;
      info.full_name = method->full_name();
      info.service_name = service->full_name();
      info.method_name = method->name();
      info.type = ClassifyMethod(method);
      methods_.push_back(std::move(info));
    }
  }
}

bool GrpcDescriptorStore::loadProtoFile(
    const std::string& path, const std::vector<std::string>& include_paths,
    std::string* err) {
  if (path.empty()) {
    SetError(err, "Proto path is empty");
    return false;
  }
  if (!std::filesystem::exists(path)) {
    SetError(err, "Proto file not found: " + path);
    return false;
  }

  google::protobuf::compiler::DiskSourceTree source_tree;
  for (const auto& include : include_paths) {
    source_tree.MapPath("", include);
  }
  // Also map the file's directory so absolute/relative paths resolve.
  source_tree.MapPath(
      "", std::filesystem::path(path).parent_path().generic_string());

  CollectingErrorCollector errors;
  google::protobuf::compiler::Importer importer(&source_tree, &errors);
  const std::string import_name = VirtualImportName(path, include_paths);
  const google::protobuf::FileDescriptor* file = importer.Import(import_name);
  if (file == nullptr) {
    SetError(err, errors.text.empty() ? "Failed to import " + import_name
                                      : errors.text);
    return false;
  }
  return ingestFileDescriptor(file, err);
}

bool GrpcDescriptorStore::loadAutomsgsRpcs(std::string* err) {
#ifndef AUTOVIZ_AUTOMSGS_PROTO_ROOT
  SetError(err, "AUTOVIZ_AUTOMSGS_PROTO_ROOT is not defined at compile time");
  return false;
#else
  namespace fs = std::filesystem;
  const fs::path root(AUTOVIZ_AUTOMSGS_PROTO_ROOT);
  const fs::path rpcs_dir = root / "rpcs";
  if (!fs::is_directory(rpcs_dir)) {
    SetError(err, "automsgs rpcs directory not found: " + rpcs_dir.string());
    return false;
  }

  google::protobuf::compiler::DiskSourceTree source_tree;
  // Source layout is proto/{msgs,rpcs,...}; imports use automsgs/{msgs,rpcs}/...
  source_tree.MapPath("automsgs/msgs", (root / "msgs").generic_string());
  source_tree.MapPath("automsgs/rpcs", (root / "rpcs").generic_string());
  source_tree.MapPath("automsgs/srvs", (root / "srvs").generic_string());
  source_tree.MapPath("automsgs/actions", (root / "actions").generic_string());

  CollectingErrorCollector errors;
  google::protobuf::compiler::Importer importer(&source_tree, &errors);

  bool any = false;
  for (const auto& entry : fs::directory_iterator(rpcs_dir)) {
    if (!entry.is_regular_file() || entry.path().extension() != ".proto") {
      continue;
    }
    const std::string import_name =
        "automsgs/rpcs/" + entry.path().filename().generic_string();
    const google::protobuf::FileDescriptor* file = importer.Import(import_name);
    if (file == nullptr) {
      SetError(err, errors.text.empty() ? "Failed to import " + import_name
                                        : errors.text);
      return false;
    }
    if (!ingestFileDescriptor(file, err)) {
      return false;
    }
    any = true;
  }

  if (!any) {
    SetError(err, "No .proto files found under " + rpcs_dir.string());
    return false;
  }
  return true;
#endif
}

bool GrpcDescriptorStore::loadFromFileDescriptorSet(
    const google::protobuf::FileDescriptorSet& fds, std::string* err) {
  if (fds.file_size() == 0) {
    SetError(err, "FileDescriptorSet is empty");
    return false;
  }

  // Multi-pass BuildFile: FDS order is not required to be topological.
  std::vector<const google::protobuf::FileDescriptorProto*> pending;
  pending.reserve(static_cast<size_t>(fds.file_size()));
  for (const auto& file_proto : fds.file()) {
    pending.push_back(&file_proto);
  }

  std::string last_err;
  while (!pending.empty()) {
    std::vector<const google::protobuf::FileDescriptorProto*> still_pending;
    still_pending.reserve(pending.size());
    size_t progress = 0;

    for (const auto* file_proto : pending) {
      if (pool_.FindFileByName(file_proto->name()) != nullptr) {
        ++progress;
        continue;
      }
      PoolErrorCollector collector;
      const google::protobuf::FileDescriptor* built =
          pool_.BuildFileCollectingErrors(*file_proto, &collector);
      if (built != nullptr) {
        indexFile(built);
        ++progress;
        continue;
      }
      last_err = collector.text.empty() ? "BuildFile failed for " + file_proto->name()
                                        : collector.text;
      still_pending.push_back(file_proto);
    }

    if (progress == 0) {
      SetError(err, last_err.empty()
                        ? "Failed to resolve FileDescriptorSet dependencies"
                        : last_err);
      return false;
    }
    pending.swap(still_pending);
  }
  return true;
}

std::vector<MethodInfo> GrpcDescriptorStore::listMethods() const {
  return methods_;
}

const google::protobuf::MethodDescriptor* GrpcDescriptorStore::findMethod(
    const std::string& full_name) const {
  return pool_.FindMethodByName(full_name);
}

MethodType GrpcDescriptorStore::methodType(
    const google::protobuf::MethodDescriptor* method) const {
  return ClassifyMethod(method);
}

std::string GrpcDescriptorStore::exampleJson(
    const google::protobuf::Descriptor* msg) const {
  if (msg == nullptr) {
    return "{}";
  }
  const google::protobuf::Message* prototype = factory_.GetPrototype(msg);
  if (prototype == nullptr) {
    return "{}";
  }
  std::unique_ptr<google::protobuf::Message> message(prototype->New());
  if (message == nullptr) {
    return "{}";
  }

  google::protobuf::util::JsonPrintOptions options;
  options.preserve_proto_field_names = true;
  SetAlwaysPrintPrimitiveFields(&options);

  std::string json;
  const auto status =
      google::protobuf::util::MessageToJsonString(*message, &json, options);
  if (!status.ok() || json.empty()) {
    return "{}";
  }
  return json;
}

}  // namespace grpc_client
}  // namespace integration
}  // namespace autoviz
