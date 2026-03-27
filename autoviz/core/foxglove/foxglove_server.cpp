#include "autonomy/autoviz/core/foxglove/foxglove_server.hpp"

#include "autonomy/autoviz/core/convert/message/convert_automsgs_message.hpp"

#include "autolink/common/log.hpp"
#include "autolink/proto/proto_desc.pb.h"

#include <foxglove/channel.hpp>
#include <foxglove/server.hpp>
#include <foxglove/error.hpp>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>
#include <optional>
#include <vector>
#include <unordered_map>

namespace autoviz {
namespace server {

namespace {

// protobuf full_name may come with a leading dot (e.g. ".pkg.Msg").
std::string TrimLeadingDot(std::string s) {
  if (!s.empty() && s.front() == '.') {
    s.erase(s.begin());
  }
  return s;
}

std::string AutolinkProtoDescToFileDescriptorSetBytes(const std::string& proto_desc_str) {
  autolink::proto::ProtoDesc root;
  if (!root.ParseFromString(proto_desc_str)) {
    return "";
  }

  std::unordered_map<std::string, google::protobuf::FileDescriptorProto> by_name;
  std::vector<const autolink::proto::ProtoDesc*> stack;
  stack.push_back(&root);

  while (!stack.empty()) {
    const auto* node = stack.back();
    stack.pop_back();
    if (node == nullptr) {
      continue;
    }

    google::protobuf::FileDescriptorProto fd_proto;
    if (!fd_proto.ParseFromString(node->desc())) {
      continue;
    }
    by_name[fd_proto.name()] = fd_proto;

    for (int i = 0; i < node->dependencies_size(); ++i) {
      stack.push_back(&node->dependencies(i));
    }
  }

  google::protobuf::FileDescriptorSet fds;
  for (const auto& [_, fd_proto] : by_name) {
    *fds.add_file() = fd_proto;
  }

  std::string out;
  if (!fds.SerializeToString(&out)) {
    return "";
  }
  return out;
}

bool SchemaBytesHasFile(const std::string& schema_bytes,
                         const std::string& file_name) {
  if (schema_bytes.empty() || file_name.empty()) {
    return false;
  }
  google::protobuf::FileDescriptorSet fds;
  if (!fds.ParseFromString(schema_bytes)) {
    return false;
  }
  for (const auto& fd : fds.file()) {
    if (fd.name() == file_name) {
      return true;
    }
  }
  return false;
}

// Build a protobuf FileDescriptorSet for the message, including all
// transitive file dependencies.
std::string BuildProtobufFileDescriptorSetBytes(const std::string& msg_full_name) {
  const auto* pool = google::protobuf::DescriptorPool::generated_pool();
  if (pool == nullptr) {
    return "";
  }

  const std::string lookup_name = TrimLeadingDot(msg_full_name);
  const auto* desc = pool->FindMessageTypeByName(lookup_name);
  if (desc == nullptr) {
    // Some protobuf builds may require an explicit leading dot.
    desc = pool->FindMessageTypeByName("." + lookup_name);
  }
  if (desc == nullptr) {
    return "";
  }

  std::unordered_map<std::string, const google::protobuf::FileDescriptor*> file_by_name;
  std::vector<const google::protobuf::FileDescriptor*> stack;
  stack.push_back(desc->file());

  while (!stack.empty()) {
    auto* f = stack.back();
    stack.pop_back();
    if (f == nullptr) {
      continue;
    }
    const std::string fname = f->name();
    if (file_by_name.count(fname) > 0) {
      continue;
    }
    file_by_name[fname] = f;
    for (int i = 0; i < f->dependency_count(); ++i) {
      const auto* dep = f->dependency(i);
      if (dep != nullptr) {
        stack.push_back(dep);
      }
    }
  }

  google::protobuf::FileDescriptorSet fds;
  for (const auto& [_, fd] : file_by_name) {
    google::protobuf::FileDescriptorProto fd_proto;
    fd->CopyTo(&fd_proto);
    *fds.add_file() = std::move(fd_proto);
  }

  std::string out;
  if (!fds.SerializeToString(&out)) {
    return "";
  }
  return out;
}

}  // namespace

FoxgloveServer::FoxgloveServer(const config::FoxgloveConfig& cfg)
    : cfg_(cfg) {}

FoxgloveServer::~FoxgloveServer() {
  Stop();
}

std::string FoxgloveServer::MakeChannelKey(const std::string& topic,
                                           const std::string& msg_type,
                                           const std::string& encoding) const {
  return topic + "|" + msg_type + "|" + encoding;
}

void FoxgloveServer::Start() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (running_.exchange(true)) {
    return;
  }

  // Setup callbacks (no-op for now).
  ::foxglove::WebSocketServerCallbacks callbacks;
  callbacks.onSubscribe = [](uint64_t, const ::foxglove::ClientMetadata&) {};
  callbacks.onUnsubscribe = [](uint64_t, const ::foxglove::ClientMetadata&) {};

  ::foxglove::WebSocketServerCapabilities capabilities =
      ::foxglove::WebSocketServerCapabilities::ClientPublish |
      ::foxglove::WebSocketServerCapabilities::ConnectionGraph;

  ::foxglove::WebSocketServerOptions ws_options;
  ws_options.name = cfg_.session_id.empty() ? "autoviz" : cfg_.session_id;
  ws_options.host = cfg_.host;
  ws_options.port = static_cast<uint16_t>(cfg_.port);
  ws_options.callbacks = callbacks;
  ws_options.capabilities = capabilities;
  ws_options.supported_encodings =
      !cfg_.supported_encodings.empty() ? cfg_.supported_encodings
                                         : std::vector<std::string>{"protobuf"};

  // Create server; if port is taken, try subsequent ports.
  std::optional<::foxglove::FoxgloveError> last_error;
  std::optional<::foxglove::WebSocketServer> created_server;

  const uint16_t base_port = static_cast<uint16_t>(cfg_.port);
  constexpr int kMaxPortTries = 20;
  for (int i = 0; i < kMaxPortTries; ++i) {
    auto opts = ws_options;
    opts.port = static_cast<uint16_t>(base_port + i);
    auto res = ::foxglove::WebSocketServer::create(std::move(opts));
    if (res.has_value()) {
      created_server.emplace(std::move(res.value()));
      if (i > 0) {
        AINFO << "Foxglove port " << base_port << " unavailable, using "
              << (base_port + i);
      }
      break;
    }
    last_error = res.error();
  }

  if (!created_server.has_value()) {
    AERROR << "Failed to create Foxglove WebSocket server (last error): "
           << (last_error ? static_cast<int>(*last_error) : -1);
    running_.store(false);
    return;
  }

  server_ = std::make_unique<::foxglove::WebSocketServer>(std::move(*created_server));
  AINFO << "Foxglove WebSocket server started on " << ws_options.host << ":"
        << server_->port();
}

void FoxgloveServer::Stop() {
  std::unordered_map<std::string, std::unique_ptr<::foxglove::RawChannel>> to_close;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_.exchange(false)) {
      return;
    }
    to_close = std::move(channels_);
    channels_.clear();
    foxglove_native_schema_blob_by_key_.clear();
  }

  // Close channels outside lock (RawChannel is thread-safe).
  for (auto& kv : to_close) {
    if (kv.second) {
      kv.second->close();
    }
  }

  if (server_) {
    auto err = server_->stop();
    if (err != ::foxglove::FoxgloveError::Ok) {
      AWARN << "Error stopping Foxglove WebSocket server: "
            << static_cast<int>(err);
    }
    server_.reset();
  }
}

void FoxgloveServer::RegisterSchema(
    const converter::FoxgloveMessageSchema& schema) {
  std::lock_guard<std::mutex> lock(mutex_);
  schemas_by_proto_type_[schema.proto_full_name] = schema;
}

void FoxgloveServer::EnsureChannel(const std::string& topic,
                                   const std::string& msg_type,
                                   const std::string& encoding,
                                   const std::string* proto_desc) {
  if (!running_.load()) {
    return;
  }

  const std::string key = MakeChannelKey(topic, msg_type, encoding);

  std::lock_guard<std::mutex> lock(mutex_);
  if (channels_.count(key) > 0) {
    return;
  }

  std::optional<::foxglove::Schema> schema = std::nullopt;
  std::string schema_bytes;

  // Preferred: build a complete FileDescriptorSet (including dependencies)
  // from protobuf descriptors in this binary.
  const std::string built_schema_bytes = BuildProtobufFileDescriptorSetBytes(msg_type);

  const std::string autolink_schema_bytes =
      (proto_desc != nullptr && !proto_desc->empty())
          ? AutolinkProtoDescToFileDescriptorSetBytes(*proto_desc)
          : std::string{};

  // If possible, validate which schema actually includes the dependency
  // file for std_msgs.Header. This is the failure mode Foxglove reports.
  const std::string header_file_name = "automsgs/msgs/std_msgs/header.proto";

  const bool autolink_has_header_file =
      SchemaBytesHasFile(autolink_schema_bytes, header_file_name);
  const bool built_has_header_file =
      SchemaBytesHasFile(built_schema_bytes, header_file_name);

  if (!header_file_name.empty()) {
    if (autolink_has_header_file) {
      schema_bytes = autolink_schema_bytes;
    } else if (built_has_header_file) {
      schema_bytes = built_schema_bytes;
    } else {
      schema_bytes = !autolink_schema_bytes.empty() ? autolink_schema_bytes
                                                    : built_schema_bytes;
    }
  } else {
    schema_bytes = !autolink_schema_bytes.empty() ? autolink_schema_bytes
                                                  : built_schema_bytes;
  }

  if (!schema_bytes.empty()) {
    // Schema requires stable backing memory for foxglove::Schema::data pointer.
    proto_desc_by_proto_type_[msg_type] = std::move(schema_bytes);
    const auto& stored = proto_desc_by_proto_type_[msg_type];

    ::foxglove::Schema fs_schema;
    fs_schema.name = TrimLeadingDot(msg_type);
    fs_schema.encoding = "protobuf";
    fs_schema.data = reinterpret_cast<const std::byte*>(stored.data());
    fs_schema.data_len = stored.size();
    schema = fs_schema;
  }

  auto ch_res =
      ::foxglove::RawChannel::create(topic, encoding, schema /* schema */);
  if (!ch_res.has_value()) {
    AWARN << "Failed to create RawChannel for topic=" << topic
          << " err=" << static_cast<int>(ch_res.error());
    return;
  }

  auto ch = std::move(ch_res.value());
  channels_.emplace(key, std::make_unique<::foxglove::RawChannel>(std::move(ch)));
}

void FoxgloveServer::EmitProtobuf(const std::string& topic,
                                const std::string& msg_type,
                                const std::string& encoding,
                                const void* data,
                                std::size_t size,
                                std::uint64_t timestamp_ns,
                                const std::string* proto_desc) {
  if (!running_.load()) {
    return;
  }

  if (data == nullptr || size == 0) {
    return;
  }

  EnsureChannel(topic, msg_type, encoding, proto_desc);

  const std::string key = MakeChannelKey(topic, msg_type, encoding);
  ::foxglove::RawChannel* channel = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = channels_.find(key);
    if (it == channels_.end() || it->second == nullptr) {
      return;
    }
    channel = it->second.get();
  }

  const auto* bytes = reinterpret_cast<const std::byte*>(data);
  auto err = channel->log(bytes, size, timestamp_ns);
  if (err != ::foxglove::FoxgloveError::Ok) {
    AWARN << "Failed to log message to foxglove channel topic=" << topic
          << " err=" << static_cast<int>(err);
  }
}

void FoxgloveServer::PrepareFoxgloveChannel(const std::string& topic,
                                                 const std::string& schema_name,
                                                 const std::string& schema_encoding,
                                                 const std::string& schema_data_blob) {
#if defined(AUTOVIZ_FOXGLOVE_HAS_ENCODE) && AUTOVIZ_FOXGLOVE_HAS_ENCODE
  if (!running_.load()) {
    return;
  }
  const std::string name = TrimLeadingDot(schema_name);
  const std::string key = MakeChannelKey(topic, name, schema_encoding);
  std::lock_guard<std::mutex> lock(mutex_);
  if (channels_.count(key) > 0) {
    return;
  }
  if (!schema_data_blob.empty()) {
    foxglove_native_schema_blob_by_key_[key] = schema_data_blob;
  }
  const auto it = foxglove_native_schema_blob_by_key_.find(key);
  const std::string* blob =
      it != foxglove_native_schema_blob_by_key_.end() ? &it->second : nullptr;

  ::foxglove::Schema fs;
  fs.name = name;
  fs.encoding = schema_encoding;
  if (blob != nullptr && !blob->empty()) {
    fs.data = reinterpret_cast<const std::byte*>(blob->data());
    fs.data_len = blob->size();
  } else {
    fs.data = nullptr;
    fs.data_len = 0;
  }

  auto ch_res = ::foxglove::RawChannel::create(topic, schema_encoding, fs);
  if (!ch_res.has_value()) {
    AWARN << "Failed to create native Foxglove RawChannel topic=" << topic
          << " schema=" << name << " err=" << static_cast<int>(ch_res.error());
    return;
  }
  auto ch = std::move(ch_res.value());
  channels_.emplace(key, std::make_unique<::foxglove::RawChannel>(std::move(ch)));
#else
  (void)topic;
  (void)schema_name;
  (void)schema_encoding;
  (void)schema_data_blob;
#endif
}

void FoxgloveServer::EmitFoxglove(const std::string& topic,
                                              const converter::FoxgloveConvertedMessage& m,
                                              std::uint64_t timestamp_ns) {
#if defined(AUTOVIZ_FOXGLOVE_HAS_ENCODE) && AUTOVIZ_FOXGLOVE_HAS_ENCODE
  if (!running_.load() || !m.ok || m.payload.empty() || m.schema_name.empty()) {
    return;
  }
  PrepareFoxgloveChannel(topic, m.schema_name, m.schema_encoding, m.schema_data);
  const std::string key =
      MakeChannelKey(topic, TrimLeadingDot(m.schema_name), m.schema_encoding);
  ::foxglove::RawChannel* channel = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = channels_.find(key);
    if (it == channels_.end() || it->second == nullptr) {
      return;
    }
    channel = it->second.get();
  }
  const auto* bytes = reinterpret_cast<const std::byte*>(m.payload.data());
  auto err = channel->log(bytes, m.payload.size(), timestamp_ns);
  if (err != ::foxglove::FoxgloveError::Ok) {
    AWARN << "Failed to log native Foxglove message topic=" << topic
          << " schema=" << m.schema_name << " err=" << static_cast<int>(err);
  }
#else
  (void)topic;
  (void)m;
  (void)timestamp_ns;
#endif
}

}  // namespace server
}  // namespace autoviz

