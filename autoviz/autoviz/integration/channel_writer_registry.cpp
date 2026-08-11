/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/integration/channel_writer_registry.hpp"

#include <autolink/message/protobuf_factory.hpp>
#include <autolink/proto/role_attributes.pb.h>
#include <autolink/service_discovery/topology_manager.hpp>

#include <chrono>
#include <glog/logging.h>
#include <thread>

namespace autoviz {
namespace integration {
namespace {

bool WriteWithRetry(
    const std::shared_ptr<::autolink::Writer<::autolink::message::RawMessage>>&
        writer,
    const std::string& payload, int attempts, int sleep_ms) {
  if (writer == nullptr) {
    return false;
  }
  // Proto3 all-default messages (e.g. zero Twist) serialize to an empty string;
  // that is still a valid publish payload for RawMessage.
  auto message = std::make_shared<::autolink::message::RawMessage>();
  message->message = payload;
  for (int i = 0; i < attempts; ++i) {
    if (writer->Write(message)) {
      if (i > 0) {
        LOG(INFO) << "autoviz publish succeeded after " << (i + 1)
                  << " attempts";
      }
      return true;
    }
    if (i + 1 < attempts) {
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
  }
  return false;
}

}  // namespace

ChannelWriterRegistry& ChannelWriterRegistry::instance() {
  static ChannelWriterRegistry registry;
  return registry;
}

void ChannelWriterRegistry::setNode(
    const std::shared_ptr<::autolink::Node>& node) {
  std::lock_guard<std::mutex> lock(mutex_);
  node_ = node;
  if (!node) {
    writers_.clear();
    writer_schema_types_.clear();
  }
}

PublishDiagnostics ChannelWriterRegistry::lastDiagnostics() const {
  std::lock_guard<std::mutex> lock(diagnostics_mutex_);
  return last_diagnostics_;
}

void ChannelWriterRegistry::setDiagnostics(bool topology_has_reader,
                                           bool writer_has_reader,
                                           const std::string& detail) {
  std::lock_guard<std::mutex> lock(diagnostics_mutex_);
  last_diagnostics_.topology_has_reader = topology_has_reader;
  last_diagnostics_.writer_has_reader = writer_has_reader;
  last_diagnostics_.detail = detail;
}

bool ChannelWriterRegistry::waitForWriterReader(
    const std::shared_ptr<::autolink::Writer<::autolink::message::RawMessage>>&
        writer,
    const std::string& channel, int timeout_ms) const {
  auto channel_manager =
      ::autolink::service_discovery::TopologyManager::Instance()
          ->channel_manager();
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    const bool topo =
        channel_manager != nullptr && channel_manager->HasReader(channel);
    const bool on_writer = writer != nullptr && writer->HasReader();
    if (topo && on_writer) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return writer != nullptr && writer->HasReader();
}

void ChannelWriterRegistry::resetWriterLocked(const std::string& channel) {
  writers_.erase(channel);
  writer_schema_types_.erase(channel);
}

bool ChannelWriterRegistry::ensureWriterLocked(
    const std::string& channel, const std::string& message_type) {
  auto node = node_.lock();
  if (node == nullptr || channel.empty()) {
    return false;
  }

  const auto schema_it = writer_schema_types_.find(channel);
  const bool schema_changed =
      !message_type.empty() && schema_it != writer_schema_types_.end() &&
      schema_it->second != message_type;

  auto it = writers_.find(channel);
  if (schema_changed) {
    writers_.erase(it);
    writer_schema_types_.erase(channel);
    it = writers_.end();
  }
  if (it != writers_.end() && it->second != nullptr) {
    return true;
  }

  ::autolink::proto::RoleAttributes attr;
  attr.set_channel_name(channel);
  if (!message_type.empty()) {
    attr.set_message_type(message_type);
    std::string proto_desc;
    ::autolink::message::ProtobufFactory::Instance()->GetDescriptorString(
        message_type, &proto_desc);
    if (!proto_desc.empty()) {
      attr.set_proto_desc(proto_desc);
    }
  }

  auto writer =
      node->CreateWriter<::autolink::message::RawMessage>(attr);
  if (writer == nullptr) {
    LOG(WARNING) << "Failed to create autoviz RawMessage writer for channel: "
                 << channel;
    writers_.erase(channel);
    writer_schema_types_.erase(channel);
    return false;
  }

  writers_[channel] = std::move(writer);
  if (!message_type.empty()) {
    writer_schema_types_[channel] = message_type;
  }
  return true;
}

bool ChannelWriterRegistry::publishLoop(const std::string& channel,
                                        const std::string& payload,
                                        const std::string& message_type) {
  if (channel.empty()) {
    return false;
  }

  auto node = node_.lock();
  if (node == nullptr) {
    return false;
  }

  std::shared_ptr<::autolink::Writer<::autolink::message::RawMessage>> writer;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ensureWriterLocked(channel, message_type)) {
      return false;
    }
    writer = writers_[channel];
  }

  return WriteWithRetry(writer, payload, 1, 0);
}

bool ChannelWriterRegistry::publish(const std::string& channel,
                                    const std::string& payload,
                                    const std::string& message_type) {
  if (channel.empty()) {
    setDiagnostics(false, false, "empty channel");
    return false;
  }

  auto node = node_.lock();
  if (node == nullptr) {
    LOG(WARNING) << "autoviz publish skipped: autolink node unavailable";
    setDiagnostics(false, false, "autolink node unavailable");
    return false;
  }

  for (int pass = 0; pass < 2; ++pass) {
    std::shared_ptr<::autolink::Writer<::autolink::message::RawMessage>>
        writer;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (pass > 0) {
        resetWriterLocked(channel);
      }
      if (!ensureWriterLocked(channel, message_type)) {
        setDiagnostics(false, false, "failed to create RawMessage writer");
        return false;
      }
      writer = writers_[channel];
    }

    waitForWriterReader(writer, channel, pass == 0 ? 5000 : 2000);

    if (WriteWithRetry(writer, payload, 80, 50)) {
      setDiagnostics(true, true, "ok");
      return true;
    }
  }

  auto channel_manager =
      ::autolink::service_discovery::TopologyManager::Instance()
          ->channel_manager();
  const bool topo_reader =
      channel_manager != nullptr && channel_manager->HasReader(channel);
  std::shared_ptr<::autolink::Writer<::autolink::message::RawMessage>> writer;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = writers_.find(channel);
    if (it != writers_.end()) {
      writer = it->second;
    }
  }
  const bool writer_reader = writer != nullptr && writer->HasReader();
  std::string detail = "no matched reader peer";
  if (!topo_reader) {
    detail =
        "topology has no reader on this channel; start "
        "`autolink channel echo " +
        channel +
        "` in the same process namespace (same Docker container)";
  }
  setDiagnostics(topo_reader, writer_reader, detail);
  LOG(WARNING) << "autoviz publish failed for channel " << channel
               << " topology_has_reader=" << topo_reader
               << " writer_has_reader=" << writer_reader << " detail="
               << detail;
  return false;
}

}  // namespace integration
}  // namespace autoviz
