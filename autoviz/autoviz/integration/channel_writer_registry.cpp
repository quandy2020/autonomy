/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/integration/channel_writer_registry.hpp"

#include <glog/logging.h>

namespace autoviz {
namespace integration {

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
  }
}

bool ChannelWriterRegistry::ensureWriterLocked(const std::string& channel) {
  auto node = node_.lock();
  if (node == nullptr || channel.empty()) {
    return false;
  }
  auto& writer = writers_[channel];
  if (writer != nullptr) {
    return true;
  }
  writer = node->CreateWriter<::autolink::message::RawMessage>(channel);
  if (writer == nullptr) {
    LOG(WARNING) << "Failed to create autoviz writer for channel: " << channel;
  }
  return writer != nullptr;
}

bool ChannelWriterRegistry::publish(const std::string& channel,
                                    const std::string& payload) {
  if (channel.empty() || payload.empty()) {
    return false;
  }

  std::shared_ptr<::autolink::Writer<::autolink::message::RawMessage>> writer;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ensureWriterLocked(channel)) {
      return false;
    }
    writer = writers_[channel];
  }

  if (writer == nullptr) {
    return false;
  }
  auto message = std::make_shared<::autolink::message::RawMessage>();
  message->message = payload;
  return writer->Write(message);
}

}  // namespace integration
}  // namespace autoviz
