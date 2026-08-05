/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "autolink/message/raw_message.hpp"
#include "autolink/node/node.hpp"
#include "autolink/node/writer.hpp"

namespace autoviz {
namespace integration {

/** Caches one RawMessage writer per channel for autoviz publish panels/tools. */
class ChannelWriterRegistry {
 public:
  static ChannelWriterRegistry& instance();

  void setNode(const std::shared_ptr<::autolink::Node>& node);
  bool publish(const std::string& channel, const std::string& payload);

 private:
  ChannelWriterRegistry() = default;

  bool ensureWriterLocked(const std::string& channel);

  std::mutex mutex_;
  std::weak_ptr<::autolink::Node> node_;
  std::unordered_map<std::string,
                     std::shared_ptr<::autolink::Writer<::autolink::message::RawMessage>>>
      writers_;
};

}  // namespace integration
}  // namespace autoviz
