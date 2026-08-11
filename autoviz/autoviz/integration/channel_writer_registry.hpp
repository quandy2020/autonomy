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

struct PublishDiagnostics {
  bool topology_has_reader = false;
  bool writer_has_reader = false;
  std::string detail;
};

/** Caches one RawMessage writer per channel (matches teleop + autolink echo). */
class ChannelWriterRegistry {
 public:
  static ChannelWriterRegistry& instance();

  void setNode(const std::shared_ptr<::autolink::Node>& node);
  bool publish(const std::string& channel, const std::string& payload,
               const std::string& message_type = {});

  /** Non-blocking loop publish: no reader wait, single write attempt. */
  bool publishLoop(const std::string& channel, const std::string& payload,
                   const std::string& message_type = {});

  PublishDiagnostics lastDiagnostics() const;

 private:
  ChannelWriterRegistry() = default;

  bool ensureWriterLocked(const std::string& channel,
                          const std::string& message_type);
  void resetWriterLocked(const std::string& channel);
  bool waitForWriterReader(
      const std::shared_ptr<::autolink::Writer<::autolink::message::RawMessage>>&
          writer,
      const std::string& channel, int timeout_ms) const;
  void setDiagnostics(bool topology_has_reader, bool writer_has_reader,
                      const std::string& detail);

  std::mutex mutex_;
  mutable std::mutex diagnostics_mutex_;
  PublishDiagnostics last_diagnostics_;
  std::weak_ptr<::autolink::Node> node_;
  std::unordered_map<
      std::string,
      std::shared_ptr<::autolink::Writer<::autolink::message::RawMessage>>>
      writers_;
  std::unordered_map<std::string, std::string> writer_schema_types_;
};

}  // namespace integration
}  // namespace autoviz
