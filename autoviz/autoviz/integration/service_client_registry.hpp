/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "autolink/message/raw_message.hpp"
#include "autolink/node/node.hpp"
#include "autolink/service/client.hpp"

namespace autoviz {
namespace integration {

struct ServiceCallResult {
  bool ok = false;
  std::string response_bytes;
  std::string error;
};

/** Caches RawMessage service clients for Autoviz service call panels. */
class ServiceClientRegistry {
 public:
  static ServiceClientRegistry& instance();

  void setNode(const std::shared_ptr<::autolink::Node>& node);

  ServiceCallResult call(const std::string& service_name,
                         const std::string& request_bytes,
                         std::chrono::seconds timeout);

  bool serviceIsReady(const std::string& service_name) const;

 private:
  ServiceClientRegistry() = default;

  using RawClient =
      ::autolink::Client<::autolink::message::RawMessage,
                         ::autolink::message::RawMessage>;
  using RawClientPtr = std::shared_ptr<RawClient>;

  RawClientPtr ensureClientLocked(const std::string& service_name);

  mutable std::mutex mutex_;
  std::weak_ptr<::autolink::Node> node_;
  std::unordered_map<std::string, RawClientPtr> clients_;
};

}  // namespace integration
}  // namespace autoviz
