/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file service_client_registry.hpp
 * @brief Process-wide cache of Autolink RawMessage service clients.
 *
 * Service Call panels and similar UI invoke @ref call() with opaque request
 * bytes; the registry reuses one @c Client per service name on the shared node.
 *
 * @see ServiceCallResult
 * @see ListServices
 * @see AutolinkContext
 */

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

/**
 * @struct ServiceCallResult
 * @brief Outcome of a synchronous Autolink service call from the UI.
 */
struct ServiceCallResult {
  /** @c true when a response was received without transport / timeout error. */
  bool ok = false;
  /** Serialized response payload (RawMessage bytes) when @c ok. */
  std::string response_bytes;
  /** Human-readable error when @c ok is @c false. */
  std::string error;
};

/**
 * @class ServiceClientRegistry
 * @brief Singleton that caches RawMessage service clients for Autoviz panels.
 *
 * ## Lifetime
 *
 * Call @ref setNode() after @ref AutolinkContext::initialize(). Clients are
 * created lazily on first @ref call() / @ref serviceIsReady().
 *
 * @note Thread-safe; @ref call() blocks up to @p timeout on the calling thread
 *       (prefer a worker for long timeouts so the Qt UI stays responsive).
 */
class ServiceClientRegistry {
 public:
  /**
   * @brief Returns the process-wide registry singleton.
   * @return Reference to the shared instance.
   */
  static ServiceClientRegistry& instance();

  /**
   * @brief Attaches the Autolink node used to create clients.
   *
   * @param node Shared node from @ref AutolinkContext; may be empty to clear.
   *        Existing cached clients are invalidated when the weak node expires.
   */
  void setNode(const std::shared_ptr<::autolink::Node>& node);

  /**
   * @brief Synchronously invokes @p service_name with @p request_bytes.
   *
   * @param service_name Fully-qualified Autolink service name.
   * @param request_bytes Serialized RawMessage request body.
   * @param timeout Maximum wait for a response.
   * @return @ref ServiceCallResult with response bytes or error text.
   */
  ServiceCallResult call(const std::string& service_name,
                         const std::string& request_bytes,
                         std::chrono::seconds timeout);

  /**
   * @brief Checks whether a client for @p service_name reports ready.
   *
   * @param service_name Fully-qualified Autolink service name.
   * @return @c true if a client exists and Autolink reports the service ready.
   */
  bool serviceIsReady(const std::string& service_name) const;

 private:
  /** @brief Private default constructor (singleton). */
  ServiceClientRegistry() = default;

  /** RawMessage request/response Autolink client type. */
  using RawClient =
      ::autolink::Client<::autolink::message::RawMessage,
                         ::autolink::message::RawMessage>;
  /** Shared pointer to a cached RawMessage client. */
  using RawClientPtr = std::shared_ptr<RawClient>;

  /**
   * @brief Returns an existing client or creates one for @p service_name.
   *
   * @param service_name Service to bind.
   * @return Shared client, or empty if no node is set / creation fails.
   *
   * @note Caller must hold @c mutex_.
   */
  RawClientPtr ensureClientLocked(const std::string& service_name);

  /** Guards @c node_ and @c clients_. */
  mutable std::mutex mutex_;

  /** Weak Autolink node; expired when context shuts down. */
  std::weak_ptr<::autolink::Node> node_;

  /** Cached clients keyed by service name. */
  std::unordered_map<std::string, RawClientPtr> clients_;
};

}  // namespace integration
}  // namespace autoviz
