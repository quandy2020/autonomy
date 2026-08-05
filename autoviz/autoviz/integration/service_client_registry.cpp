/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/integration/service_client_registry.hpp"

namespace autoviz {
namespace integration {

ServiceClientRegistry& ServiceClientRegistry::instance() {
  static ServiceClientRegistry registry;
  return registry;
}

void ServiceClientRegistry::setNode(
    const std::shared_ptr<::autolink::Node>& node) {
  std::lock_guard<std::mutex> lock(mutex_);
  node_ = node;
  clients_.clear();
}

ServiceClientRegistry::RawClientPtr ServiceClientRegistry::ensureClientLocked(
    const std::string& service_name) {
  const auto found = clients_.find(service_name);
  if (found != clients_.end() && found->second != nullptr) {
    return found->second;
  }

  auto node = node_.lock();
  if (node == nullptr || service_name.empty()) {
    return nullptr;
  }

  RawClientPtr client =
      node->CreateClient<::autolink::message::RawMessage,
                         ::autolink::message::RawMessage>(service_name);
  if (client != nullptr) {
    clients_.emplace(service_name, client);
  }
  return client;
}

bool ServiceClientRegistry::serviceIsReady(const std::string& service_name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = clients_.find(service_name);
  if (found == clients_.end() || found->second == nullptr) {
    return false;
  }
  return found->second->ServiceIsReady();
}

ServiceCallResult ServiceClientRegistry::call(
    const std::string& service_name, const std::string& request_bytes,
    std::chrono::seconds timeout) {
  ServiceCallResult result;
  if (service_name.empty()) {
    result.error = "service name is empty";
    return result;
  }

  RawClientPtr client;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    client = ensureClientLocked(service_name);
  }
  if (client == nullptr) {
    result.error = "failed to create service client";
    return result;
  }

  auto request =
      std::make_shared<::autolink::message::RawMessage>(request_bytes);
  const auto response = client->SendRequest(request, timeout);
  if (response == nullptr) {
    result.error = "service call failed or timed out";
    return result;
  }

  result.response_bytes = response->message;
  result.ok = true;
  return result;
}

}  // namespace integration
}  // namespace autoviz
