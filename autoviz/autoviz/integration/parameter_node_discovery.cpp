/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/integration/parameter_node_discovery.hpp"

#include <algorithm>

#include "autolink/parameter/parameter_service_names.hpp"
#include "autolink/proto/role_attributes.pb.h"
#include "autolink/service_discovery/topology_manager.hpp"

namespace autoviz {
namespace integration {
namespace {

std::string ListParametersServiceSuffix() {
  return std::string(autolink::SERVICE_NAME_DELIMITER) +
         std::string(autolink::LIST_PARAMETERS_SERVICE_NAME);
}

}  // namespace

std::vector<std::string> ListParameterServerNodes() {
  auto* topology = autolink::service_discovery::TopologyManager::Instance();
  if (topology == nullptr || topology->service_manager() == nullptr) {
    return {};
  }

  std::vector<autolink::proto::RoleAttributes> servers;
  topology->service_manager()->GetServers(&servers);

  const std::string suffix = ListParametersServiceSuffix();
  std::vector<std::string> nodes;
  nodes.reserve(servers.size());
  for (const autolink::proto::RoleAttributes& server : servers) {
    const std::string& service_name = server.service_name();
    if (service_name.size() <= suffix.size()) {
      continue;
    }
    if (service_name.compare(service_name.size() - suffix.size(), suffix.size(),
                             suffix) != 0) {
      continue;
    }
    nodes.push_back(service_name.substr(0, service_name.size() - suffix.size()));
  }

  std::sort(nodes.begin(), nodes.end());
  nodes.erase(std::unique(nodes.begin(), nodes.end()), nodes.end());
  return nodes;
}

}  // namespace integration
}  // namespace autoviz
