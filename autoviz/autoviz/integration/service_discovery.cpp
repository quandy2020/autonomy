/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/integration/service_discovery.hpp"

#include <algorithm>

#include "autolink/common/types.hpp"
#include "autolink/message/raw_message.hpp"
#include "autolink/proto/role_attributes.pb.h"
#include "autolink/service_discovery/topology_manager.hpp"

namespace autoviz {
namespace integration {
namespace {

constexpr char kRawMessageType[] = "autolink.message.RawMessage";

bool ResolveChannelMessageType(const std::string& channel_name,
                               std::string* message_type) {
  if (message_type == nullptr) {
    return false;
  }
  message_type->clear();

  auto* topology = autolink::service_discovery::TopologyManager::Instance();
  if (topology == nullptr || topology->channel_manager() == nullptr) {
    return false;
  }

  const auto channel_manager = topology->channel_manager();
  if (channel_manager == nullptr) {
    return false;
  }
  std::string resolved_type;
  if (channel_manager->HasWriter(channel_name)) {
    channel_manager->GetMsgType(channel_name, &resolved_type);
  }
  if (resolved_type.empty() || resolved_type == kRawMessageType) {
    std::vector<autolink::proto::RoleAttributes> writers;
    channel_manager->GetWritersOfChannel(channel_name, &writers);
    for (const autolink::proto::RoleAttributes& attr : writers) {
      if (!attr.message_type().empty() &&
          attr.message_type() != kRawMessageType) {
        resolved_type = attr.message_type();
        break;
      }
    }
  }
  if (resolved_type.empty() || resolved_type == kRawMessageType) {
    std::vector<autolink::proto::RoleAttributes> readers;
    channel_manager->GetReadersOfChannel(channel_name, &readers);
    for (const autolink::proto::RoleAttributes& attr : readers) {
      if (!attr.message_type().empty() &&
          attr.message_type() != kRawMessageType) {
        resolved_type = attr.message_type();
        break;
      }
    }
  }
  if (resolved_type.empty() || resolved_type == kRawMessageType) {
    return false;
  }
  *message_type = resolved_type;
  return true;
}

bool ResolveFromServiceServer(const std::string& service_name,
                              std::string* message_type) {
  if (message_type == nullptr) {
    return false;
  }
  message_type->clear();

  auto* topology = autolink::service_discovery::TopologyManager::Instance();
  if (topology == nullptr || topology->service_manager() == nullptr) {
    return false;
  }

  std::vector<autolink::proto::RoleAttributes> servers;
  topology->service_manager()->GetServers(&servers);
  for (const autolink::proto::RoleAttributes& attr : servers) {
    if (attr.service_name() != service_name) {
      continue;
    }
    if (!attr.message_type().empty() && attr.message_type() != kRawMessageType) {
      *message_type = attr.message_type();
      return true;
    }
  }
  return false;
}

}  // namespace

std::vector<std::string> ListServices() {
  auto* topology = autolink::service_discovery::TopologyManager::Instance();
  if (topology == nullptr || topology->service_manager() == nullptr) {
    return {};
  }

  std::vector<autolink::proto::RoleAttributes> servers;
  topology->service_manager()->GetServers(&servers);

  std::vector<std::string> names;
  names.reserve(servers.size());
  for (const autolink::proto::RoleAttributes& server : servers) {
    if (!server.service_name().empty()) {
      names.push_back(server.service_name());
    }
  }
  std::sort(names.begin(), names.end());
  names.erase(std::unique(names.begin(), names.end()), names.end());
  return names;
}

bool ResolveServiceMessageType(const std::string& service_name, bool request,
                               std::string* message_type) {
  if (service_name.empty() || message_type == nullptr) {
    return false;
  }

  if (ResolveFromServiceServer(service_name, message_type)) {
    return true;
  }

  const std::string channel =
      service_name +
      std::string(request ? autolink::SRV_CHANNEL_REQ_SUFFIX
                          : autolink::SRV_CHANNEL_RES_SUFFIX);
  return ResolveChannelMessageType(channel, message_type);
}

}  // namespace integration
}  // namespace autoviz
