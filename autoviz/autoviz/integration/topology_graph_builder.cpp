/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/integration/topology_graph_builder.hpp"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#include "autolink/common/types.hpp"
#include "autolink/proto/role_attributes.pb.h"
#include "autolink/service_discovery/topology_manager.hpp"

namespace autoviz {
namespace integration {
namespace {

constexpr char kNodePrefix[] = "n:";
constexpr char kChannelPrefix[] = "c:";
constexpr char kServicePrefix[] = "s:";

std::string MakeNodeId(const std::string& node_name) {
  return kNodePrefix + node_name;
}

std::string MakeChannelId(const std::string& channel_name) {
  return kChannelPrefix + channel_name;
}

std::string MakeServiceId(const std::string& service_name) {
  return kServicePrefix + service_name;
}

bool IsInternalServiceChannel(const std::string& channel_name) {
  const std::size_t req_len = strlen(autolink::SRV_CHANNEL_REQ_SUFFIX);
  const std::size_t res_len = strlen(autolink::SRV_CHANNEL_RES_SUFFIX);
  if (channel_name.size() >= req_len &&
      channel_name.compare(channel_name.size() - req_len, req_len,
                           autolink::SRV_CHANNEL_REQ_SUFFIX) == 0) {
    return true;
  }
  if (channel_name.size() >= res_len &&
      channel_name.compare(channel_name.size() - res_len, res_len,
                           autolink::SRV_CHANNEL_RES_SUFFIX) == 0) {
    return true;
  }
  return false;
}

std::string ToLowerAscii(std::string value) {
  for (char& ch : value) {
    ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
  }
  return value;
}

bool MatchesFilter(const std::string& haystack, const std::string& needle_lower) {
  if (needle_lower.empty()) {
    return true;
  }
  return ToLowerAscii(haystack).find(needle_lower) != std::string::npos;
}

std::string ExtractFirstPrefix(const std::string& channel_name) {
  if (channel_name.empty() || channel_name.front() != '/') {
    return channel_name;
  }
  const std::size_t second = channel_name.find('/', 1);
  if (second == std::string::npos) {
    return channel_name;
  }
  return channel_name.substr(0, second);
}

bool MatchesPrefixFilter(const std::string& channel_name,
                         const std::string& prefix_filter) {
  if (prefix_filter.empty()) {
    return true;
  }
  if (prefix_filter == channel_name) {
    return true;
  }
  if (prefix_filter.back() == '/') {
    return channel_name.rfind(prefix_filter, 0) == 0;
  }
  return channel_name.rfind(prefix_filter + "/", 0) == 0 ||
         channel_name == prefix_filter;
}

void AppendHash(std::ostringstream* stream, const std::string& value) {
  if (stream == nullptr) {
    return;
  }
  (*stream) << value << '\n';
}

}  // namespace

TopologyGraph BuildTopologyGraph(const TopologyGraphBuildOptions& options) {
  TopologyGraph graph;
  auto* topology = autolink::service_discovery::TopologyManager::Instance();
  if (topology == nullptr) {
    return graph;
  }

  const std::string filter_lower = ToLowerAscii(options.filter_text);
  std::unordered_map<std::string, GraphVertex> vertex_map;
  std::vector<GraphEdge> edges;
  std::ostringstream hash_stream;

  auto ensure_node = [&](const std::string& node_name) -> const std::string& {
    const std::string id = MakeNodeId(node_name);
    if (vertex_map.find(id) == vertex_map.end()) {
      GraphVertex vertex;
      vertex.id = id;
      vertex.kind = GraphVertexKind::kNode;
      vertex.label = node_name;
      vertex_map.emplace(id, std::move(vertex));
    }
    return vertex_map.find(id)->first;
  };

  auto ensure_channel = [&](const std::string& channel_name,
                            const std::string& message_type) -> const std::string& {
    const std::string id = MakeChannelId(channel_name);
    auto it = vertex_map.find(id);
    if (it == vertex_map.end()) {
      GraphVertex vertex;
      vertex.id = id;
      vertex.kind = GraphVertexKind::kChannel;
      vertex.label = channel_name;
      vertex.detail = message_type;
      it = vertex_map.emplace(id, std::move(vertex)).first;
    } else if (it->second.detail.empty() && !message_type.empty()) {
      it->second.detail = message_type;
    }
    return it->first;
  };

  auto ensure_service = [&](const std::string& service_name,
                            const std::string& message_type) -> const std::string& {
    const std::string id = MakeServiceId(service_name);
    auto it = vertex_map.find(id);
    if (it == vertex_map.end()) {
      GraphVertex vertex;
      vertex.id = id;
      vertex.kind = GraphVertexKind::kService;
      vertex.label = service_name;
      vertex.detail = message_type;
      it = vertex_map.emplace(id, std::move(vertex)).first;
    } else if (it->second.detail.empty() && !message_type.empty()) {
      it->second.detail = message_type;
    }
    return it->first;
  };

  auto channel_visible = [&](const std::string& channel_name) -> bool {
    if (IsInternalServiceChannel(channel_name)) {
      return false;
    }
    if (!MatchesPrefixFilter(channel_name, options.prefix_filter)) {
      return false;
    }
    return MatchesFilter(channel_name, filter_lower);
  };

  if (topology->channel_manager() != nullptr) {
    const auto channel_manager = topology->channel_manager();
    std::vector<std::string> channel_names;
    channel_manager->GetChannelNames(&channel_names);
    std::sort(channel_names.begin(), channel_names.end());

    std::vector<autolink::proto::RoleAttributes> writers;
    std::vector<autolink::proto::RoleAttributes> readers;
    channel_manager->GetWriters(&writers);
    channel_manager->GetReaders(&readers);

    struct ChannelEndpoints {
      std::vector<std::string> writer_nodes;
      std::vector<std::string> reader_nodes;
      std::string message_type;
    };
    std::unordered_map<std::string, ChannelEndpoints> endpoints;

    for (const autolink::proto::RoleAttributes& writer : writers) {
      if (writer.channel_name().empty() || writer.node_name().empty()) {
        continue;
      }
      ChannelEndpoints& entry = endpoints[writer.channel_name()];
      entry.writer_nodes.push_back(writer.node_name());
      if (entry.message_type.empty() && !writer.message_type().empty()) {
        entry.message_type = writer.message_type();
      }
    }
    for (const autolink::proto::RoleAttributes& reader : readers) {
      if (reader.channel_name().empty() || reader.node_name().empty()) {
        continue;
      }
      ChannelEndpoints& entry = endpoints[reader.channel_name()];
      entry.reader_nodes.push_back(reader.node_name());
      if (entry.message_type.empty() && !reader.message_type().empty()) {
        entry.message_type = reader.message_type();
      }
    }

    for (const std::string& channel_name : channel_names) {
      if (!channel_visible(channel_name)) {
        continue;
      }
      const auto it = endpoints.find(channel_name);
      if (it == endpoints.end()) {
        continue;
      }

      const ChannelEndpoints& entry = it->second;
      bool endpoint_matches = MatchesFilter(channel_name, filter_lower);
      if (!endpoint_matches) {
        for (const std::string& node_name : entry.writer_nodes) {
          if (MatchesFilter(node_name, filter_lower)) {
            endpoint_matches = true;
            break;
          }
        }
      }
      if (!endpoint_matches) {
        for (const std::string& node_name : entry.reader_nodes) {
          if (MatchesFilter(node_name, filter_lower)) {
            endpoint_matches = true;
            break;
          }
        }
      }
      if (!endpoint_matches) {
        continue;
      }

      std::unordered_set<std::string> writer_nodes(entry.writer_nodes.begin(),
                                                   entry.writer_nodes.end());
      std::unordered_set<std::string> reader_nodes(entry.reader_nodes.begin(),
                                                   entry.reader_nodes.end());

      if (options.show_channels) {
        const std::string& channel_id =
            ensure_channel(channel_name, entry.message_type);
        AppendHash(&hash_stream, channel_id);
        for (const std::string& node_name : writer_nodes) {
          const std::string& node_id = ensure_node(node_name);
          edges.push_back({node_id, channel_id, GraphEdgeKind::kPublish});
          AppendHash(&hash_stream, node_id + ">" + channel_id);
        }
        for (const std::string& node_name : reader_nodes) {
          const std::string& node_id = ensure_node(node_name);
          edges.push_back({channel_id, node_id, GraphEdgeKind::kSubscribe});
          AppendHash(&hash_stream, channel_id + ">" + node_id);
        }
      } else {
        for (const std::string& writer_node : writer_nodes) {
          const std::string& writer_id = ensure_node(writer_node);
          AppendHash(&hash_stream, writer_id);
          for (const std::string& reader_node : reader_nodes) {
            const std::string& reader_id = ensure_node(reader_node);
            edges.push_back({writer_id, reader_id, GraphEdgeKind::kRelay});
            AppendHash(&hash_stream, writer_id + "~>" + reader_id);
          }
        }
      }
    }
  }

  if (options.show_services && topology->service_manager() != nullptr) {
    const auto service_manager = topology->service_manager();
    std::vector<autolink::proto::RoleAttributes> servers;
    service_manager->GetServers(&servers);
    std::sort(servers.begin(), servers.end(),
              [](const autolink::proto::RoleAttributes& lhs,
                 const autolink::proto::RoleAttributes& rhs) {
                return lhs.service_name() < rhs.service_name();
              });

    for (const autolink::proto::RoleAttributes& server : servers) {
      const std::string& service_name = server.service_name();
      if (service_name.empty()) {
        continue;
      }
      if (!MatchesFilter(service_name, filter_lower) &&
          !MatchesFilter(server.node_name(), filter_lower)) {
        std::vector<autolink::proto::RoleAttributes> clients;
        service_manager->GetClients(service_name, &clients);
        bool client_matches = false;
        for (const autolink::proto::RoleAttributes& client : clients) {
          if (MatchesFilter(client.node_name(), filter_lower)) {
            client_matches = true;
            break;
          }
        }
        if (!client_matches) {
          continue;
        }
      }

      const std::string& service_id =
          ensure_service(service_name, server.message_type());
      AppendHash(&hash_stream, service_id);

      if (!server.node_name().empty()) {
        const std::string& node_id = ensure_node(server.node_name());
        edges.push_back({node_id, service_id, GraphEdgeKind::kServiceServer});
        AppendHash(&hash_stream, node_id + "srv>" + service_id);
      }

      std::vector<autolink::proto::RoleAttributes> clients;
      service_manager->GetClients(service_name, &clients);
      std::unordered_set<std::string> client_nodes;
      for (const autolink::proto::RoleAttributes& client : clients) {
        if (client.node_name().empty()) {
          continue;
        }
        client_nodes.insert(client.node_name());
      }
      for (const std::string& client_node : client_nodes) {
        const std::string& node_id = ensure_node(client_node);
        edges.push_back({node_id, service_id, GraphEdgeKind::kServiceClient});
        AppendHash(&hash_stream, node_id + "cli>" + service_id);
      }
    }
  }

  if (topology->node_manager() != nullptr) {
    std::vector<autolink::proto::RoleAttributes> nodes;
    topology->node_manager()->GetNodes(&nodes);
    for (const autolink::proto::RoleAttributes& node : nodes) {
      if (node.node_name().empty()) {
        continue;
      }
      if (!MatchesFilter(node.node_name(), filter_lower)) {
        continue;
      }
      ensure_node(node.node_name());
    }
  }

  graph.vertices.reserve(vertex_map.size());
  for (auto& entry : vertex_map) {
    graph.vertices.push_back(std::move(entry.second));
  }
  std::sort(graph.vertices.begin(), graph.vertices.end(),
            [](const GraphVertex& lhs, const GraphVertex& rhs) {
              if (lhs.kind != rhs.kind) {
                return static_cast<int>(lhs.kind) < static_cast<int>(rhs.kind);
              }
              return lhs.label < rhs.label;
            });
  graph.edges = std::move(edges);
  graph.topology_hash = hash_stream.str();
  return graph;
}

std::vector<std::string> ListChannelPrefixGroups() {
  auto* topology = autolink::service_discovery::TopologyManager::Instance();
  if (topology == nullptr || topology->channel_manager() == nullptr) {
    return {};
  }

  std::vector<std::string> channel_names;
  topology->channel_manager()->GetChannelNames(&channel_names);

  std::unordered_set<std::string> prefixes;
  for (const std::string& channel_name : channel_names) {
    if (IsInternalServiceChannel(channel_name)) {
      continue;
    }
    prefixes.insert(ExtractFirstPrefix(channel_name));
  }

  std::vector<std::string> groups(prefixes.begin(), prefixes.end());
  std::sort(groups.begin(), groups.end());
  return groups;
}

}  // namespace integration
}  // namespace autoviz
